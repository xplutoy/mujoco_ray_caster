# 源码阅读指南

整个工程共 14 个源文件，建议分 4 个阶段阅读，从入口到核心，从框架到细节。

---

## 阶段一：入口与注册（1 个文件，5 分钟）

理解插件如何被 MuJoCo 加载。

| 文件 | 作用 |
|------|------|
| `register.cc` | 唯一入口，`mjPLUGIN_LIB_INIT` 宏触发三个 `RegisterPlugin()` |

**关键函数：**

| 函数 | 行号 | 作用 |
|------|------|------|
| `mjPLUGIN_LIB_INIT` | register.cc:8 | MuJoCo 加载 `.so` 时自动调用，注册三个插件 |

这是整个工程的起点。MuJoCo 在加载 `libsensor_raycaster.so` 时会找到这个宏导出的函数，然后三个插件就进入了 MuJoCo 的插件注册表。

---

## 阶段二：Plugin 胶水层（5 个文件，30 分钟）

理解 MJCF 配置如何变成运行时对象。**三个插件的结构完全对称**，先读一个再读其余两个就是重复模式。

建议顺序：**`ray_plugin.h` → `ray_plugin.cc` → `ray_caster_camera_plugin.h/.cc`**（只需细读 camera 版本，其余两个是复制结构）。

### 2.1 框架基类

| 文件 | 作用 |
|------|------|
| `ray_plugin.h` | `RayPlugin` 基类 + `SensorData` + `VisCfg` + `DataType` 全部定义 |
| `ray_plugin.cc` | `RayPlugin` 的 `Compute`/`getBaseCfg`/`initSensor`/`Visualize` 实现 |

**关键函数：**

| 函数 | 位置 | 作用 | 读懂后的收获 |
|------|------|------|------------|
| `SensorData::fromStr` | ray_plugin.h:180-209 | 解析 `"image inv_noise"` 等字符串为 `DataType` + 三个标记 | **理解可组合数据输出的核心** |
| `computeDateSize` | ray_plugin.cc:58-77 | 计算一个 `sensor_data_types` 需要多少 sensordata 空间 | 理解 sensordata 内存布局 |
| `RayPlugin::Compute` | ray_plugin.cc:88-111 | 每步执行入口：`compute_distance()` → 逐一调用 `SensorData.func` | **理解每帧数据流的全貌** |
| `RayPlugin::getBaseCfg` | ray_plugin.cc:113-202 | 从 MJCF XML 读取所有通用配置 | 理解 MJCF 属性和代码的对应关系 |
| `RayPlugin::initSensor` | ray_plugin.cc:204-397 | 创建 SensorData lambda、设置噪声、写入 plugin_state | **最关键的胶水函数——把配置翻译成运行时 lambda** |

`initSensor` 是**整个 Plugin 层最重要的函数**。它做三件事：
1. 为每个 `SensorData` 类型创建 lambda（决定数据怎么从 `dist[]`/`pos_w[]` 写入 `sensordata`）
2. 根据配置设置噪声对象（switch 5 种噪声类型）
3. 把数据布局写入 `plugin_state`（让用户能定位数据）

### 2.2 Camera 插件（重点读）

| 文件 | 作用 |
|------|------|
| `ray_caster_camera_plugin.h` | Camera 专属配置属性定义 |
| `ray_caster_camera_plugin.cc` | `Create` + 构造函数 + `RegisterPlugin` |

**关键函数：**

| 函数 | 位置 | 作用 |
|------|------|------|
| `RayCasterCameraPlugin::构造函数` | ray_caster_camera_plugin.cc:35-72 | 读 focal_length/aperture/baseline，创建 `RayCasterCamera` |
| `RegisterPlugin` | ray_caster_camera_plugin.cc:74-151 | 定义 `nstate`/`nsensordata`/`init`/`destroy`/`compute`/`visualize` 六个回调 |

`RegisterPlugin` 展示了 MuJoCo 插件的标准注册模式：6 个 C 函数指针回调，通过 `reinterpret_cast` 把 `d->plugin_data[instance]` 转回 C++ 对象。Lidar 和 base 版本结构完全一样，可快速扫读。

### 2.3 Base/Lidar 插件（快速扫读）

| 文件 | 作用 |
|------|------|
| `ray_caster_plugin.cc` | Base 版——读 resolution/size/type |
| `ray_caster_lidar_plugin.cc` | Lidar 版——读 fov_h/fov_v/size/dis_range |

与 Camera 版结构对称，只需注意各自特有的配置属性。

---

## 阶段三：Core 核心算法层（5 个文件，核心重点，60 分钟）

这是整个工程的灵魂。射线生成、投射、噪声、数据输出全在这里。

建议顺序：**`Noise.hpp` → `RayNoise.hpp` → `RayCaster.h` → `RayCaster.cpp` → `RayCasterCamera.h/.cpp` → `RayCasterLidar.h/.cpp`**

### 3.1 噪声基础设施

| 文件 | 作用 |
|------|------|
| `raycaster_src/Noise.hpp` | 噪声基类 + 三种通用分布噪声 |
| `raycaster_src/RayNoise.hpp` | 三种射线专用噪声 |

**关键函数：**

| 函数 | 位置 | 作用 | 读懂后的收获 |
|------|------|------|------------|
| `Noise::produce_noise` | Noise.hpp:28-29 | 空实现——基类不做任何噪声 | 理解噪声的两阶段设计 |
| `Noise::produce_united_noise` | Noise.hpp:53 | 空实现——基类不做联合噪声 | 联合噪声是子类专属 |
| `DistributionNoise::produce_noise` | Noise.hpp:74-76 | `data += dist(gen)`——通用分布叠加 | 逐射线噪声的基础模式 |
| `RayNoise1::produce_noise` | RayNoise.hpp:39-44 | 均匀偏移 + 概率置零 | **理解 noise1 的两层效果** |
| `RayNoise2::produce_united_noise` | RayNoise.hpp:72-132 | 遍历每个像素的 8 邻域，算最大深度差→入射角→概率丢失 | **理解角度依赖丢失的完整算法** |
| `RayNoise2::getDegAngle` | RayNoise.hpp:142-169 | 用点积+叉积→atan2 计算向量夹角 | 理解无 Eigen 的向量计算实现 |
| `StereoNoise::produce_united_noise` | RayNoise.hpp:189-201 | `(1-E_normalized)^power` 概率丢失 | **理解 stereo 能量丢失公式** |

**噪声设计的关键洞察**：噪声分两阶段——`produce_noise`（逐射线独立）在 `compute_ray` 内对每条射线调用；`produce_united_noise`（联合跨射线）在 `compute_distance` 最后对所有射线一起调用。这是噪声体系的核心架构决策。

### 3.2 RayCaster 基类（工程的核心枢纽）

| 文件 | 作用 |
|------|------|
| `raycaster_src/RayCaster.h` | **全工程最重要的文件**——基类定义 + 所有数据输出模板 |
| `raycaster_src/RayCaster.cpp` | 基类实现：初始化、射线计算、命中计算、可视化 |

**关键函数（按调用顺序排列）：**

| 函数 | 位置 | 作用 | 读懂后的收获 |
|------|------|------|------------|
| `RayCaster::_init` | RayCaster.cpp:37-91 | 分配所有内存数组、设置参数 | 理解所有内部数组的用途和大小 |
| `RayCaster::create_rays` | RayCaster.cpp:216-257 | 基础版射线生成——网格+方向 | 理解 `_ray_vec` / `_ray_vec_offset` 的含义 |
| `RayCaster::compute_ray_vec` | RayCaster.cpp:193-214 | `_ray_vec`→`ray_vec`（相机坐标系→世界坐标系） | 理解 `mju_mulMatVec3` 的坐标变换 |
| `RayCaster::compute_ray` | RayCaster.cpp:259-295 | **核心——对每条射线调用 `mj_ray`** | **理解射线投射、距离计算、噪声、丢失的完整流程** |
| `RayCaster::compute_distance` | RayCaster.cpp:297-326 | 每帧总调度：射线→命中→联合噪声 | **理解完整的每帧执行管线** |
| `RayCaster::compute_hit` | RayCaster.cpp:553-573 | 计算世界坐标系命中位置 `pos_w` | 理解命中点如何从射线+距离得出 |
| `RayCaster::compute_hit_b` | RayCaster.cpp:575-595 | 计算自身坐标系命中位置 `pos_b` | 理解两种坐标系命中位置的区别 |
| `RayCaster::compute_loss_ray` | RayCaster.h:193-209 | 入射角小于阈值则标记丢失 | 理解 loss_angle 机制 |

**`compute_ray` 是最值得细读的函数**（RayCaster.cpp:259-295）。它浓缩了核心逻辑：

```
对每条射线 idx [start, end):
  1. 计算射线起点 pnt = pos + ray_vec_offset（如果有偏移）
  2. mj_ray(m, d, pnt, ray_vec, ...) → dist_ratio[idx], geomids[idx], ray_normal[idx]
  3. 处理边界值：
     geomid=-1 → dist_ratio=1（未命中→最大距离）
     dist_ratio>1 → geomid=-1（超出范围→未命中）
     dist_ratio<deep_min_ratio → 限制最小值
  4. dist[idx] = deep_max * dist_ratio[idx]  （比例→绝对距离）
  5. _noise->produce_noise(dist[idx])         （逐射线噪声）
  6. clamp dist[idx] 到 [deep_min, deep_max]
  7. compute_loss_ray(idx)                    （入射角丢失检查）
```

**数据输出模板（在 RayCaster.h 中定义，不在 .cpp 中）：**

| 函数 | 位置 | 作用 |
|------|------|------|
| `get_data<T>` | RayCaster.h:378-412 | 原始距离输出，noise/inf_max 逻辑 |
| `get_data_normalized<T>` | RayCaster.h:334-376 | 归一化深度输出，inv/scale 逻辑 |
| `get_distance_to_image_plane<T>` | RayCaster.h:424-472 | 投影距离：`s × cosθ`，`cosθ = |dz|/||d||` |
| `get_distance_to_image_plane_normalized<T>` | RayCaster.h:474-532 | 投影距离归一化 |
| `_get_data_pos_dim1<T>` | RayCaster.h:298-313 | 位置数据输出（pos_w/pos_b） |

这些模板都遵循同一个模式：遍历 `nray`，对每个 `idx` 判断 `geomids[idx]<0`（未命中）、`is_lost[idx]`（丢失）、`is_noise==has_noise`（选噪声版本），然后从 `dist[]`/`dist_ratio[]`/`pos_w[]`/`pos_b[]` 中取值输出。

**`is_noise == has_noise` 这个条件是理解噪声版本切换的关键**：

- `has_noise` 是传感器全局标记（是否配置了噪声）
- `is_noise` 是当前 `SensorData` 的标记（这个输出要不要噪声）
- 两者相等 → 输出带噪版本（`dist[idx]`，经过 `produce_noise` 处理）
- 两者不等 → 输出无噪版本（`dist_ratio[idx] * deep_max`，原始值）

### 3.3 Camera 子类

| 文件 | 作用 |
|------|------|
| `raycaster_src/RayCasterCamera.h` | Camera 配置 + stereo 结构 |
| `raycaster_src/RayCasterCamera.cpp` | 虚像平面射线生成 + stereo 验证 |

**关键函数：**

| 函数 | 位置 | 作用 | 读懂后的收获 |
|------|------|------|------------|
| `RayCasterCamera::init` | RayCasterCamera.cpp:10-38 | 计算像素尺寸、纵横比 | 理解焦距→像素尺寸的换算 |
| `compute_ray_vec_virtual_plane` | RayCasterCamera.cpp:41-60 | 针孔模型射线生成 | **理解相机射线如何从焦距+像素位置得出** |
| `compute_stereo_ray` | RayCasterCamera.cpp:74-145 | 双目验证 + 光路能量计算 | **理解 stereo 遮挡阴影和能量丢失** |
| `RayCasterCamera::compute_distance` | RayCasterCamera.cpp:147-177 | 基类计算后追加 stereo 计算 | 理解 stereo 是后处理步骤 |

**`compute_ray_vec_virtual_plane` 是理解相机模型的核心**（RayCasterCamera.cpp:41-60）：

```
对每个像素 (i, j):
  x = (j + 0.5 - h/2) × h_pixel_size   // 水平偏移
  y = (h/2 - i - 0.5) × v_pixel_size    // 垂直偏移（Y翻转）
  z = -focal_length                       // 相机方向是 -Z
  point = (x, y, z)
  dir = normalize(point)                  // 单位方向
  _ray_vec = dir × deep_max               // 缩放到最大距离
```

这就是针孔相机模型的射线生成：每个像素在虚像平面上有一个点，从相机原点指向该点的方向就是射线方向。

**`compute_stereo_ray` 的逻辑**（RayCasterCamera.cpp:74-145）：

```
对每个已命中的射线 idx:
  1. 左眼位置 = pos + mat × (-baseline/2, 0, 0)
     右眼位置 = pos + mat × (+baseline/2, 0, 0)
  2. 从左眼和右眼分别投射射线到命中点
  3. 若任一眼睛的 mj_ray 返回值 ≠ 1.0 → 被遮挡 → 标记无效
  4. 若开启了 loss_angle：
     计算反射光路法线 (V+L)/2 的归一化
     ray_energy = min(cos_左反射法线, cos_右反射法线)
     cos_光源 < loss_angle_cos 或 energy < min_energy → 丢失
```

### 3.4 Lidar 子类

| 文件 | 作用 |
|------|------|
| `raycaster_src/RayCasterLidar.h` | Lidar 配置 |
| `raycaster_src/RayCasterLidar.cpp` | FOV 四元数射线生成 |

**关键函数：**

| 函数 | 位置 | 作用 |
|------|------|------|
| `RayCasterLidar::create_rays` | RayCasterLidar.cpp:30-55 | 用 `mju_axisAngle2Quat` + `mju_rotVecQuat` 旋转参考向量 |

逻辑简洁：参考向量 `(0, 0, -deep_max)` 按水平/垂直角度用四元数旋转，生成 FOV 内的射线网格。

---

## 阶段四：整体串联（复习，10 分钟）

读完所有文件后，回到 `ray_plugin.cc` 的 `Compute` 函数（ray_plugin.cc:88-111），用 10 行代码串起全流程：

```cpp
void RayPlugin::Compute(...) {
  n_step++; if (n_step < n_step_update) return;   // ① 降频控制
  ray_caster->compute_distance();                  // ② 核心：射线→命中→噪声
  for (i = 0; i < n_sensor_data; i++) {
    sensor_data_list[i].func(sensordata + ...);    // ③ 每种数据类型写入 sensordata
  }
}
```

展开 `compute_distance()`（RayCaster.cpp:297-326）：

```cpp
void RayCaster::compute_distance() {
  compute_ray_vec();           // ②a 相机坐标系→世界坐标系
  compute_ray(0, nray);        // ②b mj_ray 投射 + produce_noise + loss_angle
  compute_hit();               // ②c 世界坐标命中位置（如果需要）
  compute_hit_b();             // ②c 自身坐标命中位置（如果需要）
  _noise->produce_united_noise(); // ②d 联合噪声（跨射线效应）
}
```

Camera 版追加（RayCasterCamera.cpp:147-177）：

```cpp
void RayCasterCamera::compute_distance() {
  RayCaster::compute_distance();  // 先做基类全部流程
  if (baseline > 0)
    compute_stereo_ray(0, nray);  // 再做双目验证
}
```

---

## 全工程主要函数一览表

按**调用链深度**从浅到深排列：

| 层级 | 函数 | 文件:行 | 一句话描述 |
|------|------|---------|-----------|
| 0 | `mjPLUGIN_LIB_INIT` | register.cc:8 | MuJoCo 加载入口，注册三个插件 |
| 1 | `RegisterPlugin` | ray_caster_camera_plugin.cc:74 | 定义 MuJoCo 的 6 个回调函数指针 |
| 1 | `RayCasterCameraPlugin::构造函数` | ray_caster_camera_plugin.cc:35 | 读 MJCF → 创建 `RayCasterCamera` → `initSensor` |
| 2 | `RayPlugin::Compute` | ray_plugin.cc:88 | 每步入口：`compute_distance()` → `SensorData.func()` |
| 2 | `RayPlugin::initSensor` | ray_plugin.cc:204 | 创建 lambda、设置噪声、写 plugin_state |
| 2 | `SensorData::fromStr` | ray_plugin.h:180 | 解析 `"image_noise_inf_zero"` 为类型+标记 |
| 3 | `RayCaster::compute_distance` | RayCaster.cpp:297 | 总调度：射线→命中→联合噪声 |
| 3 | `RayCasterCamera::compute_distance` | RayCasterCamera.cpp:147 | 基类计算后追加 stereo 验证 |
| 4 | `RayCaster::compute_ray_vec` | RayCaster.cpp:193 | `_ray_vec`→`ray_vec`，相机→世界坐标系 |
| 4 | `RayCaster::compute_ray` | RayCaster.cpp:259 | **核心**：`mj_ray` + 噪声 + 丢失 |
| 4 | `RayCaster::compute_hit` | RayCaster.cpp:553 | 计算 `pos_w`：起点+方向×比例 |
| 4 | `RayCaster::compute_hit_b` | RayCaster.cpp:575 | 计算 `pos_b`：相机坐标系版本 |
| 5 | `compute_ray_vec_virtual_plane` | RayCasterCamera.cpp:41 | 针孔模型：像素→虚像平面→射线方向 |
| 5 | `RayCasterLidar::create_rays` | RayCasterLidar.cpp:30 | FOV 四元数旋转生成射线 |
| 5 | `compute_stereo_ray` | RayCasterCamera.cpp:74 | 双目遮挡验证 + 光路能量 |
| 5 | `compute_loss_ray` | RayCaster.h:193 | 入射角 < 阈值 → 标记丢失 |
| 5 | `Noise::produce_noise` | Noise.hpp:28 | 逐射线噪声空基类 |
| 5 | `RayNoise1::produce_noise` | RayNoise.hpp:39 | 均匀偏移+概率置零 |
| 5 | `RayNoise2::produce_united_noise` | RayNoise.hpp:72 | 8邻域角度→概率丢失 |
| 5 | `StereoNoise::produce_united_noise` | RayNoise.hpp:189 | `(1-E)^power` 概率丢失 |
| 6 | `get_data<T>` | RayCaster.h:378 | 原始距离输出模板 |
| 6 | `get_data_normalized<T>` | RayCaster.h:334 | 归一化深度输出模板 |
| 6 | `get_distance_to_image_plane<T>` | RayCaster.h:424 | 投影距离：`s × cosθ` |

---

## 内部数组用途速查

| 数组 | 大小 | 含义 | 何时写入 |
|------|------|------|----------|
| `_ray_vec` | nray×3 | 相机坐标系下的射线方向（初始化时生成） | `create_rays` / `compute_ray_vec_virtual_plane` |
| `_ray_vec_offset` | nray×3 | 相机坐标系下的射线起点偏移（仅 offset 类型用） | `create_rays` |
| `ray_vec` | nray×3 | 世界坐标系下的射线方向（每帧从 `_ray_vec` 变换） | `compute_ray_vec` |
| `ray_vec_offset` | nray×3 | 世界坐标系下的起点偏移 | `compute_ray_vec` |
| `dist_ratio` | nray | `mj_ray` 返回值（0~1 的比例，= 实际距离/deep_max） | `compute_ray` |
| `dist` | nray | 绝对距离 = deep_max × dist_ratio，**经过噪声处理** | `compute_ray` |
| `geomids` | nray | 命中的 geom id，-1 表示未命中 | `compute_ray` |
| `is_lost` | nray | loss_angle 导致的丢失 | `compute_loss_ray` |
| `is_lost_noise` | nray | StereoNoise 导致的丢失 | `StereoNoise::produce_united_noise` |
| `pos_w` | nray×3 | 世界坐标系命中位置 | `compute_hit` |
| `pos_b` | nray×3 | 自身坐标系命中位置 | `compute_hit_b` |
| `ray_normal` | nray×3 | 命中表面法线（MuJoCo >= 3.5.0） | `mj_ray` 输出 |
| `ray_energy` | nray | 光路能量 = min(cos_左反射, cos_右反射) | `compute_stereo_ray` |

**`dist` vs `dist_ratio` 是最常混淆的**：`dist_ratio` 是无噪声原始值（0~1比例），`dist` 是经过噪声处理的绝对距离值（deep_min~deep_max）。所有数据输出函数通过 `is_noise == has_noise` 选择使用哪一个。

---

## 调用链完整图

```
mj_step(model, data)
  └─ plugin.compute(instance)
     └─ RayPlugin::Compute(m, d, instance)
        ├─ ray_caster->compute_distance()
        │  ├─ compute_ray_vec()                    ← 相机→世界坐标变换
        │  │  └─ [多线程] ray_task_func → compute_ray(start, end)
        │  │     ├─ mj_ray() → dist_ratio, geomids, ray_normal
        │  │     ├─ dist = deep_max × dist_ratio
        │  │     ├─ _noise->produce_noise(dist[i]) ← 逐射线噪声
        │  │     ├─ clamp dist[i]
        │  │     └─ compute_loss_ray(i)            ← loss_angle 检查
        │  ├─ compute_hit()                        ← 计算 pos_w
        │  ├─ compute_hit_b()                      ← 计算 pos_b
        │  ├─ _noise->produce_united_noise()       ← 联合噪声
        │  └─ [Camera版] compute_stereo_ray()      ← stereo 验证
        │     ├─ 计算左右眼位置
        │     ├─ mj_ray(左眼→命中点), mj_ray(右眼→命中点)
        │     ├─ 遮挡检查：ratio ≠ 1 → 无效
        │     └─ 能量检查：ray_energy < threshold → 丢失
        └─ for each SensorData:
           └─ func(sensordata + data_point)
              ├─ get_data() / get_data_normalized()
              ├─ get_distance_to_image_plane()
              ├─ get_data_pos_w() / get_data_pos_b()
              └─ ... (根据 DataType 调用对应模板)
```