# 可组合数据输出详解

## 一、设计思路

传感器数据输出采用**字符串驱动的可组合设计**。用户在 MJCF XML 中用 `sensor_data_types` 属性指定一个空格分隔的字符串，例如 `"image inv_image pos_w pos_b"`，每个词代表一种数据输出。系统解析这些词，为每种输出创建一个 `SensorData` 对象，其中包含一个 lambda 函数，在仿真每步被调用，将数据写入 `mjData.sensordata` 的对应偏移位置。

这样做的好处是：**一次射线投射计算，可以同时产出多种格式的数据**，避免重复计算。

---

## 二、解析流程

### 2.1 字符串解析 — `SensorData::fromStr`

`ray_plugin.h:180-209` 定义了解析规则。每个词按关键字匹配确定 `DataType` 和三个修饰标记：

| 关键字 | 匹配规则 | 效果 |
|--------|----------|------|
| `noise` | 包含 "noise" | `is_noise = true` |
| `inf_zero` | 包含 "inf_zero" | `is_inf_max = false` |
| `inv` | 包含 "inv" | `is_inv = true` |

然后按核心关键字确定类型（优先级从上到下）：

| 关键字 | DataType | 含义 |
|--------|----------|------|
| `data` | `DataType::data` | 原始距离值 |
| `distance_to_image_plane` | `DataType::distance_to_image_plane` | 投影到相机平面的距离 |
| `image_plane_image` | `DataType::image_plane_image` | image plane 深度归一化到 0~255 |
| `image_plane_normal` | `DataType::image_plane_normal` | image plane 深度归一化到 0~1 |
| `image`（其他含 "image" 的词） | `DataType::image` | 沿射线距离归一化到 0~255 |
| `normal` | `DataType::normal` | 沿射线距离归一化到 0~1 |
| `pos_w` | `DataType::pos_w` | 世界坐标系命中位置 (x,y,z) |
| `pos_b` | `DataType::pos_b` | 自身坐标系命中位置 (x,y,z) |

**组合示例：**

| 写法 | 解析结果 |
|------|----------|
| `image` | image, is_noise=false, is_inf_max=true, is_inv=false |
| `inv_image` | image, is_noise=false, is_inf_max=true, is_inv=true |
| `image_noise` | image, is_noise=true, is_inf_max=true, is_inv=false |
| `inv_image_noise` | image, is_noise=true, is_inf_max=true, is_inv=true |
| `inv_image_inf_zero_noise` | image, is_noise=true, is_inf_max=false, is_inv=true |
| `data_inf_zero` | data, is_noise=false, is_inf_max=false |
| `pos_w` | pos_w, 3×nray 数据 |

修饰标记可以**任意组合**，只要包含对应子串即可。

---

## 三、每种 DataType 的计算逻辑

### 3.1 `DataType::data` — 原始距离

对应 `RayCaster::get_data<T>` 模板（`RayCaster.h:379-412`）。

```
对每条射线 idx:
  geomids[idx] < 0 (未命中):
    is_inf_max=true → dist[idx] = deep_max
    is_inf_max=false → 0.0
  is_lost[idx] = true → 0.0
  is_noise == has_noise:
    is_lost_noise[idx] → 0.0
    否则 → dist[idx]         ← 带噪声的距离
  is_noise != has_noise:
    → dist_ratio[idx] * deep_max  ← 无噪声的距离
```

**关键区分：`is_noise` 标记控制输出的是带噪声版本还是无噪声版本。**

- 如果传感器配置了噪声（`has_noise=true`），那么 `dist[idx]` 是经过噪声处理的值，`dist_ratio[idx] * deep_max` 是原始无噪声值。
- 当 `is_noise=true`（词含 "noise"），输出 `dist[idx]`（带噪）。
- 当 `is_noise=false`（词不含 "noise"），输出 `dist_ratio[idx] * deep_max`（无噪）。
- 通过同时请求 `image` 和 `image_noise`，你可以**同时获得无噪和带噪两帧数据**。

---

### 3.2 `DataType::image` — 深度图（0~255）

对应 `RayCaster::get_data_normalized<T>` 模板（`RayCaster.h:334-376`），`scale = 255.0`。

```
对每条射线 idx:
  geomids[idx] < 0:
    is_inf_max=true → distance = deep_max
    is_inf_max=false → 0.0 (跳过)
  is_lost[idx] → 0.0
  is_noise == has_noise:
    is_lost_noise[idx] → 0.0
    否则 → distance = dist[idx]     (带噪)
  is_noise != has_noise:
    distance = dist_ratio[idx] * deep_max  (无噪)

  is_inv=false (默认): data[idx] = ((distance - deep_min) / deep_min_dif) * 255
  is_inv=true  (含inv): data[idx] = (1.0 - (distance - deep_min) / deep_min_dif) * 255
```

**归一化公式：**

| 模式 | 公式 | 物理含义 |
|------|------|----------|
| 正向 (`is_inv=false`) | `((d - min) / (max - min)) × 255` | 近→暗(0)，远→亮(255) |
| 反色 (`is_inv=true`) | `(1 - (d - min) / (max - min)) × 255` | 近→亮(255)，远→暗(0) |

反色模式与真实深度图的视觉习惯一致（近处物体明亮，远处暗淡）。

---

### 3.3 `DataType::normal` — 归一化深度值（0~1）

与 `image` 相同逻辑，但 `scale = 1.0`。输出范围 0~1 而非 0~255，适合作为浮点数据输入给下游算法。

---

### 3.4 `DataType::distance_to_image_plane` — 投影距离

对应 `RayCaster::get_distance_to_image_plane<T>` 模板（`RayCaster.h:424-472`）。

**这是与 `data`/`image` 的关键区别**：前者是**沿射线方向的距离**（射线起点到命中点的3D距离），而 `distance_to_image_plane` 是**沿相机 Z 轴方向的投影距离**。

计算方式：
```
d_plane = distance × |dz| / ||d||
```
其中 `d = (_ray_vec_x, _ray_vec_y, _ray_vec_z)` 是射线在相机坐标系下的方向向量，`dz` 是其 Z 分量的绝对值，`||d||` 是其模长。

**`cos_theta = |dz| / ||d||`** 就是射线与相机平面法线（-Z 方向）夹角的余弦值。沿射线的距离乘以这个余弦值，投影到相机平面的深度方向上。

**物理意义：** 这是真实深度相机（如 Orbbec、RealSense）输出的 "depth" ——它测量的是物体到相机平面的垂直距离，而非沿每条射线方向的斜距。对于针孔相机模型，`distance_to_image_plane` 才是真正对应真实深度图的数据。

---

### 3.5 `DataType::image_plane_image` / `image_plane_normal` — 投影距离的归一化版本

与 `image`/`normal` 的归一化逻辑相同，但输入数据是 `distance_to_image_plane` 而非沿射线距离。

| 类型 | scale | 含义 |
|------|-------|------|
| `image_plane_image` | 255.0 | 投影距离归一化到 0~255，适合直接显示为深度图 |
| `image_plane_normal` | 1.0 | 投影距离归一化到 0~1，适合作为浮点输入 |

---

### 3.6 `DataType::pos_w` — 世界坐标系命中位置

对应 `_get_data_pos_dim1<T>` 模板（`RayCaster.h:298-313`）。

每条射线输出 3 个值 `(x, y, z)`，是命中点在**世界坐标系**下的位置。未命中的射线输出 `(NAN, NAN, NAN)`。

**计算方式**（`RayCaster.cpp:553-573`）：
```
pos_w = 起点位置 + ray_vec_offset + ray_vec × dist_ratio
```
即射线起点加上沿射线方向的命中距离。

---

### 3.7 `DataType::pos_b` — 自身坐标系命中位置

对应 `_get_data_pos_dim1<T>` 模板，但使用 `_ray_vec` 和 `_ray_vec_offset`（相机坐标系下的值）。

**计算方式**（`RayCaster.cpp:575-595`）：
```
pos_b = _ray_vec_offset + _ray_vec × dist_ratio
```
命中点在**相机自身坐标系**下的位置，未命中同样为 `(NAN, NAN, NAN)`。

---

## 四、数据布局 — sensordata 与 plugin_state

### 4.1 sensordata 内存布局

数据连续写入 `d->sensordata + m->sensor_adr[sensor_id]`，按 `sensor_data_types` 的顺序依次排列：

```
示例: sensor_data_types = "image inv_image pos_w pos_b"
      射线数 nray = 64×36 = 2304

sensordata 布局:
  [0      .. 2303  ] ← image:      2304 个 mjtNum
  [2304   .. 4607  ] ← inv_image:  2304 个 mjtNum
  [4608   .. 11519 ] ← pos_w:      2304×3 = 6912 个 mjtNum
  [11520  .. 18431 ] ← pos_b:      2304×3 = 6912 个 mjtNum
```

每个 `SensorData` 记录了自己的 `data_point`（在 sensordata 中的偏移）和 `data_size`（数据长度），这些信息写入 `plugin_state` 供用户读取。

### 4.2 plugin_state 内存布局

`ray_plugin.cc:351-359`：

```
plugin_state[state_idx    ] = h_ray_num           // 水平射线数
plugin_state[state_idx + 1] = v_ray_num           // 垂直射线数
对每个 SensorData i:
  plugin_state[state_idx + 2 + 2i    ] = data_point  // sensordata 偏移
  plugin_state[state_idx + 2 + 2i + 1] = data_size   // 数据长度
```

用户可以通过读取 `plugin_state` 来定位每种数据在 `sensordata` 中的位置。

---

## 五、三种标记修饰符详解

### 5.1 `noise` 标记

**默认（不含 noise）：输出无噪声数据。**
**含 noise：输出带噪声数据。**

两者使用不同的内部数据源：

| 标记 | 数据源 | 条件 |
|------|--------|------|
| 无 `noise` | `dist_ratio[idx] * deep_max` | 原始无噪距离 |
| 有 `noise` | `dist[idx]` | 经过 `produce_noise` 处理的距离 |

典型用法：同时请求 `"image image_noise"`，获得同一帧的无噪和带噪版本，便于对比或训练去噪模型。

### 5.2 `inf_zero` 标记

**默认（不含 inf_zero）：未命中的射线输出 `deep_max`。**
**含 inf_zero：未命中的射线输出 `0.0`。**

| 标记 | 未命中值 | 适用场景 |
|------|----------|----------|
| 无 `inf_zero` (`is_inf_max=true`) | deep_max | 距离数据需要保留"无穷远"信息 |
| 有 `inf_zero` (`is_inf_max=false`) | 0.0 | 深度图中未命中区域应显示为黑色 |

深度图通常用 `inf_zero`（未命中=黑），而原始距离数据通常不用（未命中=最大距离）。

### 5.3 `inv` 标记（反色）

**默认：近→小值，远→大值（正向归一化）。**
**含 inv：近→大值，远→小值（反向归一化）。**

```
正向: value = ((distance - deep_min) / (deep_max - deep_min)) × scale
反色: value = (1.0 - (distance - deep_min) / (deep_max - deep_min)) × scale
```

反色模式符合真实深度图的视觉习惯：近处物体反射率高（亮），远处物体反射率低（暗）。

---

## 六、完整组合示例

### 6.1 双目相机深度传感器

```xml
<config key="sensor_data_types" value="image inv_image pos_w pos_b" />
```

| 数据段 | 类型 | 大小 | 含义 |
|--------|------|------|------|
| image | image, is_inv=false | nray | 正向深度图 0~255（近暗远亮） |
| inv_image | image, is_inv=true | nray | 反色深度图 0~255（近亮远暗） |
| pos_w | pos_w | nray×3 | 世界坐标系命中位置 |
| pos_b | pos_b | nray×3 | 自身坐标系命中位置 |

### 6.2 含噪声对比的深度传感器

```xml
<config key="sensor_data_types" value="inv_image_noise image_noise" />
<config key="noise_type" value="uniform" />
<config key="noise_cfg" value="-0.1 0.1 1" />
```

| 数据段 | 类型 | 含义 |
|--------|------|------|
| inv_image_noise | image, is_inv=true, is_noise=true | 反色带噪深度图 |
| image_noise | image, is_inv=false, is_noise=true | 正向带噪深度图 |

### 6.3 同时获取沿射线距离和投影距离

```xml
<config key="sensor_data_types" value="data distance_to_image_plane" />
```

| 数据段 | 类型 | 含义 |
|--------|------|------|
| data | data, is_inf_max=true | 沿射线的3D距离 |
| distance_to_image_plane | distance_to_image_plane | 投影到相机平面的距离 |

这两种距离的物理含义不同：射线距离是3D空间中从起点到命中点的直线距离，投影距离是沿相机 Z 轴的垂直距离。对于针孔相机，后者才是真实深度图对应的测量值。

### 6.4 全功能配置

```xml
<config key="sensor_data_types" 
  value="image inv_image_inf_zero image_noise inv_image_inf_zero_noise 
         data distance_to_image_plane image_plane_image pos_w pos_b" />
```

这会输出 9 种数据段，涵盖正向/反色/带噪/无噪/投影/原始/位置的所有组合。

---

## 七、三种距离的概念辨析

这是理解数据输出的核心，很多人容易混淆：

| 概念 | 计算方式 | 对应 DataType | 物理含义 |
|------|----------|---------------|----------|
| **沿射线距离** | 起点→命中点的3D直线距离 | `data`, `image`, `normal` | 每条射线独立测量的距离 |
| **投影到相机平面距离** | 沿射线距离 × cos(射线与Z轴夹角) | `distance_to_image_plane`, `image_plane_image/normal` | 真实深度相机输出的 "depth" |
| **命中位置** | 起点 + 射线方向 × 距离比值 | `pos_w` (世界坐标), `pos_b` (自身坐标) | 命中点在3D空间中的坐标 |

对于正前方射线（与Z轴对齐），三者几乎相同；对于边缘射线（与Z轴夹角大），沿射线距离明显大于投影距离——这正是针孔相机深度图与激光雷达点云的区别。

---

## 八、数据读取代码示例

### Python

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("your_scene.xml")
data = mujoco.MjData(model)

# 找到传感器在 sensordata 中的地址
sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "depth_sensor")
adr = model.sensor_adr[sensor_id]
dim = model.sensor_dim[sensor_id]

# 从 plugin_state 获取数据布局
instance = model.sensor_plugin[sensor_id]
state_adr = model.plugin_stateadr[instance]
h_ray = int(data.plugin_state[state_adr])      # 水平射线数
v_ray = int(data.plugin_state[state_adr + 1])  # 垂直射线数

# 读取深度图
mujoco.mj_step(model, data)

# 逐段读取（通过 plugin_state 定位）
for i in range(n_sensor_data):
    offset = int(data.plugin_state[state_adr + 2 + 2*i])
    size   = int(data.plugin_state[state_adr + 2 + 2*i + 1])
    segment = data.sensordata[adr + offset:adr + offset + size]
    # pos_w/pos_b 数据 size = nray*3，reshape 为 (v_ray, h_ray, 3)
    # image/data 数据 size = nray，reshape 为 (v_ray, h_ray)
    if size == h_ray * v_ray * 3:
        arr = segment.reshape(v_ray, h_ray, 3)
    else:
        arr = segment.reshape(v_ray, h_ray)
```

### C++ (Standalone API)

```cpp
// 使用 raycaster_src/ 独立 API（无需插件层）
RayCasterCameraCfg cfg;
cfg.m = m; cfg.d = d; cfg.cam_name = "depth_cam";
cfg.focal_length = 11.41;
cfg.horizontal_aperture = 20.995;
cfg.h_ray_num = 160; cfg.v_ray_num = 90;
cfg.dis_range = {0.2, 5.0};

RayCasterCamera raycaster(cfg);
raycaster.compute_distance();

// 获取数据
auto depth_vec = raycaster.get_data_normalized_vec(false, true, true, 255.0);
auto pos_w_vec = raycaster.get_data_pos_w();
auto d_plane_vec = raycaster.get_distance_to_image_plane_normalized_vec(false, true, true, 255.0);
```