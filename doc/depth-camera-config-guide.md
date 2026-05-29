# 深度相机配置指南

机器狗（如 Unitree Go2、ANYmal）上用的深度相机是结构光/ToF 类型（如 Orbbec），输出的是**相机平面投影深度图**，不是沿射线的距离。你应该用 **`ray_caster_camera`** 插件。

---

## 一、确认真实相机参数

配置前需要查清三个核心参数（通常在相机规格页或 SDK 文档中）：

| 参数 | 来源示例 | 说明 |
|------|----------|------|
| **分辨率** | 640×480, 320×240 等 | `size` 属性直接填此值 |
| **焦距** | 从相机内参矩阵 `fx` 换算 | `focal_length` 单位为 cm |
| **深度范围** | 如 0.2~5.0m 或 0.3~10.0m | `dis_range` 单位为 m |
| **基线** | 双目相机两眼间距 | 单目深度相机填 0 或不填 |

**焦距换算方法**（这是最容易出错的地方）：

代码中 `focal_length` 和 `horizontal_aperture` 单位都是 **cm**（见 `RayCasterCamera.h:12-14`）。真实相机给出的内参 `fx` 通常以像素为单位，换算公式：

```
focal_length_cm = fx × pixel_size_cm
```

其中 `pixel_size_cm = sensor_width_cm / image_width_pixels`。

或者更直接地：

```
focal_length_cm = fx / image_width × horizontal_aperture_cm
```

**简化做法**：如果你知道相机的水平 FOV 角度 `hfov`，可以不用算焦距，直接用：

```
horizontal_aperture = 任意值（比如实际传感器宽度）
focal_length = horizontal_aperture / (2 × tan(hfov/2))
```

---

## 二、推荐 MJCF 配置

### 2.1 基础深度图（单目，最常用）

适合只需要深度图做导航/避障的机器狗：

```xml
<mujoco>
  <extension>
    <plugin plugin="mujoco.sensor.ray_caster_camera" />
  </extension>

  <worldbody>
    <!-- 机器狗身体 -->
    <body name="dog_body" ...>
      ...
      <!-- 头部深度相机，朝前方 -->
      <camera name="depth_cam" pos="0.05 0 0.08" 
              euler="0 15 0" fovy="60" />
    </body>
  </worldbody>

  <sensor>
    <plugin name="depth_sensor" 
            plugin="mujoco.sensor.ray_caster_camera" 
            objtype="camera" objname="depth_cam">
      
      <!-- 分辨率：低分辨率节省计算 -->
      <config key="size" value="160 90" />
      
      <!-- 相机光学参数（根据你的真实相机换算） -->
      <config key="focal_length" value="11.41" />
      <config key="horizontal_aperture" value="20.995" />
      
      <!-- 深度范围：与真实相机一致 -->
      <config key="dis_range" value="0.2 5.0" />
      
      <!-- 输出投影深度图（真实深度相机的输出） -->
      <config key="sensor_data_types" 
              value="image_plane_image_inv_inf_zero" />
      
      <!-- 噪声：模拟真实深度相机的测量误差 -->
      <config key="noise_type" value="noise1" />
      <config key="noise_cfg" value="-0.02 0.02 0.05 0" />
      
      <!-- 不检测自身身体 -->
      <config key="detect_parentbody" value="1" />
      
      <!-- 每5步更新一次（降低计算频率） -->
      <config key="n_step_update" value="5" />
      
      <!-- 调试可视化 -->
      <config key="draw_hip_point" value="1 0.02" />
    </plugin>
  </sensor>
</mujoco>
```

**关键说明：**

- `sensor_data_types` 用的是 `image_plane_image_inv_inf_zero`，这是**最接近真实深度相机输出**的组合：
  - `image_plane`：投影到相机平面的距离（而非沿射线距离）
  - `image`：归一化到 0~255
  - `inv`：反色（近亮远暗，符合深度图视觉习惯）
  - `inf_zero`：未命中像素为 0（黑色），而非 deep_max
- `noise1`：均匀偏移 + 5% 概率置零，模拟真实深度相机的随机噪点和像素丢失
- `detect_parentbody`：避免射线命中自身机器狗身体
- `n_step_update`：深度相机通常 20~30Hz，机器狗仿真 500Hz，所以每 5 步更新一次

### 2.2 带 stereo 验证的深度图（双目/结构光）

如果你的真实相机是双目结构（如 Orbbec DaA2），需要模拟遮挡阴影：

```xml
<plugin name="depth_sensor" 
        plugin="mujoco.sensor.ray_caster_camera" 
        objtype="camera" objname="depth_cam">
  
  <config key="size" value="160 90" />
  <config key="focal_length" value="11.41" />
  <config key="horizontal_aperture" value="20.995" />
  <config key="dis_range" value="0.2 5.0" />
  
  <!-- 双目基线（cm），与真实相机一致 -->
  <config key="baseline" value="1.2" />
  
  <!-- 输出：带噪深度图 + 无噪深度图 + 点云 -->
  <config key="sensor_data_types" 
          value="image_plane_image_inv_inf_zero_noise 
                 image_plane_image_inv_inf_zero 
                 pos_w" />
  
  <!-- stereo噪声：基于光路能量的概率丢失 -->
  <config key="noise_type" value="stereo_noise" />
  <config key="noise_cfg" value="2 0" />
  
  <!-- 最低能量阈值 -->
  <config key="min_energy" value="0.15" />
  
  <config key="detect_parentbody" value="1" />
  <config key="n_step_update" value="5" />
  <config key="num_thread" value="4" />
</plugin>
```

**与基础版的区别：**

| 参数 | 基础版 | stereo 版 | 原因 |
|------|--------|-----------|------|
| `baseline` | 不填（0） | 填真实基线值 | 激发 stereo 验证逻辑 |
| `noise_type` | `noise1` | `stereo_noise` | stereo_noise 基于光路能量概率丢失，更真实 |
| `min_energy` | 不填 | 填值 | 控制光路能量阈值，低能量的射线更容易丢失 |
| `sensor_data_types` | 单输出 | 多输出 | 同时获取带噪/无噪版本便于训练 |
| `num_thread` | 不填 | 4 | stereo 计算量大，建议多线程 |

**`baseline > 0` 时发生了什么**（`RayCasterCamera.cpp:147-177`）：

1. 基类 `compute_distance()` 正常投射左眼射线
2. 计算左右眼位置：左眼在 `(-baseline/2, 0, 0)`（相机坐标系），右眼在 `(+baseline/2, 0, 0)`
3. 对每个已命中的像素点，从左眼和右眼分别投射射线到该命中点
4. 如果**任一眼睛的射线**被其他物体遮挡（`mj_ray` 返回值 ≠ 1.0），则该像素标记为无效（`dist=0, pos_w=NAN`）
5. 这模拟了真实双目/结构光相机在**遮挡阴影区域**无法测量的现象

### 2.3 多相机前+下配置（完整感知）

机器狗通常头部朝前、腹部朝下各装一个深度相机：

```xml
<worldbody>
  <body name="dog_body" ...>
    <!-- 头部朝前方 -->
    <camera name="depth_cam_front" pos="0.05 0 0.08" euler="0 15 0" fovy="60"/>
    <!-- 腹部朝地面 -->
    <camera name="depth_cam_down"   pos="0 0 -0.05" euler="0 90 0" fovy="60"/>
  </body>
</worldbody>

<sensor>
  <plugin name="depth_front" plugin="mujoco.sensor.ray_caster_camera" 
          objtype="camera" objname="depth_cam_front">
    <config key="size" value="160 90" />
    <config key="focal_length" value="11.41" />
    <config key="horizontal_aperture" value="20.995" />
    <config key="dis_range" value="0.2 5.0" />
    <config key="sensor_data_types" value="image_plane_image_inv_inf_zero" />
    <config key="noise_type" value="noise1" />
    <config key="noise_cfg" value="-0.02 0.02 0.05 0" />
    <config key="detect_parentbody" value="1" />
    <config key="n_step_update" value="5" />
  </plugin>

  <plugin name="depth_down" plugin="mujoco.sensor.ray_caster_camera" 
          objtype="camera" objname="depth_cam_down">
    <config key="size" value="120 90" />
    <config key="focal_length" value="11.41" />
    <config key="horizontal_aperture" value="20.995" />
    <config key="dis_range" value="0.05 3.0" />
    <config key="sensor_data_types" value="image_plane_image_inv_inf_zero pos_w" />
    <config key="noise_type" value="noise1" />
    <config key="noise_cfg" value="-0.01 0.01 0.03 0" />
    <config key="detect_parentbody" value="1" />
    <config key="n_step_update" value="10" />
  </plugin>
</sensor>
```

腹部相机特点：更短的深度范围（0.05~3m，只需测地面距离）、更低分辨率（120×90 足够）、更低更新频率（地形变化慢）。

---

## 三、sensor_data_types 选择指南

对于深度相机，核心决策是**选哪种距离**：

| 你的需求 | 推荐写法 | 原因 |
|----------|----------|------|
| 只需要深度图显示/导航 | `image_plane_image_inv_inf_zero` | 投影深度+反色+未命中黑，最接近真实输出 |
| 训练去噪模型 | `image_plane_image_inv_inf_zero_noise image_plane_image_inv_inf_zero` | 同时输出带噪和无噪版本 |
| 需要 3D 点云 | `image_plane_image_inv_inf_zero pos_w` | pos_w 给出世界坐标命中位置 |
| SLAM / 地形估计 | `pos_w pos_b` | 两种坐标系命中位置，适合建图 |
| 对比沿射线距离 vs 投影距离 | `data distance_to_image_plane` | 两种原始距离对比 |

**务必用 `image_plane` 系列而非普通 `image`**，原因：

```
沿射线距离:    d_ray = 起点→命中点的3D直线长度
投影平面距离:  d_plane = d_ray × cos(射线与Z轴夹角)
```

真实深度相机（Orbbec、RealSense）输出的是 **d_plane**。对于针孔相机边缘像素，`d_ray` 明显大于 `d_plane`。用普通 `image` 会导致边缘物体看起来比实际更远。

---

## 四、噪声选择指南

| 噪声类型 | noise_type | noise_cfg 参数 | 适用场景 |
|----------|------------|----------------|----------|
| 基础均匀偏移 | `uniform` | `low high seed` | 快速验证，不追求真实感 |
| 高斯偏移 | `gaussian` | `mean stddev seed` | 更接近真实传感器噪声分布 |
| 均匀偏移+像素丢失 | `noise1` | `low high zero_prob seed` | 最推荐——真实深度相机既有偏移也有随机黑像素 |
| 入射角丢失 | `noise2` | `low high zero_prob min_angle max_angle low_prob high_prob seed` | 边缘区域退化严重时 |
| 双目能量丢失 | `stereo_noise` | `power seed` | 双目/结构光相机，遮挡阴影区域的概率丢失 |

**机器狗推荐**：先用 `noise1`（简单有效），确认参数合理后如果需要更真实可以升级到 `noise2` 或 `stereo_noise`。

`noise1` 参数调优参考：
- `low=-0.02, high=0.02`：±2cm 偏移（典型 ToF 相机精度）
- `zero_prob=0.05`：5% 概率像素丢失（真实深度相机常见）
- `seed=0`：使用随机种子（每次运行不同）

---

## 五、Python 数据读取

配置好后，在 Python 中读取深度数据：

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("your_scene.xml")
data = mujoco.MjData(model)

# 找到传感器在 sensordata 中的地址
sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "depth_front")
adr = model.sensor_adr[sensor_id]
dim = model.sensor_dim[sensor_id]

# 从 plugin_state 获取数据布局
instance = model.sensor_plugin[sensor_id]
state_adr = model.plugin_stateadr[instance]
h_ray = int(data.plugin_state[state_adr])      # 水平射线数
v_ray = int(data.plugin_state[state_adr + 1])  # 垂直射线数

# 仿真步进
mujoco.mj_step(model, data)

# 通过 plugin_state 定位每个数据段
# 假设 sensor_data_types = "image_plane_image_inv_inf_zero pos_w"
# 则有 2 个 SensorData

n_segments = 2  # = sensor_data_types 中空格分隔的词数
segments = {}
for i in range(n_segments):
    offset = int(data.plugin_state[state_adr + 2 + 2*i])
    size   = int(data.plugin_state[state_adr + 2 + 2*i + 1])
    raw = data.sensordata[adr + offset:adr + offset + size]
    if size == h_ray * v_ray:
        segments[i] = raw.reshape(v_ray, h_ray)
    elif size == h_ray * v_ray * 3:
        segments[i] = raw.reshape(v_ray, h_ray, 3)

# segments[0] = 反色深度图 (v_ray, h_ray)
# segments[1] = 世界坐标点云 (v_ray, h_ray, 3)
```

---

## 六、常见错误排查

| 问题 | 原因 | 解决 |
|------|------|------|
| 射线命中了机器狗自身 | `detect_parentbody` 未设 | 加 `<config key="detect_parentbody" value="1" />` |
| 深度图边缘物体距离偏大 | 用了 `image` 而非 `image_plane_image` | 改用 `image_plane` 系列 |
| 深度图全是黑/白 | `dis_range` 与场景尺度不匹配 | 调整 deep_min/deep_max |
| stereo_noise 不生效 | MuJoCo 版本 < 3.5.0 | 升级 MuJoCo |
| 计算太慢 | 分辨率太高 | 降低 `size`，或用 `n_step_update` 降低频率 |
| `focal_length` 单位不对 | 代码用 cm，你填了 mm 或 px | 确认单位为 cm |