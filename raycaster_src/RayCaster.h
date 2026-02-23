#pragma once
#include "Noise.hpp"
#include "RayNoise.hpp"
#include "mujoco/mjthread.h"
#include "mujoco/mjtnum.h"
#include <array>
#include <cmath>
#include <iostream>
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <type_traits>
#include <vector>

#define LIMIT_MAX 0.999999999

enum RayCasterType { base, yaw, world, none };

class RayCasterCfg {
public:
  const mjModel *m = nullptr;
  mjData *d = nullptr;
  std::string cam_name = "";
  mjtNum resolution = 0.01;
  std::array<mjtNum, 2> size = {1.0, 1.0};
  std::array<mjtNum, 2> dis_range = {0.0, 10.0};
  RayCasterType type = RayCasterType::none;
  bool is_detect_parentbody = false;
  mjtNum loss_angle = 0.0;
};

class RayCaster {
public:
  RayCaster();
  RayCaster(const RayCasterCfg &cfg);
  ~RayCaster();
  void init(const RayCasterCfg &cfg);
  /** @brief 初始化
   * @param m mjModel
   * @param d mjData
   * @param cam_name 相机名称
   * @param h_ray_num 水平射线数量
   * @param v_ray_num 垂直射线数量
   * @param dis_range 距离范围 [最小，最大] (M)
   * @param is_detect_parentbody 是否检测自身
   */
  void _init(const mjModel *m, mjData *d, std::string cam_name, int h_ray_num,
             int v_ray_num, const std::array<mjtNum, 2> &dis_range,
             bool is_detect_parentbody, mjtNum loss_angle = 0.0);

  /** @brief 设置线程数量
   * @param n 线程数量
   */
  virtual void set_num_thread(int n);
  /** @brief ldm反射到stereo_camera的光路法线和物体表面法线相差角度，(0,90)
   * @param n 射线丢失角度
   */
  void set_lossangle(mjtNum loss_angle);

  /** @brief 计算距离 数值存放在dist中*/
  virtual void compute_distance();

  /** @brief 绘制测量射线 在mjv_updateScene和mjr_render中间
   * @param scn mjvScene
   * @param ratio 绘制比例
   * @param width 射线宽度
   * @param edge 仅绘制边缘
   * @param color 颜色
   */
  void draw_deep_ray(mjvScene *scn, int ratio, int width = 5, bool edge = false,
                     float *color = nullptr);
  /** @brief 绘制特定射线 在mjv_updateScene和mjr_render中间
   * @param scn mjvScene
   * @param idx 射线索引 get_idx获取
   * @param width 射线宽度
   * @param color 颜色
   */
  void draw_deep_ray(mjvScene *scn, int idx, int width = 5,
                     float *color = nullptr);

  /** @brief 绘制距离线段 在mjv_updateScene和mjr_render中间
   * @param scn mjvScene
   * @param ratio 绘制比例
   * @param width 射线宽度
   * @param color 颜色
   */
  void draw_deep(mjvScene *scn, int ratio, int width = 5,
                 float *color = nullptr);
  /** @brief 绘制射线命中点 在mjv_updateScene和mjr_render中间
   * @param scn mjvScene
   * @param ratio 绘制比例
   * @param size 点大小
   * @param color 颜色
   */
  void draw_hip_point(mjvScene *scn, int ratio, mjtNum size = 0.1,
                      float *color = nullptr);
  /** @brief 绘制法向量 在mjv_updateScene和mjr_render中间
   * @param scn mjvScene
   * @param ratio 绘制比例
   * @param width 法向量宽度
   * @param color 颜色
   */
  void draw_normal(mjvScene *scn, int ratio, int width = 5,
                   float *color = nullptr);

  /** @brief 获取dist中索引
  * @param h 水平索引
  * @param v 垂直索引
  无效索引返回-1
  */
  int get_idx(int h, int v);

  template <typename NoiseType> void setNoise(NoiseType noise) {
    delete _noise;
    _noise = new NoiseType(noise);
    if constexpr (std::is_same_v<NoiseType, ray_noise::Noise>)
      return;
    has_noise = true;
  }
  bool has_noise = false;

  static int get_nray(RayCasterCfg &cfg);
  void setNoise(ray_noise::RayNoise2 noise);
#if mjVERSION_HEADER > 340
  void setNoise(ray_noise::RayNoise3 noise);
#endif

  ray_noise::Noise *_noise;

  mjtNum *dist;               // 距离 h_ray_num * v_ray_num
  int nray;                   // 射线数量
  int no_detect_body_id = -1; // 是否检测 id 不检测就是-1

  const mjModel *m;
  mjData *d;
  int cam_id;  // 相机id
  mjtNum *pos; // 相机位置
  mjtNum *mat; // 相机的旋转矩阵
  mjtNum yaw = 0.0;
  int h_ray_num = 50; // 水平
  int v_ray_num = 50; // 垂直
  mjtNum deep_max = 1e6;
  mjtNum deep_min = 0;
  mjtNum deep_min_ratio;
  mjtNum deep_min_ratio_dif;
  mjtNum deep_min_dif;
  mjtNum *_ray_vec;        // h_ray_num * v_ray_num * 3 相对于相机坐标系的偏转
  mjtNum *_ray_vec_offset; // h_ray_num * v_ray_num * 3 相对于相机坐标系的位移
  mjtNum *ray_vec;         // h_ray_num * v_ray_num * 3 世界坐标系下的偏转
  mjtNum *ray_vec_offset;  // h_ray_num * v_ray_num * 3 世界坐标系下的位移
  int *geomids;            // 命中的geomid
  bool *is_lost;           // 是否丢失射线
  mjtNum *dist_ratio;
  mjtByte geomgroup[8] = {true,  true,  false,
                          false, false, false}; // 检测哪些类型的geom
  bool is_offert = true;
  RayCasterType type = RayCasterType::none;
  int num_thread = 0;

#if mjVERSION_HEADER > 340
  mjtNum *ray_normal; // 射线法线
#endif
  mjtNum loss_angle = 0.0;
  mjtNum loss_angle_cos = 0.0; // 角度cos小于stereo_loss_angle_cos
  bool is_loss_angle = false;
#if mjVERSION_HEADER > 340
  virtual void compute_loss_ray(int idx) {
    if (is_loss_angle)
    {
      const mjtNum *ray_ptr = ray_vec + idx * 3;
      mjtNum L[3] = {-ray_ptr[0], -ray_ptr[1], -ray_ptr[2]};
      mjtNum cos_light = mju_dot3(ray_normal, L);
      mju_normalize3(L);
      if (cos_light < loss_angle_cos) {
        pos_w[idx * 3] = pos_w[idx * 3 + 1] = pos_w[idx * 3 + 2] = NAN;
        dist[idx] = 0;
        dist_ratio[idx] = 0;
      }
    }
  };
#else
  virtual void compute_loss_ray(int idx) {}
#endif

  int _get_idx(int h, int v);
  // 将ray从相机坐标系转换到世界坐标系
  void compute_ray_vec();

  // 初始化时创建射线相对于相机坐标系偏转向量 _ray_vec，非单位向量
  virtual void create_rays();

  void get_inv_image_data(unsigned char *image_data, bool is_noise = false,
                          bool is_inf_max = true);

  void get_image_data(unsigned char *image_data, bool is_noise = false,
                      bool is_inf_max = true, bool is_inv = false);

  std::vector<double> get_data_normalized_vec(bool is_noise,
                                              bool is_inf_max = false,
                                              bool is_inv = false,
                                              double scale = 1.0);

  // 直接测量距离信息
  std::vector<double> get_data_vec(bool is_inf_max = true);

  // 世界坐标系命中位置 没命中的返回(NAN,NAN,NAN)
  std::vector<std::vector<double>> get_data_pos_w();
  // 自身坐标系命中位置 没命中的返回(NAN,NAN,NAN)
  std::vector<std::vector<double>> get_data_pos_b();

  void draw_line(mjvScene *scn, mjtNum *from, mjtNum *to, mjtNum width,
                 float *rgba);
  void draw_arrow(mjvScene *scn, mjtNum *from, mjtNum *to, mjtNum width,
                 float *rgba);
  void draw_geom(mjvScene *scn, int type, mjtNum *size, mjtNum *pos,
                 mjtNum *mat, float rgba[4]);

  void rotate_vector_with_yaw(mjtNum result[3], mjtNum yaw,
                              const mjtNum vec[3]);

  /// 返回未归一化 distance_to_image_plane 的 std::vector
  std::vector<double> get_distance_to_image_plane_vec(bool is_noise = false,
                                                      bool is_inf_max = true);

  /// 返回归一化 distance_to_image_plane 的 std::vector
  std::vector<double> get_distance_to_image_plane_normalized_vec(
      bool is_noise = false, bool is_inf_max = true, bool is_inv = false,
      double scale = 1.0);

  /// distance_to_image_plane 图像化（0~255），行为与 get_image_data 对齐
  void get_distance_to_image_plane_image(unsigned char *image_data,
                                         bool is_noise = false,
                                         bool is_inf_max = true,
                                         bool is_inv = false);

  /// distance_to_image_plane 图像化 + 反色，行为与 get_inv_image_data 对齐
  void get_distance_to_image_plane_inv_image(unsigned char *image_data,
                                             bool is_noise = false,
                                             bool is_inf_max = true);

  // 如果不绘制落点可以关掉提高性能，ray_noise::RayNoise2会自动开启is_compute_hit
  bool is_compute_hit = true;
  bool is_compute_hit_b = true;
  mjtNum *pos_w; // 命中位置
  mjtNum *pos_b; // 命中位置
  void compute_hit();
  void compute_hit_b();

  mjThreadPool *pool = nullptr;
  struct RayTaskData {
    RayCaster *instance; // 指向你的类实例
    int start;
    int end;
  };
  std::vector<RayTaskData> ray_task_datas;
  static void *ray_task_func(void *user_data) {
    RayTaskData *data = static_cast<RayTaskData *>(user_data);
    // 调用成员函数
    data->instance->compute_ray(data->start, data->end);
    return nullptr;
  }

  void compute_ray(int start, int end);

private:
  void draw_ray(int idx, int width, float *color, mjvScene *scn, bool is_scale);

  mjtNum resolution;
  mjtNum size[2];

  /*-----------模板-----------*/
  template <typename T> void _get_data_pos_dim1(T &data, const mjtNum *pos) {
    for (int i = 0; i < v_ray_num; i++) {
      for (int j = 0; j < h_ray_num; j++) {
        int idx = _get_idx(i, j);
        data[idx * 3] = pos[idx * 3];
        data[idx * 3 + 1] = pos[idx * 3 + 1];
        data[idx * 3 + 2] = pos[idx * 3 + 2];
      }
    }
  }

  template <typename T> void _get_data_pos_dim2(T &data, const mjtNum *pos) {
    for (int i = 0; i < v_ray_num; i++) {
      for (int j = 0; j < h_ray_num; j++) {
        int idx = _get_idx(i, j);
        data[idx][0] = pos[idx * 3];
        data[idx][1] = pos[idx * 3 + 1];
        data[idx][2] = pos[idx * 3 + 2];
      }
    }
  }

public:
  template <typename T>
  void get_data_normalized(T &data, bool is_noise, bool is_inf_max, bool is_inv,
                           double scale) {
    for (int idx = 0; idx < nray; idx++) {
      if (is_lost[idx]) {
        data[idx] = 0.0;
        continue;
      }
      mjtNum distance;
      // 未命中
      if (geomids[idx] < 0) {
        if (is_inf_max)
          distance = deep_max;
        else {
          data[idx] = 0;
          continue;
        }
      } else {
        // 命中
        if (is_noise == has_noise) {
          distance = dist[idx];
        } else {
          distance = dist_ratio[idx] * deep_max;
        }
      }
      if (is_inv) {
        // 近 -> 大，远 -> 小
        data[idx] = (1.0 - (distance - deep_min) / deep_min_dif) * scale;
      } else {
        // 近 -> 小，远 -> 大
        data[idx] = ((distance - deep_min) / deep_min_dif) * scale;
      }
    }
  }

  template <typename T>
  void get_data(T &data, bool is_noise = false, bool is_inf_max = true) {
    for (int idx = 0; idx < nray; idx++) {
      if (is_lost[idx]) {
        data[idx] = 0.0;
        continue;
      }
      // 未命中
      if (geomids[idx] < 0) {
        if (is_inf_max)
          data[idx] = deep_max;
        else
          data[idx] = 0.0;
        continue;
      } else {
        if (is_noise == has_noise) {
          // 使用带噪声的 dist（distance_to_camera）
          data[idx] = dist[idx];
        } else {
          // 使用无噪声：dist_ratio * deep_max
          data[idx] = dist_ratio[idx] * deep_max;
        }
      }
    }
  }

  template <typename T> void get_data_pos_w(T &data) {
    _get_data_pos_dim1(data, pos_w);
  }
  template <typename T> void get_data_pos_b(T &data) {
    _get_data_pos_dim1(data, pos_b);
  }

  //================ distance_to_image_plane 接口 ================
  // 未归一化的 distance_to_image_plane（在相机坐标系中直接计算）
  template <typename T>
  void get_distance_to_image_plane(T &data, bool is_noise, bool is_inf_max) {
    for (int idx = 0; idx < nray; idx++) {
      if (is_lost[idx]) {
        data[idx] = 0.0;
        continue;
      }
      // 未命中
      if (geomids[idx] < 0) {
        if (is_inf_max)
          data[idx] = deep_max;
        else
          data[idx] = 0.0;
        continue;
      }

      // 1) 取当前射线在相机坐标系下的方向向量 d =
      // (_ray_vec_x,_ray_vec_y,_ray_vec_z)
      const mjtNum dx = _ray_vec[idx * 3 + 0];
      const mjtNum dy = _ray_vec[idx * 3 + 1];
      const mjtNum dz = _ray_vec[idx * 3 + 2];

      const mjtNum d_norm = mju_sqrt(dx * dx + dy * dy + dz * dz);
      if (d_norm < 1e-12) {
        data[idx] = 0.0;
        continue;
      }

      // 2) 选择沿射线的长度 s：带噪声 or 无噪声
      mjtNum s;
      if (is_noise == has_noise) {
        // 使用带噪声的 dist（distance_to_camera）
        s = dist[idx];
      } else {
        // 使用无噪声：dist_ratio * deep_max
        s = dist_ratio[idx] * deep_max;
      }

      // 3) 与相机平面法向 (-Z) 的夹角：cosθ = |dz| / ||d||
      const mjtNum cos_theta = mju_abs(dz) / d_norm;

      // 4) 距离到相机平面 = s * cosθ
      mjtNum d_plane = s * cos_theta;

      // 可选：限制在 [deep_min, deep_max]
      if (d_plane < deep_min)
        d_plane = deep_min;
      if (d_plane > deep_max)
        d_plane = deep_max;

      data[idx] = d_plane;
    }
  }

  template <typename T>
  void get_distance_to_image_plane_normalized(T &data, bool is_noise,
                                              bool is_inf_max, bool is_inv,
                                              double scale) {
    for (int idx = 0; idx < nray; idx++) {
      if (is_lost[idx]) {
        data[idx] = 0.0;
        continue;
      }
      mjtNum d_plane;
      // 未命中
      if (geomids[idx] < 0) {
        if (is_inf_max)
          d_plane = deep_max;
        else {
          data[idx] = 0;
          continue;
        }
      } else {
        const mjtNum dx = _ray_vec[idx * 3 + 0];
        const mjtNum dy = _ray_vec[idx * 3 + 1];
        const mjtNum dz = _ray_vec[idx * 3 + 2];

        const mjtNum d_norm = mju_sqrt(dx * dx + dy * dy + dz * dz);
        if (d_norm < 1e-12) {
          data[idx] = 0;
          continue;
        }

        mjtNum s;
        if (is_noise == has_noise) {
          s = dist[idx];
        } else {
          s = dist_ratio[idx] * deep_max;
        }

        const mjtNum cos_theta = mju_abs(dz) / d_norm;
        d_plane = s * cos_theta;

        if (d_plane < deep_min)
          d_plane = deep_min;
        if (d_plane > deep_max)
          d_plane = deep_max;
      }

      mjtNum v;
      if (is_inv) {
        // 近 -> 大，远 -> 小
        v = (1.0 - (d_plane - deep_min) / deep_min_dif) * scale;
      } else {
        // 近 -> 小，远 -> 大
        v = ((d_plane - deep_min) / deep_min_dif) * scale;
      }
      data[idx] = v;
    }
  }
};
