#pragma once
// clang-format off
/* === MODULE MANIFEST V2 ===
module_name: PowerControl
module_description: Power control for chassis (supports omni and helm wheel)
constructor_args:
  - superpower: '@&superpower'
  - is_helm: false
  - chassis_static_power_loss: 3.5
  - motor_count_3508: 4
  - motor_count_6020: 0
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "RLS.hpp"
#include "SuperPower.hpp"
#include "app_framework.hpp"
#include "message.hpp"
#include "thread.hpp"

#define ERROR_POWERDISTRIBUTION_SET 20
#define POP_POWERDISTRIBUTION 15
#define CHASSIS_POWER_LIMIT_MARGIN_W 4.0f /* 底盘限功率余量 */

/**
 * @brief 计算单个电机模型预测功率 (不含静态损耗)
 */
inline float calculate_motor_model_power(float current, float rpm, float kt,
                                         float k1, float k2) {
  return (kt * current * rpm) + (k1 * current * current) + (k2 * rpm * rpm);
}

/**
 * @brief 根据目标功率反解电流
 */
inline float solve_current_for_power(float target_power, float rpm, float kt,
                                     float k1, float k2,
                                     float original_current) {
  float a = k1;
  float b = kt * rpm;
  float c = k2 * rpm * rpm - target_power;
  float delta = b * b - 4.0f * a * c;

  float x3 = -b / (2.0f * a);

  float final_current = 0;

  if (delta < 1e-9f) {
    final_current = x3;
  } else {
    float sqrt_delta = sqrtf(delta);
    float x1 = (-b + sqrt_delta) / (2.0f * a);
    float x2 = (-b - sqrt_delta) / (2.0f * a);

    if (original_current >= 0) {
      final_current = x1;
    } else {
      final_current = x2;
    }
  }

  return std::clamp(final_current, -16384.0f, 16384.0f);
}

static constexpr int POWER_CONTROL_MAX_MOTOR_COUNT = 6; /* 最大电机数目 */

struct PowerControlData {
  float new_output_current_3508[POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float new_output_current_6020[POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  bool is_power_limited = false;
};

class PowerControl : public LibXR::Application {
 public:
  static constexpr int MAX_MOTOR_COUNT = POWER_CONTROL_MAX_MOTOR_COUNT;
  /* 3508 组分配偏置: 先保底, 再分剩余功率 */
  struct AllocationBias3508 {
    bool enabled = false;
    float reserve_fraction = 0.0f;
    float reserve_weight[MAX_MOTOR_COUNT] = {};
    float allocation_weight_scale[MAX_MOTOR_COUNT] = {};
  };

  PowerControl(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
               SuperPower* super_power, bool is_helm = false,
               float chassis_static_power_loss = 0.0f, int motor_count_3508 = 4,
               int motor_count_6020 = 4)
      : superpower_(super_power),
        is_helm_(is_helm),
        rls_(1e-5f, 0.99999f),
        k3_chassis_(chassis_static_power_loss),
        motor_count_3508_(motor_count_3508 > MAX_MOTOR_COUNT
                              ? MAX_MOTOR_COUNT
                              : motor_count_3508),
        motor_count_6020_(motor_count_6020 > MAX_MOTOR_COUNT
                              ? MAX_MOTOR_COUNT
                              : motor_count_6020) {
    UNUSED(hw);
    UNUSED(app);
    params_3508_(0, 0) = 2.0e-07f;
    params_3508_(1, 0) = 3.0e-07f;
    k1_3508_ = params_3508_(0, 0);
    k2_3508_ = params_3508_(1, 0);
  }

  void SetMotorData3508(float* output_current, float* rotorspeed_rpm,
                        float* speed_error = nullptr) {
    LibXR::Mutex::LockGuard lock(mutex_);
    for (int i = 0; i < motor_count_3508_; i++) {
      output_current_3508_[i] = output_current[i];
      rotorspeed_rpm_3508_[i] = rotorspeed_rpm[i];
      if (speed_error) {
        speed_error_3508_[i] = fabsf(speed_error[i]);
      }
    }
  }

  void SetMotorData6020(float* output_current, float* rotorspeed_rpm,
                        float* speed_error = nullptr) {
    LibXR::Mutex::LockGuard lock(mutex_);
    for (int i = 0; i < motor_count_6020_; i++) {
      output_current_6020_[i] = output_current[i];
      rotorspeed_rpm_6020_[i] = rotorspeed_rpm[i];
      if (speed_error) {
        speed_error_6020_[i] = fabsf(speed_error[i]);
      }
    }
  }

  void SetAllocationBias3508(const AllocationBias3508& bias) {
    LibXR::Mutex::LockGuard lock(mutex_);
    allocation_bias_3508_.enabled = bias.enabled;
    allocation_bias_3508_.reserve_fraction =
        std::clamp(bias.reserve_fraction, 0.0f, 1.0f);
    for (int i = 0; i < MAX_MOTOR_COUNT; i++) {
      allocation_bias_3508_.reserve_weight[i] =
          std::max(0.0f, bias.reserve_weight[i]);
      const float WEIGHT_SCALE = bias.allocation_weight_scale[i];
      /* 未配置时保持 1.0 */
      allocation_bias_3508_.allocation_weight_scale[i] =
          WEIGHT_SCALE > 0.0f ? WEIGHT_SCALE : 1.0f;
    }
  }

  void CalculatePowerControlParam() {
    LibXR::Mutex::LockGuard lock(mutex_);
    /* 用实测功率修正 3508 功率模型参数 */
    measured_power_ = superpower_->GetChassisPower();
    samples_3508_(0, 0) = 0;
    samples_3508_(1, 0) = 0;
    bool online = superpower_->IsOnline();

    float mechanical_power = 0;

    for (int i = 0; i < motor_count_3508_; i++) {
      samples_3508_(0, 0) += output_current_3508_[i] * output_current_3508_[i];
      samples_3508_(1, 0) += rotorspeed_rpm_3508_[i] * rotorspeed_rpm_3508_[i];
      mechanical_power +=
          kt_3508_ * output_current_3508_[i] * rotorspeed_rpm_3508_[i];
    }

    float residual = measured_power_ - mechanical_power - k3_chassis_;

    if (is_helm_) {
      float power_6020 = 0;
      for (int i = 0; i < motor_count_6020_; i++) {
        power_6020 += calculate_motor_model_power(output_current_6020_[i],
                                                  rotorspeed_rpm_6020_[i],
                                                  kt_6020_, k1_6020_, k2_6020_);
      }
      residual -= power_6020;
    }

    if (residual > 0 && online && measured_power_ > 5.0f) {
      params_3508_ = rls_.Update(samples_3508_, residual);
      k1_3508_ = static_cast<float>(fmax(params_3508_(0, 0), 1.0e-07f));
      k2_3508_ = static_cast<float>(fmax(params_3508_(1, 0), 1.0e-07f));
    }
  }

  void OutputLimit(float max_power) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (is_helm_) {
      OutputLimitHelm(max_power);
    } else {
      OutputLimitOmni(max_power);
    }
  }

  PowerControlData GetPowerControlData() {
    LibXR::Mutex::LockGuard lock(mutex_);
    return powercontrol_data_;
  }

  float GetMeasuredPower() const { return measured_power_; }

  float GetCapEnergy() { return superpower_->GetCapEnergy(); }

  bool IsOnline() { return superpower_->IsOnline(); }

  void OnMonitor() override {}

 private:
  void OutputLimitOmni(float max_power) {
    float required_power_3508_sum = 0.0f;
    float available_power =
        max_power - k3_chassis_ - CHASSIS_POWER_LIMIT_MARGIN_W;

    /* 正功率参与分配, 负功率等效增加可用功率 */
    sum_error_ = 0.0f;
    for (int i = 0; i < motor_count_3508_; i++) {
      motor_power_3508_[i] = calculate_motor_model_power(
          output_current_3508_[i], rotorspeed_rpm_3508_[i], kt_3508_, k1_3508_,
          k2_3508_);

      if (motor_power_3508_[i] > 0) {
        required_power_3508_sum += motor_power_3508_[i];
        sum_error_ += speed_error_3508_[i];
      } else {
        available_power -= motor_power_3508_[i];
      }
    }

    if (required_power_3508_sum > available_power) {
      powercontrol_data_.is_power_limited = true;

      /* 计算误差置信度: sum_error 越大, 越倾向按误差分配功率 */
      if (sum_error_ > ERROR_POWERDISTRIBUTION_SET) {
        error_confidence_ = 1.0f;
      } else if (sum_error_ > POP_POWERDISTRIBUTION) {
        error_confidence_ = std::clamp(
            (sum_error_ - static_cast<float>(POP_POWERDISTRIBUTION)) /
                static_cast<float>(ERROR_POWERDISTRIBUTION_SET -
                                   POP_POWERDISTRIBUTION),
            0.0f, 1.0f);
      } else {
        error_confidence_ = 0.0f;
      }

      if (!allocation_bias_3508_.enabled) {
        /* 默认按误差/需求混合权重分配 */
        for (int i = 0; i < motor_count_3508_; i++) {
          if (motor_power_3508_[i] > 0 && required_power_3508_sum > 1e-6f) {
            /* 误差权重: 按速度跟踪误差大小分配 */
            float power_weight_error = (sum_error_ > 1e-6f)
                                           ? (speed_error_3508_[i] / sum_error_)
                                           : 0.0f;
            /* 比例权重: 按功率需求比例分配 */
            float power_weight_prop =
                motor_power_3508_[i] / required_power_3508_sum;
            /* 混合权重 */
            float power_weight = error_confidence_ * power_weight_error +
                                 (1.0f - error_confidence_) * power_weight_prop;
            float power_quota = available_power * power_weight;

            powercontrol_data_.new_output_current_3508[i] =
                solve_current_for_power(power_quota, rotorspeed_rpm_3508_[i],
                                        kt_3508_, k1_3508_, k2_3508_,
                                        output_current_3508_[i]);
          } else {
            powercontrol_data_.new_output_current_3508[i] =
                output_current_3508_[i];
          }
        }
      } else {
        /* 偏置路径: 先保底, 再分剩余功率 */
        float reserve_power_3508[MAX_MOTOR_COUNT] = {};
        float residual_power_3508[MAX_MOTOR_COUNT] = {};
        float weighted_residual_3508[MAX_MOTOR_COUNT] = {};
        const float RESERVE_POWER_POOL = std::max(0.0f, available_power) *
                                         allocation_bias_3508_.reserve_fraction;
        float reserve_weight_sum = 0.0f;
        float reserved_power_sum = 0.0f;

        for (int i = 0; i < motor_count_3508_; i++) {
          if (motor_power_3508_[i] > 0.0f) {
            reserve_weight_sum += allocation_bias_3508_.reserve_weight[i];
          }
        }

        if (RESERVE_POWER_POOL > 1e-6f && reserve_weight_sum > 1e-6f) {
          /* 保底池只分给正功率电机 */
          for (int i = 0; i < motor_count_3508_; i++) {
            if (motor_power_3508_[i] <= 0.0f) {
              continue;
            }
            const float RESERVE_QUOTA =
                RESERVE_POWER_POOL * allocation_bias_3508_.reserve_weight[i] /
                reserve_weight_sum;
            reserve_power_3508[i] =
                std::min(motor_power_3508_[i], RESERVE_QUOTA);
            reserved_power_sum += reserve_power_3508[i];
          }
        }

        /* 剩余功率再走普通分配 */
        const float REMAINING_AVAILABLE_POWER =
            std::max(0.0f, available_power - reserved_power_sum);
        float residual_required_power_sum = 0.0f;
        for (int i = 0; i < motor_count_3508_; i++) {
          residual_power_3508[i] =
              std::max(0.0f, motor_power_3508_[i] - reserve_power_3508[i]);
          residual_required_power_sum += residual_power_3508[i];
        }

        /* 剩余功率权重仍基于误差/需求, 再叠加偏置缩放 */
        float weighted_residual_sum = 0.0f;
        if (residual_required_power_sum > 1e-6f) {
          for (int i = 0; i < motor_count_3508_; i++) {
            if (residual_power_3508[i] <= 0.0f) {
              continue;
            }
            const float POWER_WEIGHT_ERROR =
                (sum_error_ > 1e-6f) ? (speed_error_3508_[i] / sum_error_)
                                     : 0.0f;
            const float POWER_WEIGHT_PROP =
                residual_power_3508[i] / residual_required_power_sum;
            const float POWER_WEIGHT =
                error_confidence_ * POWER_WEIGHT_ERROR +
                (1.0f - error_confidence_) * POWER_WEIGHT_PROP;
            weighted_residual_3508[i] =
                POWER_WEIGHT * allocation_bias_3508_.allocation_weight_scale[i];
            weighted_residual_sum += weighted_residual_3508[i];
          }
        }

        for (int i = 0; i < motor_count_3508_; i++) {
          if (motor_power_3508_[i] > 0.0f && required_power_3508_sum > 1e-6f) {
            float residual_quota = 0.0f;
            if (residual_power_3508[i] > 0.0f &&
                REMAINING_AVAILABLE_POWER > 1e-6f) {
              if (weighted_residual_sum > 1e-6f) {
                residual_quota = REMAINING_AVAILABLE_POWER *
                                 weighted_residual_3508[i] /
                                 weighted_residual_sum;
              } else if (residual_required_power_sum > 1e-6f) {
                residual_quota = REMAINING_AVAILABLE_POWER *
                                 residual_power_3508[i] /
                                 residual_required_power_sum;
              }
            }
            /* 单路电机的最终配额 = 保底配额 + 剩余池配额, 但不超过自身请求 */
            const float POWER_QUOTA = std::min(
                motor_power_3508_[i], reserve_power_3508[i] + residual_quota);
            powercontrol_data_.new_output_current_3508[i] =
                solve_current_for_power(POWER_QUOTA, rotorspeed_rpm_3508_[i],
                                        kt_3508_, k1_3508_, k2_3508_,
                                        output_current_3508_[i]);
          } else {
            powercontrol_data_.new_output_current_3508[i] =
                output_current_3508_[i];
          }
        }
      }
    } else {
      powercontrol_data_.is_power_limited = false;
      for (int i = 0; i < motor_count_3508_; i++) {
        powercontrol_data_.new_output_current_3508[i] = output_current_3508_[i];
      }
    }
  }

  void OutputLimitHelm(float max_power) {
    float required_power_3508_sum = 0.0f;
    float required_power_6020_sum = 0.0f;
    float sum_error_3508 = 0.0f;
    float sum_error_6020 = 0.0f;

    /* 舵轮需要给模型误差和控制延迟留余量 */
    float available_power =
        std::max(0.0f, max_power - k3_chassis_ - CHASSIS_POWER_LIMIT_MARGIN_W);

    for (int i = 0; i < motor_count_3508_; i++) {
      motor_power_3508_[i] = calculate_motor_model_power(
          output_current_3508_[i], rotorspeed_rpm_3508_[i], kt_3508_, k1_3508_,
          k2_3508_);

      if (motor_power_3508_[i] > 0) {
        required_power_3508_sum += motor_power_3508_[i];
        sum_error_3508 += speed_error_3508_[i];
      }
    }

    for (int i = 0; i < motor_count_6020_; i++) {
      motor_power_6020_[i] = calculate_motor_model_power(
          output_current_6020_[i], rotorspeed_rpm_6020_[i], kt_6020_, k1_6020_,
          k2_6020_);

      if (motor_power_6020_[i] > 0) {
        required_power_6020_sum += motor_power_6020_[i];
        sum_error_6020 += speed_error_6020_[i];
      }
    }

    float total_required_power =
        required_power_3508_sum + required_power_6020_sum;

    if (total_required_power > available_power) {
      powercontrol_data_.is_power_limited = true;

      /*计算 6020 组的总功率限额*/
      float limit_power_6020_total =
          std::min(required_power_6020_sum, available_power * 0.8f);

      /*计算 3508 组的总功率限额 (剩下的全部)*/
      float limit_power_3508_total =
          std::max(0.0f, available_power - limit_power_6020_total);

      /* 6020 组: 误差置信度 + 混合权重分配 */
      float ec_6020 = 0.0f;
      if (sum_error_6020 > ERROR_POWERDISTRIBUTION_SET) {
        ec_6020 = 1.0f;
      } else if (sum_error_6020 > POP_POWERDISTRIBUTION) {
        ec_6020 = std::clamp(
            (sum_error_6020 - static_cast<float>(POP_POWERDISTRIBUTION)) /
                static_cast<float>(ERROR_POWERDISTRIBUTION_SET -
                                   POP_POWERDISTRIBUTION),
            0.0f, 1.0f);
      }

      for (int i = 0; i < motor_count_6020_; i++) {
        if (motor_power_6020_[i] > 0 && required_power_6020_sum > 1e-6f) {
          float pw_err = (sum_error_6020 > 1e-6f)
                             ? (speed_error_6020_[i] / sum_error_6020)
                             : 0.0f;
          float pw_prop = motor_power_6020_[i] / required_power_6020_sum;
          float pw = ec_6020 * pw_err + (1.0f - ec_6020) * pw_prop;
          float power_quota = limit_power_6020_total * pw;

          powercontrol_data_.new_output_current_6020[i] =
              solve_current_for_power(power_quota, rotorspeed_rpm_6020_[i],
                                      kt_6020_, k1_6020_, k2_6020_,
                                      output_current_6020_[i]);
        } else {
          powercontrol_data_.new_output_current_6020[i] =
              output_current_6020_[i];
        }
      }

      /*误差置信度 + 混合权重分配 */
      float ec_3508 = 0.0f;
      if (sum_error_3508 > ERROR_POWERDISTRIBUTION_SET) {
        ec_3508 = 1.0f;
      } else if (sum_error_3508 > POP_POWERDISTRIBUTION) {
        ec_3508 = std::clamp(
            (sum_error_3508 - static_cast<float>(POP_POWERDISTRIBUTION)) /
                static_cast<float>(ERROR_POWERDISTRIBUTION_SET -
                                   POP_POWERDISTRIBUTION),
            0.0f, 1.0f);
      }

      for (int i = 0; i < motor_count_3508_; i++) {
        if (motor_power_3508_[i] > 0 && required_power_3508_sum > 1e-6f) {
          float pw_err = (sum_error_3508 > 1e-6f)
                             ? (speed_error_3508_[i] / sum_error_3508)
                             : 0.0f;
          float pw_prop = motor_power_3508_[i] / required_power_3508_sum;
          float pw = ec_3508 * pw_err + (1.0f - ec_3508) * pw_prop;
          float power_quota = limit_power_3508_total * pw;

          powercontrol_data_.new_output_current_3508[i] =
              solve_current_for_power(power_quota, rotorspeed_rpm_3508_[i],
                                      kt_3508_, k1_3508_, k2_3508_,
                                      output_current_3508_[i]);
        } else {
          powercontrol_data_.new_output_current_3508[i] =
              output_current_3508_[i];
        }
      }

    } else {
      /*功率充足，不限制*/
      powercontrol_data_.is_power_limited = false;
      for (int i = 0; i < motor_count_3508_; i++) {
        powercontrol_data_.new_output_current_3508[i] = output_current_3508_[i];
      }
      for (int i = 0; i < motor_count_6020_; i++) {
        powercontrol_data_.new_output_current_6020[i] = output_current_6020_[i];
      }
    }
  }

 private:
  LibXR::Mutex mutex_;
  SuperPower* superpower_;
  bool is_helm_;
  RLS<2> rls_; /* 在线辨识 3508 二次损耗模型参数 */
  PowerControlData powercontrol_data_;
  float k3_chassis_; /* 底盘静态功耗 */

  float error_confidence_ = 0.0f; /* 误差置信度 */
  float sum_error_ = 0.0f;

  float speed_error_3508_[MAX_MOTOR_COUNT] = {}; /* 3508速度跟踪误差 */
  float speed_error_6020_[MAX_MOTOR_COUNT] = {}; /* 6020速度跟踪误差 */
  AllocationBias3508 allocation_bias_3508_{};    /* 3508 组动态分配偏置 */

  int motor_count_3508_; /* 3508电机数目 */
  int motor_count_6020_; /* 6020电机数目 */

  float kt_3508_ = 1.99688994e-6f;
  float k1_3508_ = 0;
  float k2_3508_ = 0;
  Eigen::Matrix<float, 2, 1> samples_3508_; /* RLS 输入: [i^2_sum, rpm^2_sum] */
  Eigen::Matrix<float, 2, 1> params_3508_;  /* RLS 输出: [k1, k2] */

  float output_current_3508_[MAX_MOTOR_COUNT] = {};
  float rotorspeed_rpm_3508_[MAX_MOTOR_COUNT] = {};
  float motor_power_3508_[MAX_MOTOR_COUNT] = {};

  float kt_6020_ = 1.42074505e-5f;
  float k1_6020_ = 6.4276e-7f;
  float k2_6020_ = 1.0e-10f;

  float output_current_6020_[MAX_MOTOR_COUNT] = {};
  float rotorspeed_rpm_6020_[MAX_MOTOR_COUNT] = {};
  float motor_power_6020_[MAX_MOTOR_COUNT] = {};

  float measured_power_ = 0.0f;
};
