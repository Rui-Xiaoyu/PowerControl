#pragma once
// clang-format off
/* === MODULE MANIFEST V2 ===
module_name: PowerControl
module_description: Power control for chassis (supports omni and helm wheel)
constructor_args:
  - superpower: '@&super_power'
  - is_helm: false
  - chassis_static_power_loss: 3.5
  - motor_count_3508: 4
  - motor_count_6020: 0
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>

#include "RLS.hpp"
#include "SuperPower.hpp"
#include "app_framework.hpp"
#include "matrix.h"
#include "message.hpp"
#include "thread.hpp"

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

  if (delta < 0.0f || a < 1e-9f) {
    return std::clamp(original_current * 0.5f, -16384.0f, 16384.0f);
  }

  float sqrt_delta = sqrtf(delta);
  float x1 = (-b + sqrt_delta) / (2.0f * a);
  float x2 = (-b - sqrt_delta) / (2.0f * a);

  /*选择与原电流方向一致，且绝对值更小的解（即更靠近0的电流）*/
  /* TODO:优化选择逻辑 */
  float final_current = 0;
  if (original_current >= 0) {
    if (x1 >= 0 && x1 <= original_current) {
      final_current = x1;
    } else if (x2 >= 0 && x2 <= original_current) {
      final_current = x2;
    } else {
      final_current = original_current * 0.5f;
    }
  } else {
    if (x1 <= 0 && x1 >= original_current) {
      final_current = x1;
    } else if (x2 <= 0 && x2 >= original_current) {
      final_current = x2;
    } else {
      final_current = original_current * 0.5f;
    }
  }

  return std::clamp(final_current, -16384.0f, 16384.0f);
}

struct PowerControlData {
  float new_output_current_3508[4] = {};
  float new_output_current_6020[4] = {};
  bool is_power_limited = false;
};

class PowerControl : public LibXR::Application {
 public:
  static constexpr int MAX_MOTOR_COUNT = 6;  /* 最大电机数目 */

  PowerControl(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
               SuperPower* superpower, bool is_helm = false,
               float chassis_static_power_loss = 0.0f,
               int motor_count_3508 = 4, int motor_count_6020 = 4)
      : superpower_(superpower),
        is_helm_(is_helm),
        rls_(1e-5f, 0.99999f),
        k3_chassis_(chassis_static_power_loss),
        motor_count_3508_(motor_count_3508 > MAX_MOTOR_COUNT ? MAX_MOTOR_COUNT : motor_count_3508),
        motor_count_6020_(motor_count_6020 > MAX_MOTOR_COUNT ? MAX_MOTOR_COUNT : motor_count_6020) {
    UNUSED(hw);
    UNUSED(app);
    params_3508_[0][0] = 2.0e-07f;
    params_3508_[1][0] = 3.0e-07f;
    k1_3508_ = params_3508_[0][0];
    k2_3508_ = params_3508_[1][0];
  }

  void SetMotorData3508(float* output_current, float* rotorspeed_rpm) {
    for (int i = 0; i < motor_count_3508_; i++) {
      output_current_3508_[i] = output_current[i];
      rotorspeed_rpm_3508_[i] = rotorspeed_rpm[i];
    }
  }

  void SetMotorData6020(float* output_current, float* rotorspeed_rpm) {
    for (int i = 0; i < motor_count_6020_; i++) {
      output_current_6020_[i] = output_current[i];
      rotorspeed_rpm_6020_[i] = rotorspeed_rpm[i];
    }
  }

  void CalculatePowerControlParam() {
    /*从超电得到底盘的真实功率*/
    measured_power_ = superpower_->GetChassisPower();
    samples_3508_[0][0] = 0;
    samples_3508_[1][0] = 0;
    bool online = superpower_->IsOnline();

    float mechanical_power = 0;

    for (int i = 0; i < motor_count_3508_; i++) {
      samples_3508_[0][0] += output_current_3508_[i] * output_current_3508_[i];
      samples_3508_[1][0] += rotorspeed_rpm_3508_[i] * rotorspeed_rpm_3508_[i];
      mechanical_power +=
          kt_3508_ * output_current_3508_[i] * rotorspeed_rpm_3508_[i];
    }

    /*计算残差*/
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
      k1_3508_ = static_cast<float>(fmax(params_3508_[0][0], 2.0e-07f));
      k2_3508_ = static_cast<float>(fmax(params_3508_[1][0], 3.0e-07f));
    }
  }

  void OutputLimit(float max_power) {
    if (is_helm_) {
      OutputLimitHelm(max_power);
    } else {
      OutputLimitOmni(max_power);
    }
  }

  PowerControlData GetPowerControlData() {
    PowerControlData data;
    mutex_.Lock();
    data = powercontrol_data_;
    mutex_.Unlock();
    return data;
  }

  void OnMonitor() override {}

 private:
  void OutputLimitOmni(float max_power) {
    float required_power_3508_sum = 0.0f;

    float available_power = max_power - k3_chassis_;

    for (int i = 0; i < motor_count_3508_; i++) {
      motor_power_3508_[i] = calculate_motor_model_power(
          output_current_3508_[i], rotorspeed_rpm_3508_[i], kt_3508_, k1_3508_,
          k2_3508_);

      if (motor_power_3508_[i] > 0) {
        required_power_3508_sum += motor_power_3508_[i];
      } else {
        available_power -= motor_power_3508_[i];
      }
    }

    if (required_power_3508_sum > available_power) {
      powercontrol_data_.is_power_limited = true;

      for (int i = 0; i < motor_count_3508_; i++) {
        if (motor_power_3508_[i] > 0) {
          float power_quota = available_power *
                              (motor_power_3508_[i] / required_power_3508_sum);

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
      powercontrol_data_.is_power_limited = false;
    }
  }

  void OutputLimitHelm(float max_power) {
    float required_power_3508_sum = 0.0f;
    float required_power_6020_sum = 0.0f;

    /*初始可用功率 = 最大功率 - 静态功耗*/
    float available_power = max_power - k3_chassis_;

    for (int i = 0; i < motor_count_3508_; i++) {
      motor_power_3508_[i] = calculate_motor_model_power(
          output_current_3508_[i], rotorspeed_rpm_3508_[i], kt_3508_, k1_3508_,
          k2_3508_);

      if (motor_power_3508_[i] > 0) {
        required_power_3508_sum += motor_power_3508_[i];
      } else {
        available_power -= motor_power_3508_[i];
      }
    }

    for (int i = 0; i < motor_count_6020_; i++) {
      motor_power_6020_[i] = calculate_motor_model_power(
          output_current_6020_[i], rotorspeed_rpm_6020_[i], kt_6020_, k1_6020_,
          k2_6020_);

      if (motor_power_6020_[i] > 0) {
        required_power_6020_sum += motor_power_6020_[i];
      } else {
        available_power -= motor_power_6020_[i];
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

      for (int i = 0; i < motor_count_6020_; i++) {
        if (motor_power_6020_[i] > 0) {
          /*该电机的功率配额 = 总限额 * (该电机需求 / 总需求)*/
          float power_quota = limit_power_6020_total *
                              (motor_power_6020_[i] / required_power_6020_sum);

          powercontrol_data_.new_output_current_6020[i] =
              solve_current_for_power(power_quota, rotorspeed_rpm_6020_[i],
                                      kt_6020_, k1_6020_, k2_6020_,
                                      output_current_6020_[i]);
        } else {
          /*负功或零功，不需要限制，直接通过*/
          powercontrol_data_.new_output_current_6020[i] =
              output_current_6020_[i];
        }
      }

      for (int i = 0; i < motor_count_3508_; i++) {
        if (motor_power_3508_[i] > 0) {
          /*该电机的功率配额 = 总限额 * (该电机需求 / 总需求)*/
          float power_quota = limit_power_3508_total *
                              (motor_power_3508_[i] / required_power_3508_sum);

          powercontrol_data_.new_output_current_3508[i] =
              solve_current_for_power(power_quota, rotorspeed_rpm_3508_[i],
                                      kt_3508_, k1_3508_, k2_3508_,
                                      output_current_3508_[i]);
        } else {
          // 负功或零功，不需要限制，直接通过
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
  RLS<2> rls_;
  PowerControlData powercontrol_data_;
  float k3_chassis_;        /* 底盘静态功耗 */

  int motor_count_3508_;    /* 3508电机数目 */
  int motor_count_6020_;    /* 6020电机数目 */

  float kt_3508_ = 1.99688994e-6f;
  float k1_3508_ = 0;
  float k2_3508_ = 0;
  Matrixf<2, 1> samples_3508_;
  Matrixf<2, 1> params_3508_;

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
