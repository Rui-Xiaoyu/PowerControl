#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - task_stack_depth: 2048
  - superpower: '@&super_power'
template_args:
  - ChassisType: Omni
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>

#include "app_framework.hpp"
#include "matrix.h"
#include "message.hpp"
#include "thread.hpp"
#include "RLS.hpp"
#include "SuperPower.hpp"

#define CHASSIS_MAX_POWER 50.0f

template <typename ChassisType>
class PowerControl : public LibXR::Application {
 public:
  struct PowerControlData {
    float new_output_current_3508[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float new_output_current_6020[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool enable = false;
  };

  PowerControl(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
               uint32_t task_stack_depth,
               SuperPower *superpower)
      : topic_powercontrol_data_("powercontrol_data", sizeof(powercontrol_data_)),
        superpower_(superpower),
        rls_(1e-5f, 0.99999f) {

        UNUSED(hw);
        UNUSED(app);

    params_3508_[0][0] = 0.0f;
    params_3508_[1][0] = 0.0f;
    k1_3508_ = params_3508_[0][0];
    k2_3508_ = params_3508_[1][0];
    thread_.Create(this, ThreadFunction, "PowerControlThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  static void ThreadFunction(PowerControl *powercontrol) {
    powercontrol->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<typename ChassisType::MotorData> motor_data_suber("motor_data");

    motor_data_suber.StartWaiting();
    powercontrol->mutex_.Unlock();

    while (true) {
      if(motor_data_suber.Available()) {
        powercontrol->motor_data_ = motor_data_suber.GetData();
        motor_data_suber.StartWaiting();
      }

      powercontrol->mutex_.Lock();
      powercontrol->measured_power_ = powercontrol->superpower_->GetChassisPower();
      powercontrol->CalculatePowerControlParam();
      powercontrol->OutputLimit(CHASSIS_MAX_POWER);
      powercontrol->topic_powercontrol_data_.Publish(powercontrol->powercontrol_data_);
      powercontrol->mutex_.Unlock();

      powercontrol->thread_.Sleep(2);
    }
  }

  /*估计3508和6020电机的功率参数*/
  void CalculatePowerControlParam() {

    /*3508的参数估计*/
    machane_power_3508_ = 0.0f;
    samples_3508_[0][0] = 0.0f;
    samples_3508_[1][0] = 0.0f;

    for(int i = 0; i < 4; i++) {
      machane_power_3508_ += kt_3508_ * motor_data_.output_current_3508[i] * motor_data_.rotorspeed_rpm_3508[i];
      samples_3508_[0][0] += motor_data_.output_current_3508[i] * motor_data_.output_current_3508[i];
      samples_3508_[1][0] += motor_data_.rotorspeed_rpm_3508[i] * motor_data_.rotorspeed_rpm_3508[i];
    }

      params_3508_ = rls_.Update(samples_3508_, measured_power_ - machane_power_3508_ - k3_3508_ - machane_power_6020_ - k3_6020_);
      k1_3508_ = fmax(params_3508_[0][0], 2.0e-7);
      k2_3508_ = fmax(params_3508_[1][0], 2.0e-7);

      estimated_power_3508_ = machane_power_3508_ + k1_3508_ * samples_3508_[0][0] + k2_3508_ * samples_3508_[1][0] + k3_3508_;

      /*6020的参数估计*/
      machane_power_6020_ = 0.0f;
      samples_6020_[0][0] = 0.0f;
      samples_6020_[1][0] = 0.0f;

      for (int i = 0; i < 4; i++) {
        machane_power_6020_ +=
            kt_6020_ * motor_data_.output_current_6020[i] * motor_data_.rotorspeed_rpm_6020[i];
        samples_6020_[0][0] +=
            motor_data_.output_current_6020[i] * motor_data_.output_current_6020[i];
        samples_6020_[1][0] +=
            motor_data_.rotorspeed_rpm_6020[i] * motor_data_.rotorspeed_rpm_6020[i];
      }

      params_6020_ = rls_.Update(samples_6020_, measured_power_ - machane_power_3508_ - k3_3508_ - machane_power_6020_ - k3_6020_);
      k1_6020_ = fmax(params_6020_[0][0], 2.0e-7);
      k2_6020_ = fmax(params_6020_[1][0], 2.0e-7);

      estimated_power_6020_ = machane_power_6020_ + k1_6020_ * samples_6020_[0][0] + k2_6020_ * samples_6020_[1][0] + k3_6020_;
  }

  void OutputLimit(float max_power) {
    chassis_power_   = 0.0f;
    sum_error_       = 0.0f;
    required_power_  = 0.0f;
    allocated_power_total_ = max_power;
    allocated_power_6020_ = max_power * 0.8f; //6020电机分配80%的功率
    for(int i =0; i<4;i++){
      motor_power_3508_[i] = kt_3508_ * motor_data_.output_current_3508[i] * motor_data_.rotorspeed_rpm_3508[i] +
                        k1_3508_ * motor_data_.output_current_3508[i] * motor_data_.output_current_3508[i] +
                        k2_3508_ * motor_data_.rotorspeed_rpm_3508[i] * motor_data_.rotorspeed_rpm_3508[i] +
                        k3_3508_ / 4.0f;

      motor_power_6020_[i] = kt_6020_ * motor_data_.output_current_6020[i] * motor_data_.rotorspeed_rpm_6020[i] +
                             k1_6020_ * motor_data_.output_current_6020[i] * motor_data_.output_current_6020[i] +
                             k2_6020_ * motor_data_.rotorspeed_rpm_6020[i] * motor_data_.rotorspeed_rpm_6020[i] +
                             k3_6020_ / 4.0f;

      chassis_power_ += (motor_power_3508_[i] + motor_power_6020_[i]);//包含负的功率 在后面判断是否超功率
      error_[i] = fabs(motor_data_.target_motor_omega_3508[i] - motor_data_.current_motor_omega_3508[i]);

      if(motor_power_3508_[i]<0 || motor_power_6020_[i]<0){
          allocated_power_total_ -= (motor_power_3508_[i] + motor_power_6020_[i]); //可分配的功率变大
        }
      else {
        sum_error_ += error_[i];
        required_power_ += (motor_power_3508_[i]+ motor_power_6020_[i]);//全部都是正的功率 需要的功率
      }
    }

    if (chassis_power_ > max_power) {
      powercontrol_data_.enable = true;
      //误差较大的话 按照误差比例来分配功率
      if(sum_error_ >error_power_distribution_set_){
        error_confidence_ = 1.0f;
      }
      else if(sum_error_ > prop_power_distribution_set_){
        error_confidence_ = std::clamp((sum_error_- prop_power_distribution_set_)/(error_power_distribution_set_ - prop_power_distribution_set_), 0.0f, 1.0f);
      }
      else {
        error_confidence_ = 0.0f;
      }

        for(int i =0; i<4;i++){
          if(motor_power_3508_[i]<0){
              continue;
          }
          /*对6020的功率分配*/
          allocated_power_6020_ = allocated_power_total_ * 0.8f;
          float a_6020 = k1_6020_;
          float b_6020 = kt_6020_ * motor_data_.rotorspeed_rpm_6020[i];
          float c_6020 = k2_6020_ * motor_data_.rotorspeed_rpm_6020[i] * motor_data_.rotorspeed_rpm_6020[i] +  k3_6020_ / 4.0f - allocated_power_6020_ /4.0f;

          float delta_6020 = b_6020 * b_6020 - 4.0f * a_6020 * c_6020;
          if(delta_6020 <0.0f){
            powercontrol_data_.new_output_current_6020[i] = std::clamp(-b_6020 / (2.0f * a_6020), -16384.0f, 16384.0f);
          }
          else {
            if (motor_data_.output_current_6020[i] > 0.0f) {
              powercontrol_data_.new_output_current_6020[i] = std::clamp((-b_6020 + sqrtf(delta_6020)) / (2.0f * a_6020), -16384.0f, 16384.0f);
            } else  {
              powercontrol_data_.new_output_current_6020[i] = std::clamp((-b_6020 - sqrtf(delta_6020)) / (2.0f * a_6020), -16384.0f, 16384.0f);
            }
          }
          scaled_motor_power_6020_[i] = kt_6020_ * powercontrol_data_.new_output_current_6020[i] * motor_data_.rotorspeed_rpm_6020[i] +
                                        k1_6020_ * powercontrol_data_.new_output_current_6020[i] * powercontrol_data_.new_output_current_6020[i] +
                                        k2_6020_ * motor_data_.rotorspeed_rpm_6020[i] * motor_data_.rotorspeed_rpm_6020[i] + k3_6020_ / 4.0f;

          /*对3508的功率分配*/
          float power_weight_error = error_[i]/ sum_error_;
          float power_weight_prop = motor_power_3508_[i]/ required_power_;

          float power_weight = error_confidence_ * power_weight_error + (1.0f - error_confidence_) * power_weight_prop;

          float a = k1_3508_;
          float b = kt_3508_ * motor_data_.rotorspeed_rpm_3508[i];
          float c = k2_3508_ * motor_data_.rotorspeed_rpm_3508[i] * motor_data_.rotorspeed_rpm_3508[i] +  k3_3508_ / 4.0f - fmin(allocated_power_total_ * power_weight, allocated_power_total_ * 0.2);

          float delta_3508 = b * b - 4.0f * a * c;
          if(delta_3508 <0.0f){
            powercontrol_data_.new_output_current_3508[i] = std::clamp(-b / (2.0f * a), -16384.0f, 16384.0f);
          }
          else {
            if (motor_data_.output_current_3508[i] > 0.0f) {
              powercontrol_data_.new_output_current_3508[i] = std::clamp((-b + sqrtf(delta_3508)) / (2.0f * a), -16384.0f, 16384.0f);
            } else  {
              powercontrol_data_.new_output_current_3508[i] = std::clamp((-b - sqrtf(delta_3508)) / (2.0f * a), -16384.0f, 16384.0f);
            }
          }
          scaled_motor_power_3508_[i] = kt_3508_ * powercontrol_data_.new_output_current_3508[i] * motor_data_.rotorspeed_rpm_3508[i] +
                                   k1_3508_ * powercontrol_data_.new_output_current_3508[i] * powercontrol_data_.new_output_current_3508[i] +
                                   k2_3508_ * motor_data_.rotorspeed_rpm_3508[i] * motor_data_.rotorspeed_rpm_3508[i] + k3_3508_ / 4.0f;
          chassis_power_ += scaled_motor_power_3508_[i] + scaled_motor_power_6020_[i];
        }
    } else {
      powercontrol_data_.enable = false;
    }
  }

  void OnMonitor() override {}

 private:
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;
  LibXR::Topic topic_powercontrol_data_;

  typedef typename ChassisType::MotorData MotorData;
  MotorData motor_data_;

  SuperPower *superpower_;

  Matrixf<2, 1> samples_3508_;
  Matrixf<2, 1> params_3508_ ; //速度 电流

  float machane_power_3508_ = 0.0f; //机械功率
  float estimated_power_3508_ = 0.0f;  // 估计功率
  float scaled_motor_power_3508_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float motor_power_3508_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  Matrixf<2, 1> samples_6020_;
  Matrixf<2, 1> params_6020_;  // 速度 电流

  float machane_power_6020_ = 0.0f;
  float estimated_power_6020_ = 0.0f;
  float allocated_power_6020_ = 0.0f;
  float scaled_motor_power_6020_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float motor_power_6020_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float measured_power_ = 0.0f; //实测功率
  float allocated_power_total_ = 0.0f;
  float required_power_ = 0.0f;

  float chassis_power_ = 0.0f;

  float error_[4]= {0.0f, 0.0f, 0.0f, 0.0f};
  float sum_error_ = 0.0f;

  float error_power_distribution_set_ = 120.0f;
  float prop_power_distribution_set_ = 60.0f;
  float error_confidence_ = 0.0f;

  /*3508的电机拟合参数*/
  float kt_3508_ = 1.99688994e-6f;
  float k1_3508_ = 0; //电流cmd
  float k2_3508_ = 0; //转子rpm
  float k3_3508_ = 6.0f; //失能状态下底盘的功率

  /*6020的功率拟合参数 全向和麦轮未用到 k3_6020_赋值为0*/
  float kt_6020_ = 1.42074505e-5f;
  float k1_6020_ = 0; //电流cmd
  float k2_6020_ = 0; //转子rpm
  float k3_6020_ = 0.0f; //失能状态下底盘的功率

  PowerControlData powercontrol_data_;
  RLS<2> rls_;
};
