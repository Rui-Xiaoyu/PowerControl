#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>

#include "Chassis.hpp"
#include "app_framework.hpp"
#include "matrix.h"
#include "message.hpp"
#include "thread.hpp"
#include "RLS.hpp"
#include "SuperPower.hpp"

struct PowerControlData {
  float new_output_current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  bool enable = false;
};

template <typename ChassisType>
class PowerControl : public LibXR::Application {
 public:
  PowerControl(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
               uint32_t task_stack_depth, ChassisType *chassis,
               SuperPower *superpower)
      : topic_powercontrol_data_("powercontrol_data", sizeof(powercontrol_data_)),
        chassis_(chassis),
        superpower_(superpower),
        rls_(1e-5f, 0.99999f) {

        UNUSED(hw);
        UNUSED(app);

    params_[0][0] = 0.0f;
    params_[1][0] = 0.0f;
    k1_ = params_[0][0];
    k2_ = params_[1][0];
    thread_.Create(this, ThreadFunction, "PowerControlThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  // 禁止拷贝和移动，避免因 RLS<2> 不能拷贝导致的错误
  PowerControl(const PowerControl &) = delete;
  PowerControl &operator=(const PowerControl &) = delete;
  PowerControl(PowerControl &&) = delete;
  PowerControl &operator=(PowerControl &&) = delete;

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
      powercontrol->OutputLimit(10.0);
      powercontrol->topic_powercontrol_data_.Publish(powercontrol->powercontrol_data_);
      powercontrol->mutex_.Unlock();

      powercontrol->thread_.Sleep(2);
    }
  }

  void CalculatePowerControlParam() {
    machane_power_ = 0.0f;
    samples_[0][0] = 0.0f;
    samples_[1][0] = 0.0f;

    for(int i = 0; i < 4; i++) {
      machane_power_ += kt_ * motor_data_.output_current[i] * motor_data_.rotorspeed_rpm[i];
      samples_[0][0] += motor_data_.output_current[i] * motor_data_.output_current[i];
      samples_[1][0] += motor_data_.rotorspeed_rpm[i] * motor_data_.rotorspeed_rpm[i];
    }

      params_ = rls_.Update(samples_, measured_power_ - machane_power_ - k3_);
      k1_ = fmax(params_[0][0], 2.0e-7);
      k2_ = fmax(params_[1][0], 2.0e-7);

      estimated_power_ = machane_power_ + k1_ * samples_[0][0] + k2_ * samples_[1][0] + k3_;
  }

  void OutputLimit(float max_power) {

    chassis_power_   = 0.0f;
    sum_error_       = 0.0f;
    required_power_  = 0.0f;
    allocated_power_ = max_power;

    for(int i =0; i<4;i++){
      motor_power_[i] = kt_ * motor_data_.output_current[i] * motor_data_.rotorspeed_rpm[i] +
                        k1_ * motor_data_.output_current[i] * motor_data_.output_current[i] +
                        k2_ * motor_data_.rotorspeed_rpm[i] * motor_data_.rotorspeed_rpm[i] +
                        k3_ / 4.0f;

      chassis_power_ += motor_power_[i];//包含负的功率 在后面判断是否超功率
      error_[i] = fabs(motor_data_.target_motor_omega_[i] - motor_data_.current_motor_omega_[i]);

      if(motor_power_[i]<0){
          allocated_power_ += -motor_power_[i]; //可分配的功率变大
        }
      else {
        sum_error_ += error_[i];
        required_power_ += motor_power_[i];//全部都是正的功率 需要的功率
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

          if(motor_power_[i]<0){
              continue;
          }
          float power_weight_error = error_[i]/ sum_error_;
          float power_weight_prop = motor_power_[i]/ required_power_;

          float power_weight = error_confidence_ * power_weight_error + (1.0f - error_confidence_) * power_weight_prop;

          float a = k1_;
          float b = kt_ * motor_data_.rotorspeed_rpm[i];
          float c = k2_ * motor_data_.rotorspeed_rpm[i] * motor_data_.rotorspeed_rpm[i] +  k3_ / 4.0f - allocated_power_ * power_weight;

          float delta = b * b - 4.0f * a * c;
          if(delta <0.0f){
            powercontrol_data_.new_output_current[i] = std::clamp(-b / (2.0f * a), -16384.0f, 16384.0f);
          }
          else {
            if (motor_data_.output_current[i] > 0.0f) {
              powercontrol_data_.new_output_current[i] = std::clamp((-b + sqrtf(delta)) / (2.0f * a), -16384.0f, 16384.0f);
            } else  {
              powercontrol_data_.new_output_current[i] = std::clamp((-b - sqrtf(delta)) / (2.0f * a), -16384.0f, 16384.0f);
            }
          }
          scaled_motor_power_[i] = kt_ * powercontrol_data_.new_output_current[i] * motor_data_.rotorspeed_rpm[i] +
                                   k1_ * powercontrol_data_.new_output_current[i] * powercontrol_data_.new_output_current[i] +
                                   k2_ * motor_data_.rotorspeed_rpm[i] * motor_data_.rotorspeed_rpm[i] + k3_ / 4.0f;
          chassis_power_ += scaled_motor_power_[i];

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

  ChassisType *chassis_;
  SuperPower *superpower_;

  Matrixf<2, 1> samples_;
  Matrixf<2, 1> params_ ; //速度 电流

  float machane_power_ = 0.0f; //机械功率
  float estimated_power_ = 0.0f; //估计功率
  float measured_power_ = 0.0f; //实测功率

  float scaled_motor_power_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float allocated_power_ = 0.0f;
  float required_power_ = 0.0f;

  float chassis_power_ = 0.0f;
  float motor_power_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float error_[4]= {0.0f, 0.0f, 0.0f, 0.0f};
  float sum_error_ = 0.0f;
  uint32_t text_1_ = 0;

  PowerControlData powercontrol_data_;

  float error_power_distribution_set_ = 120.0f;
  float prop_power_distribution_set_ = 60.0f;

  float error_confidence_ = 0.0f;

  float kt_ = 1.99688994e-6f;
  float k1_ = 0; //电流cmd
  float k2_ = 0; //转子rpm
  float k3_ = 3.5f; //失能状态下底盘的功率

  RLS<2> rls_;
};
