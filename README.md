# PowerControl

## 1. 模块作用
功率控制模块。根据功率预算限制底盘输出，保护供电系统。
Manifest 描述：Power control for chassis (supports omni and helm wheel)

## 2. 主要函数说明
1. SetMotorData3508 / SetMotorData6020: 输入电机状态。
2. CalculatePowerControlParam: 计算功率控制参数。
3. OutputLimit / OutputLimitOmni / OutputLimitHelm: 按底盘类型限幅输出。
4. GetPowerControlData: 提供限幅后的输出数据。
5. RLS 辅助类: 参数重置与估计更新。

## 3. 接入步骤
1. 添加模块并绑定 superpower。
2. 在底盘循环中传入电机状态并读取限幅结果。
3. 将返回结果用于底盘电流/力矩输出。

标准命令流程：
    xrobot_add_mod PowerControl --instance-id powercontrol
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: PowerControl
entry_header: Modules/PowerControl/PowerControl.hpp
constructor_args:
  - superpower: '@&super_power'
  - is_helm: false
  - chassis_static_power_loss: 0.5
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
[]

Depends:
[]

## 6. 代码入口
Modules/PowerControl/PowerControl.hpp
