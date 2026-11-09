#ifndef GM6020_HPP
#define GM6020_HPP

#include <cstdint>

#include "Pid.hpp"
#include "main.h"

namespace steering
{

class Gm6020
{
public:
    /**
    * @brief 用于区分不同控制模式的枚举变量
    * 通过该变量控制controlstep使用的模式
    */
    enum class ControlMode : uint8_t
    {
        Position = 0U,
        Speed,
        Voltage
    };

    /**
    * @brief GM6020参数配置结构体
    * 用于将appconfig中的参数值传递给控制器，避免底层模块包含APP层的appconfig
    */
    struct Config
    {
        uint32_t id;
        uint32_t zero_raw;
        int32_t angle_sign;
        int32_t voltage_sign;
        float gear_ratio;

        Pid::Config position;
        Pid::Config speed;
        float control_dt_s;          // 控制周期
        float max_speed_rpm;
        int32_t voltage_limit_raw;
        uint32_t online_timeout_ms;
    };

    /**
    * @brief 创建一个无参数的GM6020对象
    * @note 这个对象应该是空的，所以必须调用Init初始化
    */
    Gm6020() = default;

    /*禁止复制，防止出现两个控制器独立运行*/
    Gm6020(const Gm6020 &) = delete;
    Gm6020 &operator=(const Gm6020 &) = delete;
    
    /**
    * @brief 初始化对象
    * @param hcan 收发电机信息的CAN口
    * @param config 参数结构体
    */
    void init(CAN_HandleTypeDef *hcan, const Config &config);

    /**
    * @brief CAN接收回调
    * @param header 收报文头
    * @param data 数据段
    * @param now_ms 接收到报文的时间
    * @note 这个函数负责维持在线状态，但不负责超时判断
    */
    void onRx(const CAN_RxHeaderTypeDef *header,
              const uint8_t data[8],
              uint32_t now_ms);
    
    /**
    * @brief 设置目标角度
    * @param angle_deg 目标角
    * @note 这个函数仅更改对象的位置目标，不做位置环
    */
    void setTargetAngle(float angle_deg);

    /**
    * @brief 设置目标速度和设置目标电压的函数
    * @param speed_rpm
    * @param voltage_raw 
    * @note 同上，只改参数，不做闭环，电压有限幅
    */
    void setTargetSpeed(float speed_rpm);
    void setTargetVoltage(int16_t voltage_raw);
    
    /**
    * @brief 设置控制模式
    * @param mode 模式枚举变量
    * @note 这个函数还会顺便把电机失能并重置PID状态
    */
    void setControlMode(ControlMode mode);
    
    void enable();
    void disable();
    void controlStep(uint32_t now_ms);
    HAL_StatusTypeDef sendVoltage(int16_t voltage_raw);

    static float wrap180(float angle_deg);

    static constexpr float RawPerRev = 8192.0f;
    static constexpr float DegPerRev = 360.0f;
    static constexpr int32_t MaxVoltageRaw = 25000;

    CAN_HandleTypeDef *hcan;
    uint8_t id;
    uint16_t zero_raw;
    int8_t angle_sign;
    int8_t voltage_sign;
    float gear_ratio;

    uint16_t raw_angle;
    float angle_deg;
    int16_t speed_rpm;
    int16_t current_raw;
    uint8_t temperature;
    uint8_t is_online;
    uint8_t is_enabled;
    uint32_t last_rx_ms;

    ControlMode control_mode;
    float target_angle_deg;
    float requested_speed_rpm;
    float target_speed_rpm;
    int16_t requested_voltage_raw;
    int16_t target_voltage;

    HAL_StatusTypeDef last_tx_status;
    uint32_t tx_ok_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
    uint32_t last_tx_std_id;
    int16_t last_tx_voltage;
    uint8_t last_tx_data[8];

    float max_speed_rpm;
    int16_t voltage_limit_raw;
    uint32_t online_timeout_ms;

    Pid position_pid;
    Pid speed_pid;

private:
    static float wrap360(float angle_deg);
    static float clampAbs(float value, float limit);
    float rawToAngle(uint16_t raw_angle) const;
    int16_t clampVoltage(float voltage_raw) const;
    void updateControlLimits();
};

} // namespace steering

#endif
