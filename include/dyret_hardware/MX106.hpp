#pragma once

#include <cmath>
#include <cstdint>

#define MX106_TORQUE_ENABLE_ADDR 64
#define MX106_TORQUE_ENABLE_SIZE 1

#define MX106_GOAL_POSITION_ADDR 116
#define MX106_GOAL_POSITION_SIZE 4

#define MX106_PROFILE_VELOCITY_ADDR 112
#define MX106_PROFILE_VELOCITY_SIZE 4

#define MX106_PRESENT_PWM_ADDR 124
#define MX106_PRESENT_PWM_SIZE 2

#define MX106_PRESENT_CURRENT_ADDR 126
#define MX106_PRESENT_CURRENT_SIZE 2

#define MX106_PRESENT_VELOCITY_ADDR 128
#define MX106_PRESENT_VELOCITY_SIZE 4

#define MX106_PRESENT_POSITION_ADDR 132
#define MX106_PRESENT_POSITION_SIZE 4

#define MX106_VELOCITY_TRAJECTORY_ADDR 136
#define MX106_VELOCITY_TRAJECTORY_SIZE 4

#define MX106_POSITION_TRAJECTORY_ADDR 140
#define MX106_POSITION_TRAJECTORY_SIZE 4

#define MX106_PRESENT_INPUT_VOLTAGE_ADDR 144
#define MX106_PRESENT_INPUT_VOLTAGE_SIZE 2

#define MX106_PRESENT_TEMPERATURE_ADDR 146
#define MX106_PRESENT_TEMPERATURE_SIZE 1

#define MX106_P_GAIN_ADDR 84
#define MX106_I_GAIN_ADDR 82
#define MX106_D_GAIN_ADDR 80

#define MX106_STATUS_RETURN_LEVEL_ADDR 68

namespace mx106 {
  float inline voltage_from_raw(const uint16_t volt) {
    return static_cast<float>(volt) / 10.0f;
  }

  float inline velocity_from_raw(const uint32_t vel) {
    auto val = static_cast<float>(static_cast<int32_t>(vel));
    // Convert to RPM:
    val *= 0.229;
    // Convert to Rad/sec
    val *= 0.10472;
    return val;
  }

  float inline current_from_raw(const uint16_t current) {
    auto val = static_cast<float>(static_cast<int16_t>(current));
    // Convert to mA:
    val *= 3.36;
    return val;
  }

  float inline temp_from_raw(const uint8_t temp) {
    return static_cast<float>(temp);
  }

  float inline pwm_from_raw(const uint16_t pwm) {
    auto val = static_cast<float>(static_cast<int16_t>(pwm));
    val /= 885;
    return val;
  }
} 