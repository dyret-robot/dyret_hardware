#pragma once

#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include "../dynamixel_sdk/dynamixel_sdk.h" // Uses Dynamixel SDK library

namespace dynamixel_wrapper {

  // Dynamixel servo ID of all DyRET servos
  static const uint8_t DYRET_SERVO_IDS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  // Enumeration of communication errors
  enum class ComError {
    Success = 0,
    PortBusy = -1000,
    TxFail = -1001,
    RxFail = -1002,
    TxError = -2000,
    RxWaiting = -3000,
    RxTimeout = -3001,
    RxCorrupt = -3002,
    NotAvailable = -9000
  };

  // Helper function to easily print ComError
  std::ostream &operator<<(std::ostream &os, const ComError &e);

  // Struct for type safe writing
  struct WriteValue {
    // Dynamixel servo ID
    uint8_t id;
    // Value to write
    uint32_t value;
  };

  struct ServoState {
    uint8_t id;
    uint16_t pwm;
    uint16_t current;
    uint32_t velocity;
    uint32_t position;
    uint32_t velocity_trajectory;
    uint32_t position_trajectory;
    uint16_t input_voltage;
    uint8_t temperature;
  };

  typedef union {
	  struct {
		  uint8_t bit6_7: 2;
		  uint8_t overload: 1;
		  uint8_t electrical_shock: 1;
		  uint8_t encoder_error: 1;
		  uint8_t overheating: 1;
		  uint8_t bit1: 1;
		  uint8_t input_voltage: 1;
	  } bits;
	  uint8_t raw;
  } HwError;

  class Wrapper {
  public:
    // Create a new Wrapper instance
    Wrapper();

    // Enable or disable torque for all connected servos
    ComError setTorque(bool);

    // Enable or disable torque for specified servo
    ComError setTorque(uint8_t, bool);

    // Set goal position of selected servos
    ComError set_goal_position(const std::vector<WriteValue> &);

    // Set velocity of selected servos
    ComError set_velocity(const std::vector<WriteValue> &);

    // Restart all servos
    void restartServos();

    // Read the state of all servos
    // Note: Input argument is the returned state
    ComError read_state(std::vector<ServoState>&);
    // Read the Hardware error status
    ComError read_hw_error(std::vector<std::pair<int, HwError>>&);

    // Set servo PIDS
    ComError setServoPIDs(std::vector<int> servoIds, std::vector<float> servoPIDs);

    ComError disableReplies();

  protected:
    // Handler for USB port
    std::unique_ptr<dynamixel::PortHandler> port;
    // Packet handler for simpler transmissions
    std::unique_ptr<dynamixel::PacketHandler> packet;
    // Group writer objects to write all servos at once
    std::unique_ptr<dynamixel::GroupSyncWrite> goal_writer;
    std::unique_ptr<dynamixel::GroupSyncWrite> speed_writer;
    // Group reader objects to read from all servos at once
    std::unique_ptr<dynamixel::GroupSyncRead> state_reader;
    // Group reader to read hardware errors
    std::unique_ptr<dynamixel::GroupSyncRead> hw_err_reader;
  };

}
