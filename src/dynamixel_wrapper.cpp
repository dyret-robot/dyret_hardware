#include "dyret_hardware/MX106.hpp"
#include "dyret_hardware/dynamixel_wrapper.hpp"

#include <unistd.h>
#include <sstream>
#include <stdexcept>

namespace dynamixel_wrapper {

  Wrapper::Wrapper() {
    // Try to open USB port associated with DyRET
    port = std::unique_ptr<dynamixel::PortHandler>(
        dynamixel::PortHandler::getPortHandler("/dev/dyretDynamixel"));
    if (!port->setBaudRate(1000000)) {
      // If we could not open the USB port throw an exception
      throw std::runtime_error("Could not open USB port for dynamixel connection");
    }
    // Get default packet handler for version 2.0
    packet = std::unique_ptr<dynamixel::PacketHandler>(
        dynamixel::PacketHandler::getPacketHandler(2.0));
    // Setup writers
    goal_writer = std::make_unique<dynamixel::GroupSyncWrite>(port.get(), packet.get(),
                                                              MX106_GOAL_POSITION_ADDR, MX106_GOAL_POSITION_SIZE);
    speed_writer = std::make_unique<dynamixel::GroupSyncWrite>(port.get(), packet.get(),
                                                               MX106_PROFILE_VELOCITY_ADDR,
                                                               MX106_PROFILE_VELOCITY_SIZE);
    state_reader = std::make_unique<dynamixel::GroupSyncRead>(port.get(), packet.get(),
                                                              MX106_PRESENT_PWM_ADDR,
        // Read full state:
                                                              MX106_PRESENT_PWM_SIZE +
                                                              MX106_PRESENT_CURRENT_SIZE +
                                                              MX106_PRESENT_VELOCITY_SIZE +
                                                              MX106_PRESENT_POSITION_SIZE +
                                                              MX106_VELOCITY_TRAJECTORY_SIZE +
                                                              MX106_POSITION_TRAJECTORY_SIZE +
                                                              MX106_PRESENT_INPUT_VOLTAGE_SIZE +
                                                              MX106_PRESENT_TEMPERATURE_SIZE);
    // Initialize reader with IDs to read from
    for (const auto &id : DYRET_SERVO_IDS) {
      state_reader->addParam(id);
    }
    // Try to read once from all servos to check that everything is fine
    std::vector<ServoState> state;
    this->read_state(state);
    if (state.size() != 12) {
      std::ostringstream os;
      os << "Initial read failed, found " << state.size()
         << ", expected 12 servos, IDs found: [";
      for (const auto &st : state) {
        os << static_cast<int>(st.id) << ", ";
      }
      os << "]";
      throw std::runtime_error(os.str());
    }
  }

  ComError Wrapper::setTorque(bool enable) {
    const uint8_t on_off = static_cast<uint8_t>(enable);
    return static_cast<ComError>(packet->write1ByteTxOnly(port.get(), BROADCAST_ID, MX106_TORQUE_ENABLE_ADDR, on_off));
  }

  ComError Wrapper::setTorque(uint8_t id, bool enable) {
    const uint8_t on_off = static_cast<uint8_t>(enable);
    return static_cast<ComError>(packet->write1ByteTxOnly(port.get(),
                                                          id, MX106_TORQUE_ENABLE_ADDR, on_off));
  }

  ComError Wrapper::set_goal_position(const std::vector<WriteValue> &params) {
    if (params.empty()) {
      return ComError::Success;
    }
    goal_writer->clearParam();
    for (const auto &val : params) {
      uint8_t param[4];
      param[0] = DXL_LOBYTE(DXL_LOWORD(val.value));
      param[1] = DXL_HIBYTE(DXL_LOWORD(val.value));
      param[2] = DXL_LOBYTE(DXL_HIWORD(val.value));
      param[3] = DXL_HIBYTE(DXL_HIWORD(val.value));
      goal_writer->addParam(val.id, param);
    }
    return static_cast<ComError>(goal_writer->txPacket());
  }

  ComError Wrapper::set_velocity(const std::vector<WriteValue> &params) {
    speed_writer->clearParam();
    for (const auto &val : params) {
      uint8_t param[4];
      param[0] = DXL_LOBYTE(DXL_LOWORD(val.value));
      param[1] = DXL_HIBYTE(DXL_LOWORD(val.value));
      param[2] = DXL_LOBYTE(DXL_HIWORD(val.value));
      param[3] = DXL_HIBYTE(DXL_HIWORD(val.value));
      speed_writer->addParam(val.id, param);
    }
    return static_cast<ComError>(speed_writer->txPacket());
  }

  void Wrapper::restartServos() {
    for (const auto &id : DYRET_SERVO_IDS) {
      uint8_t hardwareError;
      packet->reboot(port.get(), id);
      usleep(100);
    }
  }

  ComError Wrapper::read_state(std::vector<ServoState> &out) {
    // Clear output in case we error out early
    out.clear();
    // Read from all servos
    auto res = static_cast<ComError>(state_reader->txRxPacket());
    if (res != ComError::Success) {
      return res;
    }
    // Read from all servos
    ComError err = ComError::Success;
    for (const auto &id : DYRET_SERVO_IDS) {
      ServoState st;
      st.id = id;
      if (!state_reader->isAvailable(id, MX106_PRESENT_PWM_ADDR, MX106_PRESENT_PWM_SIZE)) {
        err = ComError::NotAvailable;
        continue;
      }

      st.pwm = state_reader->getData(id, MX106_PRESENT_PWM_ADDR, MX106_PRESENT_PWM_SIZE);
      st.current = state_reader->getData(id, MX106_PRESENT_CURRENT_ADDR, MX106_PRESENT_CURRENT_SIZE);
      st.velocity = state_reader->getData(id, MX106_PRESENT_VELOCITY_ADDR, MX106_PRESENT_VELOCITY_SIZE);
      st.position = state_reader->getData(id, MX106_PRESENT_POSITION_ADDR, MX106_PRESENT_POSITION_SIZE);
      st.velocity_trajectory = state_reader->getData(id, MX106_VELOCITY_TRAJECTORY_ADDR,
                                                     MX106_VELOCITY_TRAJECTORY_SIZE);
      st.position_trajectory = state_reader->getData(id, MX106_POSITION_TRAJECTORY_ADDR,
                                                     MX106_POSITION_TRAJECTORY_SIZE);
      st.input_voltage = state_reader->getData(id, MX106_PRESENT_INPUT_VOLTAGE_ADDR, MX106_PRESENT_INPUT_VOLTAGE_SIZE);
      st.temperature = state_reader->getData(id, MX106_PRESENT_TEMPERATURE_ADDR, MX106_PRESENT_TEMPERATURE_SIZE);
      out.push_back(st);
    }
    return err;
  }

  std::ostream &operator<<(std::ostream &os, const ComError &e) {
    switch (e) {
      case ComError::Success:
        os << "communication success";
        break;
      case ComError::PortBusy:
        os << "USB port busy";
        break;
      case ComError::TxFail:
        os << "failed to transmit instruction packet";
        break;
      case ComError::RxFail:
        os << "failed to get status packet";
        break;
      case ComError::TxError:
        os << "incorrect instruction packet";
        break;
      case ComError::RxWaiting:
        os << "currently receiving status packet";
        break;
      case ComError::RxTimeout:
        os << "receiving status packet timeout";
        break;
      case ComError::RxCorrupt:
        os << "received incorrect status packet";
        break;
      case ComError::NotAvailable:
        os << "communication not available";
        break;
    }

    return os;
  }

  ComError Wrapper::disableReplies(){
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet->write1ByteTxRx(port.get(), BROADCAST_ID, MX106_STATUS_RETURN_LEVEL_ADDR, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%d: %s\n", dxl_comm_result, packet->getTxRxResult(dxl_error));
        return static_cast<ComError>(dxl_comm_result);
    } else if (dxl_error != 0) {
        printf("%d: %s\n", dxl_error, packet->getRxPacketError(dxl_error));
        return static_cast<ComError>(dxl_comm_result);
    }
    return ComError::Success;
}

}