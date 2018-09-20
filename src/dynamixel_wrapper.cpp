#include <chrono>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <ctime>

#include "ros/ros.h"
#include "ros/console.h"

#include "../include/dynamixel_sdk/dynamixel_sdk.h" // Uses Dynamixel SDK library
#include "dyret_common/angleConv.h"

#define PROTOCOL_VERSION             2.0

#define ADDR_MX2_TORQUE_ENABLE        64
#define ADDR_MX2_STATUS_RETURN_LEVEL  68
#define ADDR_MX2_D_GAIN               80
#define ADDR_MX2_I_GAIN               82
#define ADDR_MX2_P_GAIN               84
#define ADDR_MX2_PROFILE_VELOCITY    112
#define ADDR_MX2_GOAL_POSITION       116
#define ADDR_MX2_CURRENT             126
#define ADDR_MX2_PRESENT_POSITION    132
#define ADDR_MX2_VOLTAGE             144
#define ADDR_MX2_TEMPERATURE         146

#define LEN_MX2_PROFILE_VELOCITY       4
#define LEN_MX2_GOAL_POSITION          4
#define LEN_MX2_CURRENT                2
#define LEN_MX2_PRESENT_POSITION       4
#define LEN_MX2_VOLTAGE                2
#define LEN_MX2_TEMPERATURE            1

//TODO: fix I gain when dynamixel documentation is updated
#define FACT_MX2_P_GAIN              128
#define FACT_MX2_I_GAIN                0
#define FACT_MX2_D_GAIN               16

#define BAUDRATE                 1000000

namespace dynamixel_wrapper {

dynamixel::PacketHandler *packetHandler;
dynamixel::PortHandler *portHandler;
dynamixel::GroupSyncWrite *goalAddressGroupSyncWriter;
dynamixel::GroupSyncWrite *speedGroupSyncWriter;
dynamixel::GroupBulkRead *posGroupBulkReader;
dynamixel::GroupBulkRead *currentGroupBulkReader;
dynamixel::GroupBulkRead *temperatureGroupBulkReader;
dynamixel::GroupBulkRead *voltageGroupBulkReader;

bool checkServoAlive(int givenId) {
  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_position = 0;              // Present position
  int dxl_comm_result;

  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, givenId, ADDR_MX2_PRESENT_POSITION,
                                                 &dxl_present_position,
                                                 &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_FATAL("Error id %d: %s", givenId, packetHandler->getRxPacketError(dxl_comm_result));
    return false;
  }

  return true;
}

bool verifyConnection() {

  bool returnValue = true;

  for (int i = 0; i < 12; i++) {
    if (checkServoAlive(i) == false) {
      returnValue = false;
    }
  }

  if (returnValue == true) ROS_INFO("All servos connected");
  return returnValue;
}

bool initializeServos(std::vector<int> givenServoIds){

  portHandler = dynamixel::PortHandler::getPortHandler("/dev/usb2dynamixel");
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  goalAddressGroupSyncWriter = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_MX2_GOAL_POSITION,
                                                  LEN_MX2_GOAL_POSITION);
  speedGroupSyncWriter = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_MX2_PROFILE_VELOCITY,
                                            LEN_MX2_PROFILE_VELOCITY);
  posGroupBulkReader = new dynamixel::GroupBulkRead(portHandler, packetHandler);
  currentGroupBulkReader = new dynamixel::GroupBulkRead(portHandler, packetHandler);
  temperatureGroupBulkReader = new dynamixel::GroupBulkRead(portHandler, packetHandler);
  voltageGroupBulkReader = new dynamixel::GroupBulkRead(portHandler, packetHandler);

  // Open port
  if (portHandler->openPort()) {
    ROS_INFO("OpenPort succeeded");
  } else {
    ROS_FATAL("OpenPort failed");
    return -1;
  }
  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    ROS_INFO("SetBaudRate succeeded!");
  } else {
    ROS_FATAL("SetBaudRate failed!");
    return -1;
  }

  if (verifyConnection() == false) {
    return false;
  }

  int dxl_comm_result;
  uint8_t dxl_error = 0;

  // Disable replies:
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 254, ADDR_MX2_STATUS_RETURN_LEVEL, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    return false;
  } else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    return false;
  }

  // Enable torques:
  dxl_comm_result = packetHandler->write1ByteTxOnly(portHandler, 254, ADDR_MX2_TORQUE_ENABLE, 1);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    return false;
  }

  // Init posGroupBulkReader
  for (int i = 0; i < 12; i++) {
    bool dxl_addparam_result = posGroupBulkReader->addParam(i, ADDR_MX2_PRESENT_POSITION, LEN_MX2_PRESENT_POSITION);
    if (dxl_addparam_result != true) {
      ROS_FATAL("posGroupBulkReader is not available");
      return false;
    }
  }

  // Init currentGroupBulkReader
  for (int i = 0; i < 12; i++) {
    bool dxl_addparam_result = currentGroupBulkReader->addParam(i, ADDR_MX2_CURRENT, LEN_MX2_CURRENT);
    if (dxl_addparam_result != true) {
      ROS_FATAL("currentGroupBulkReader is not available");
      return false;
    }
  }

  // Init temperatureGroupBulkReader
  for (int i = 0; i < 12; i++) {
    bool dxl_addparam_result = temperatureGroupBulkReader->addParam(i, ADDR_MX2_TEMPERATURE, LEN_MX2_TEMPERATURE);
    if (dxl_addparam_result != true) {
      ROS_FATAL("temperatureGroupBulkReader is not available");
      return false;
    }
  }

  // Init voltageGroupBulkReader
  for (int i = 0; i < 12; i++) {
    bool dxl_addparam_result = voltageGroupBulkReader->addParam(i, ADDR_MX2_VOLTAGE, LEN_MX2_VOLTAGE);
    if (dxl_addparam_result != true) {
      ROS_FATAL("voltageGroupBulkReader is not available");
      return false;
    }
  }

  return true;
}

void closeServoConnection(){
  portHandler->closePort();
}

void printCommResult(int dxl_comm_result, std::string givenString) {

  switch (dxl_comm_result) {
    case COMM_SUCCESS:
      ROS_INFO("COMM_SUCCESS (%s)", givenString.c_str());
      break;
    case COMM_PORT_BUSY:
      ROS_WARN("COMM_PORT_BUSY (%s)", givenString.c_str());
      break;
    case COMM_TX_FAIL:
      ROS_WARN("COMM_TX_FAIL (%s)", givenString.c_str());
      break;
    case COMM_RX_FAIL:
      ROS_WARN("COMM_RX_FAIL (%s)", givenString.c_str());
      break;
    case COMM_TX_ERROR:
      ROS_WARN("COMM_TX_ERROR (%s)", givenString.c_str());
      break;
    case COMM_RX_WAITING:
      ROS_WARN("COMM_RX_WAITING (%s)", givenString.c_str());
      break;
    case COMM_RX_TIMEOUT:
      ROS_WARN("COMM_RX_TIMEOUT (%s)", givenString.c_str());
      break;
    case COMM_RX_CORRUPT:
      ROS_WARN("COMM_RX_CORRUPT (%s)", givenString.c_str());
      break;
    case COMM_NOT_AVAILABLE:
      ROS_ERROR("COMM_NOT_AVAILABLE (%s)", givenString.c_str());
      break;
    default:
      ROS_ERROR("Unknown: %d (%s)", dxl_comm_result, givenString.c_str());
  }
}

bool setServoSpeeds(std::vector<int> servoIds, std::vector<float> servoSpeeds){

  if (servoIds.size() != servoSpeeds.size()) {
    ROS_ERROR("Received servo configuration with unequal lengths of servoIds (%lu) and servoSpeeds (%lu)", servoIds.size(), servoSpeeds.size());
    return false;
  }

  for (int i = 0; i < servoIds.size(); i++) {
    int convertedSpeed = int(round(1023.0 * servoSpeeds[i]));

    uint8_t param_speed[4];

    param_speed[0] = DXL_LOBYTE(DXL_LOWORD(convertedSpeed));
    param_speed[1] = DXL_HIBYTE(DXL_LOWORD(convertedSpeed));
    param_speed[2] = DXL_LOBYTE(DXL_HIWORD(convertedSpeed));
    param_speed[3] = DXL_HIBYTE(DXL_HIWORD(convertedSpeed));

    bool dxl_addparam_result = speedGroupSyncWriter->addParam((uint8_t) servoIds[i], param_speed);

    if (dxl_addparam_result != true) {
      ROS_ERROR("AddParam for speedGroupSyncWriter failed!");
      speedGroupSyncWriter->clearParam();
      return false;
    }

  }

  int dxl_comm_result = speedGroupSyncWriter->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printCommResult(dxl_comm_result, "Writing speeds");

  speedGroupSyncWriter->clearParam();

  return true;
}

void setServoAngles(std::vector<float> anglesInRad) {
  for (int i = 0; i < anglesInRad.size(); i++) {
    auto dynAngle = (int) round(((normalizeRad(anglesInRad[i]) / (2 * M_PI)) * 4095.0) + 2048.0);

    uint8_t param_goal_position[4];

    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dynAngle));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dynAngle));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dynAngle));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dynAngle));

    if (goalAddressGroupSyncWriter->addParam((uint8_t) i, param_goal_position) == false) ROS_ERROR("AddParam for goalAddressGroupSyncWriter failed");
  }

  int dxl_comm_result = goalAddressGroupSyncWriter->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printCommResult(dxl_comm_result, "syncWrite goal positions");

  printCommResult(dxl_comm_result, "syncWrite goal positions");

  goalAddressGroupSyncWriter->clearParam();

}

bool setServoPIDs(std::vector<int> servoIds, std::vector<float> servoPIDs){
  int dxl_comm_result;

  for (int i = 0; i < servoIds.size(); i++){
    dxl_comm_result = packetHandler->write2ByteTxOnly(portHandler, (uint8_t) servoIds[i], ADDR_MX2_P_GAIN, static_cast<uint16_t>(trunc(servoPIDs[i*3] * FACT_MX2_P_GAIN)));     // Write P
    if (dxl_comm_result != COMM_SUCCESS){ printCommResult(dxl_comm_result, "P_GAIN"); return false; }
    dxl_comm_result = packetHandler->write2ByteTxOnly(portHandler, (uint8_t) servoIds[i], ADDR_MX2_I_GAIN, static_cast<uint16_t>(trunc(servoPIDs[(i*3)+1] * FACT_MX2_I_GAIN))); // Write I
    if (dxl_comm_result != COMM_SUCCESS){ printCommResult(dxl_comm_result, "I_GAIN"); return false; }
    dxl_comm_result = packetHandler->write2ByteTxOnly(portHandler, (uint8_t) servoIds[i], ADDR_MX2_D_GAIN, static_cast<uint16_t>(trunc(servoPIDs[(i*3)+2] * FACT_MX2_D_GAIN))); // Write D
    if (dxl_comm_result != COMM_SUCCESS){ printCommResult(dxl_comm_result, "D_GAIN"); return false; }

  }
  return true;
}

std::vector<float> getServoAngles(std::vector<int> servoIds){
  std::vector<float> vectorToReturn(servoIds.size());

  int dxl_comm_result = posGroupBulkReader->txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS) {
    printCommResult(dxl_comm_result, "posGroupBulkReader");
    vectorToReturn.clear();
    return vectorToReturn;
  }

  bool dxl_getdata_result;
  for (int i = 0; i < 12; i++) {
    dxl_getdata_result = posGroupBulkReader->isAvailable(i, ADDR_MX2_PRESENT_POSITION, LEN_MX2_PRESENT_POSITION);
    if (dxl_getdata_result == false) {
      ROS_ERROR("posGroupBulkReader is not available");
      vectorToReturn.clear();
      return vectorToReturn;
    } else {
      int dxl_present_position = posGroupBulkReader->getData(servoIds[i], ADDR_MX2_PRESENT_POSITION, LEN_MX2_PRESENT_POSITION);

      vectorToReturn[i] = (float) dyn2rad(dxl_present_position);

    }
  }

  return vectorToReturn;
}

std::vector<float> getServoVoltages(){
  std::vector<float> vectorToReturn(12);

  int dxl_comm_result = voltageGroupBulkReader->txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS) {
    printCommResult(dxl_comm_result, "voltageGroupBulkReader");
    vectorToReturn.clear();
    return vectorToReturn;
  }

  bool dxl_getdata_result;
  for (int i = 0; i < 12; i++) {
    dxl_getdata_result = voltageGroupBulkReader->isAvailable(i, ADDR_MX2_VOLTAGE, LEN_MX2_VOLTAGE);
    if (dxl_getdata_result == false) {
      ROS_ERROR("voltageGroupBulkReader is not available");
      vectorToReturn.clear();
      return vectorToReturn;
    } else {
      vectorToReturn[i] = voltageGroupBulkReader->getData(i, ADDR_MX2_VOLTAGE, LEN_MX2_VOLTAGE) / 10.0f;
    }
  }

  return vectorToReturn;
}

std::vector<int> getServoTemperatures(){
  std::vector<int> vectorToReturn(12);

  int dxl_comm_result = temperatureGroupBulkReader->txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS) {
    printCommResult(dxl_comm_result, "currentGroupBulkReader");
    vectorToReturn.clear();
    return vectorToReturn;
  }

  bool dxl_getdata_result;
  for (int i = 0; i < 12; i++) {
    dxl_getdata_result = temperatureGroupBulkReader->isAvailable(i, ADDR_MX2_TEMPERATURE, LEN_MX2_TEMPERATURE);
    if (dxl_getdata_result == false) {
      ROS_ERROR("currentGroupBulkReader is not available");
      vectorToReturn.clear();
      return vectorToReturn;
    } else {
      vectorToReturn[i] = (int) temperatureGroupBulkReader->getData(i, ADDR_MX2_TEMPERATURE, LEN_MX2_TEMPERATURE);
    }
  }

  return vectorToReturn;
}

std::vector<float> getServoCurrents(){
  std::vector<float> vectorToReturn(12);

  int dxl_comm_result = currentGroupBulkReader->txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS) {
    printCommResult(dxl_comm_result, "currentGroupBulkReader");
    vectorToReturn.clear();
    return vectorToReturn;
  }

  bool dxl_getdata_result;
  for (int i = 0; i < 12; i++) {
    dxl_getdata_result = currentGroupBulkReader->isAvailable(i, ADDR_MX2_CURRENT, LEN_MX2_CURRENT);
    if (dxl_getdata_result == false) {
      ROS_ERROR("currentGroupBulkReader is not available");
      vectorToReturn.clear();
      return vectorToReturn;
    } else {
      vectorToReturn[i] = (4.5f * ((float) currentGroupBulkReader->getData(i, ADDR_MX2_CURRENT, LEN_MX2_CURRENT) - 2048.0f)) / 1000.0f;
    }
  }

  return vectorToReturn;
}

}
