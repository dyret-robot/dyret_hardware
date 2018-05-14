#include <chrono>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctime>

#include "ros/ros.h"
#include "ros/console.h"

#include "../include/dynamixel_sdk/dynamixel_sdk.h" // Uses Dynamixel SDK library

#include "dyret_common/Pose.h"
#include "dyret_common/ServoState.h"
#include "dyret_common/ServoStateArray.h"
#include "dyret_common/ServoConfig.h"
#include "dyret_common/ServoConfigArray.h"

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

#define BAUDRATE                 1000000

using namespace std::chrono; // Uses chrono functions for timekeeping
using namespace dynamixel;     // Uses functions defined in ROBOTIS namespace

FILE * servoLog;
bool servoLoggingEnabled;
std::vector<float> commandedPositions;

PacketHandler *packetHandler;
PortHandler *portHandler;
GroupSyncWrite *goalAddressGroupSyncWriter;
GroupSyncWrite *speedGroupSyncWriter;
GroupBulkRead *posGroupBulkReader;
GroupBulkRead *currentGroupBulkReader;
GroupBulkRead *temperatureGroupBulkReader;
GroupBulkRead *voltageGroupBulkReader;

std::vector<int> servoErrors(12);

long long unsigned int startTime;

/*int invertServoAngle(int angleInDyn){
  return (-(angleInDyn - 2048)) + 2048;
}*/

long long unsigned int getMs(){
  return std::chrono::duration_cast< std::chrono::milliseconds > (std::chrono::system_clock::now().time_since_epoch()).count();
}

void printCommResult(int dxl_comm_result, std::string givenString){

  switch (dxl_comm_result){
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

void servoConfigsCallback(const dyret_common::ServoConfigArray::ConstPtr& msg) {

  bool writeSpeeds = false;

  if (msg->servoConfigs.size() == 1){
      if (msg->servoConfigs[0].type == msg->servoConfigs[0].TYPE_DISABLE_LOG){
          servoLoggingEnabled = false;
      } else if (msg->servoConfigs[0].type == msg->servoConfigs[0].TYPE_ENABLE_LOG){
          servoLoggingEnabled = true;
      }

  } else {
    for (int i = 0; i < msg->servoConfigs.size(); i++){
      bool dxl_addparam_result;

      if (msg->servoConfigs[i].type == msg->servoConfigs[i].TYPE_SET_SPEED){
        writeSpeeds = true;

        int convertedSpeed = int(round(1023.0 * msg->servoConfigs[i].parameters[0]));

        uint8_t param_speed[4];

        param_speed[0] = DXL_LOBYTE(DXL_LOWORD(convertedSpeed));
        param_speed[1] = DXL_HIBYTE(DXL_LOWORD(convertedSpeed));
        param_speed[2] = DXL_LOBYTE(DXL_HIWORD(convertedSpeed));
        param_speed[3] = DXL_HIBYTE(DXL_HIWORD(convertedSpeed));

        dxl_addparam_result = speedGroupSyncWriter->addParam((uint8_t) msg->servoConfigs[i].id, param_speed);

        if (dxl_addparam_result != true){
            ROS_ERROR("%llu: AddParam for speedGroupSyncWriter failed!",(getMs()/1000) - startTime);
            speedGroupSyncWriter->clearParam();
            return;
        }

      } else if (msg->servoConfigs[i].type == msg->servoConfigs[i].TYPE_SET_PID){
        packetHandler->write1ByteTxOnly(portHandler, msg->servoConfigs[i].id, ADDR_MX2_P_GAIN, (int) msg->servoConfigs[i].parameters[0]); // Write P
        packetHandler->write1ByteTxOnly(portHandler, msg->servoConfigs[i].id, ADDR_MX2_I_GAIN, (int) msg->servoConfigs[i].parameters[1]); // Write I
        packetHandler->write1ByteTxOnly(portHandler, msg->servoConfigs[i].id, ADDR_MX2_D_GAIN, (int) msg->servoConfigs[i].parameters[2]); // Write D
      } else if (msg->servoConfigs[i].type == msg->servoConfigs[i].TYPE_DISABLE_TORQUE){
        packetHandler->write1ByteTxOnly(portHandler, msg->servoConfigs[i].id, ADDR_MX2_TORQUE_ENABLE, 0);
      } else if (msg->servoConfigs[i].type == msg->servoConfigs[i].TYPE_ENABLE_TORQUE) {
        packetHandler->write1ByteTxOnly(portHandler, msg->servoConfigs[i].id, ADDR_MX2_TORQUE_ENABLE, 1);
      } else {
        ROS_ERROR("Unknown configType detected!");
      }

    }

    if (writeSpeeds == true){
        int dxl_comm_result = speedGroupSyncWriter->txPacket();
        if (dxl_comm_result != COMM_SUCCESS) printCommResult(dxl_comm_result, "Writing speeds");

        // Clear syncwrite parameter storage
        speedGroupSyncWriter->clearParam();
    }

  }
}

long long int lastTime;

void dynCommandsCallback(const dyret_common::Pose::ConstPtr& msg) {

  //long long unsigned int initTime = getMs();

  for (int i = 0; i < msg->revolute.size(); i++){
    //if (i != 0 && i != 3 && i != 6 && i != 9) {
      int dynAngle = round(((normalizeRad(msg->revolute[i]) / (2 * M_PI)) * 4095.0) + 2048.0);

      //ROS_ERROR("%d: %.2f (%d)", i, msg->angle[i], dynAngle);

      commandedPositions[i] = dynAngle;

      uint8_t param_goal_position[4];

      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(commandedPositions[i]));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(commandedPositions[i]));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(commandedPositions[i]));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(commandedPositions[i]));

      bool dxl_addparam_result = goalAddressGroupSyncWriter->addParam((uint8_t) i, param_goal_position);
      if (dxl_addparam_result != true)
        ROS_ERROR("%llu: AddParam for goalAddressGroupSyncWriter failed", (getMs() / 1000) - startTime);
    //}
  }

  // Syncwrite goal position
  int dxl_comm_result = goalAddressGroupSyncWriter->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printCommResult(dxl_comm_result, "syncWrite goal positions");

  printCommResult(dxl_comm_result, "syncWrite goal positions");

  // Clear syncwrite parameter storage
  goalAddressGroupSyncWriter->clearParam();

  //printf("DynCommands: %llums, (total : ", initTime - getMs());
  //printf("%llums, %.2fhz)\n", getMs() - lastTime, (1000.0 / double(getMs() - lastTime)));
  //lastTime = getMs();

}

std::vector<double> readServoAngles(){
  std::vector<double> vectorToReturn(12);

  int dxl_comm_result = posGroupBulkReader->txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS){
    printCommResult(dxl_comm_result, "posGroupBulkReader");
    vectorToReturn.clear();
    return vectorToReturn;
  }

  bool dxl_getdata_result;
  for (int i = 0; i < 12; i++){
      dxl_getdata_result = posGroupBulkReader->isAvailable(i, ADDR_MX2_PRESENT_POSITION, LEN_MX2_PRESENT_POSITION);
      if (dxl_getdata_result == false) {
          ROS_ERROR("%llu: posGroupBulkReader is not available",(getMs()/1000) - startTime);
          vectorToReturn.clear();
          return vectorToReturn;
      } else {
          int dxl_present_position = posGroupBulkReader->getData(i, ADDR_MX2_PRESENT_POSITION, LEN_MX2_PRESENT_POSITION);

          //if (i == 1 || i == 5 || i == 8 || i == 10){ // Invert
          //    vectorToReturn[i] = (double) invertServoAngle(dxl_present_position);
          //} else {
              vectorToReturn[i] = (double) dxl_present_position;

          //}
      }
  }

  /*printf("Raw angles:\n  %.2f, %.2f, %.2f\n  %.2f, %.2f, %.2f\n  %.2f, %.2f, %.2f\n  %.2f, %.2f, %.2f\n\n",
         vectorToReturn[0],  vectorToReturn[1],  vectorToReturn[2],
         vectorToReturn[3],  vectorToReturn[4],  vectorToReturn[5],
         vectorToReturn[6],  vectorToReturn[7],  vectorToReturn[8],
         vectorToReturn[9], vectorToReturn[10], vectorToReturn[11]);*/

  return vectorToReturn;
}

std::vector<int> readServoError(){

  std::vector<int> vectorToReturn(12);

  for (int i = 0; i < 12; i++) {
    uint16_t dxl_model_number;
    uint8_t dxl_error = 0;

    packetHandler->ping(portHandler, i, &dxl_model_number, &dxl_error);

    vectorToReturn[i] = dxl_error;
  }


  return vectorToReturn;
}

std::vector<int> readServoVoltage(){
  std::vector<int> vectorToReturn(12);

  int dxl_comm_result = voltageGroupBulkReader->txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS){
    printCommResult(dxl_comm_result, "voltageGroupBulkReader");
    vectorToReturn.clear();
    return vectorToReturn;
  }

  bool dxl_getdata_result;
  for (int i = 0; i < 12; i++){
    dxl_getdata_result = voltageGroupBulkReader->isAvailable(i, ADDR_MX2_VOLTAGE, LEN_MX2_VOLTAGE);
    if (dxl_getdata_result == false) {
      ROS_ERROR("%llu: voltageGroupBulkReader is not available",(getMs()/1000) - startTime);
      vectorToReturn.clear();
      return vectorToReturn;
    } else {
      vectorToReturn[i] = (int) voltageGroupBulkReader->getData(i, ADDR_MX2_VOLTAGE, LEN_MX2_VOLTAGE);
    }
  }

  return vectorToReturn;
}

std::vector<int> readServoTemperature(){
  std::vector<int> vectorToReturn(12);

  int dxl_comm_result = temperatureGroupBulkReader->txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS){
    printCommResult(dxl_comm_result, "currentGroupBulkReader");
    vectorToReturn.clear();
    return vectorToReturn;
  }

  bool dxl_getdata_result;
  for (int i = 0; i < 12; i++){
    dxl_getdata_result = temperatureGroupBulkReader->isAvailable(i, ADDR_MX2_TEMPERATURE, LEN_MX2_TEMPERATURE);
    if (dxl_getdata_result == false) {
      ROS_ERROR("%llu: currentGroupBulkReader is not available",(getMs()/1000) - startTime);
      vectorToReturn.clear();
      return vectorToReturn;
    } else {
      vectorToReturn[i] = (int) temperatureGroupBulkReader->getData(i, ADDR_MX2_TEMPERATURE, LEN_MX2_TEMPERATURE);
    }
  }

  return vectorToReturn;
}

std::vector<double> readServoCurrent(){
  std::vector<double> vectorToReturn(12);

  int dxl_comm_result = currentGroupBulkReader->txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS){
    printCommResult(dxl_comm_result, "currentGroupBulkReader");
    vectorToReturn.clear();
    return vectorToReturn;
  }

  bool dxl_getdata_result;
  for (int i = 0; i < 12; i++){
      dxl_getdata_result = currentGroupBulkReader->isAvailable(i, ADDR_MX2_CURRENT, LEN_MX2_CURRENT);
      if (dxl_getdata_result == false) {
          ROS_ERROR("%llu: currentGroupBulkReader is not available",(getMs()/1000) - startTime);
          vectorToReturn.clear();
          return vectorToReturn;
      } else {
          vectorToReturn[i] = (4.5 * ((float) currentGroupBulkReader->getData(i, ADDR_MX2_CURRENT, LEN_MX2_CURRENT) - 2048.0)) / 1000.0;
      }
  }

  return vectorToReturn;
}

bool checkServoAlive(int givenId){
  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_position = 0;              // Present position
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, givenId, ADDR_MX2_PRESENT_POSITION, &dxl_present_position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_FATAL("Error id %d: %s", givenId, packetHandler->getRxPacketError(dxl_comm_result));
    return false;
  }

  return true;
}

bool verifyConnection(){

  bool returnValue = true;

  for (int i = 0; i < 12; i++){
      if (checkServoAlive(i) == false){
          returnValue = false;
      }
  }

  if (returnValue == true) ROS_INFO("All servos connected");
  return returnValue;
}

/*
bool getServoStatuses(dyret_common::GetServoStatuses::Request  &req, dyret_common::GetServoStatuses::Response &res){
  std::vector<dyret_common::ServoStatus> servoStatuses;
  servoStatuses.resize(12);

  std::vector<int> servoTemperatures = readServoTemperature();
  std::vector<int> servoVoltage = readServoVoltage();
  readServoError();

  for (int i = 0; i < 12; i++){
    servoStatuses[i].id = i;
    if (servoTemperatures.size() == 12) servoStatuses[i].temperature = servoTemperatures[i]; else servoStatuses[i].temperature = -1;
    if (servoVoltage.size() == 12) servoStatuses[i].voltage = float(servoVoltage[i]/10.0); else servoStatuses[i].voltage = -1;
    servoStatuses[i].status = servoErrors[i];
  }

  dyret_common::ServoStatusArray servoStatusArrayMsg;
  servoStatusArrayMsg.servoStatuses = servoStatuses;
  servoStatuses_pub.publish(servoStatusArrayMsg);

  res.servoStatuses = servoStatuses;
}*/

int main(int argc, char **argv){
    startTime = getMs() / 1000;

    servoLoggingEnabled = true;
    commandedPositions.resize(12);

    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    char fileNameBuffer[50];
    sprintf(fileNameBuffer,"servoLogs/servo_%04u%02u%02u%02u%02u.csv", now->tm_year+1900, now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min);

    servoLog = fopen(fileNameBuffer, "w");

    // Init ROS
    ros::init(argc, argv, "dynamixel_server");
    ros::NodeHandle n;
    sleep(5); // Delay to allow IMU node to start first

    ROS_INFO("Dynamixel_server initialized");

    ros::Subscriber servoConfigs_sub = n.subscribe("/dyret/servoConfigs", 10, servoConfigsCallback);
    ros::Subscriber dynCommands_sub = n.subscribe("/dyret/dynCommands", 1, dynCommandsCallback);

    ros::Publisher servoStates_pub = n.advertise<dyret_common::ServoStateArray>("/dyret/servoStates", 5);

    portHandler = PortHandler::getPortHandler("/dev/usb2dynamixel");
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    goalAddressGroupSyncWriter = new GroupSyncWrite(portHandler, packetHandler, ADDR_MX2_GOAL_POSITION, LEN_MX2_GOAL_POSITION);
    speedGroupSyncWriter = new GroupSyncWrite(portHandler, packetHandler, ADDR_MX2_PROFILE_VELOCITY, LEN_MX2_PROFILE_VELOCITY);
    posGroupBulkReader = new GroupBulkRead(portHandler, packetHandler);
    currentGroupBulkReader = new GroupBulkRead(portHandler, packetHandler);
    temperatureGroupBulkReader = new GroupBulkRead(portHandler, packetHandler);
    voltageGroupBulkReader = new GroupBulkRead(portHandler, packetHandler);

    // Open port
    if(portHandler->openPort() ) {
        ROS_INFO("OpenPort succeeded" );
    } else {
        ROS_FATAL("OpenPort failed" );
        return -1;
    }
    // Set port baudrate
    if(portHandler->setBaudRate(BAUDRATE)) {
        ROS_INFO("SetBaudRate succeeded!" );
    } else {
        ROS_FATAL("SetBaudRate failed!");
        return -1;
    }

    if (verifyConnection() == false){
        return -1;
    }

  int dxl_comm_result;
  uint8_t dxl_error = 0;

  // Disable replies:
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 254, ADDR_MX2_STATUS_RETURN_LEVEL, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS){
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Enable torques:
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 254, ADDR_MX2_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS){
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Enable torques:
  for (int i = 0; i < 12; i++){
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_MX2_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
  }

  // Init posGroupBulkReader
  for (int i = 0; i < 12; i++){
    bool dxl_addparam_result = posGroupBulkReader->addParam(i, ADDR_MX2_PRESENT_POSITION, LEN_MX2_PRESENT_POSITION);
    if (dxl_addparam_result != true) {
      ROS_FATAL("%llu: posGroupBulkReader is not available",(getMs()/1000) - startTime);
      return -1;
    }
  }

  // Init currentGroupBulkReader
    for (int i = 0; i < 12; i++){
      bool dxl_addparam_result = currentGroupBulkReader->addParam(i, ADDR_MX2_CURRENT, LEN_MX2_CURRENT);
      if (dxl_addparam_result != true) {
        ROS_FATAL("%llu: currentGroupBulkReader is not available",(getMs()/1000) - startTime);
        return -1;
      }
    }

  // Init temperatureGroupBulkReader
  for (int i = 0; i < 12; i++){
    bool dxl_addparam_result = temperatureGroupBulkReader->addParam(i, ADDR_MX2_TEMPERATURE, LEN_MX2_TEMPERATURE);
    if (dxl_addparam_result != true) {
      ROS_FATAL("%llu: temperatureGroupBulkReader is not available",(getMs()/1000) - startTime);
      return -1;
    }
  }

  // Init voltageGroupBulkReader
  for (int i = 0; i < 12; i++){
    bool dxl_addparam_result = voltageGroupBulkReader->addParam(i, ADDR_MX2_VOLTAGE, LEN_MX2_VOLTAGE);
    if (dxl_addparam_result != true) {
      ROS_FATAL("%llu: voltageGroupBulkReader is not available",(getMs()/1000) - startTime);
      return -1;
    }
  }

  long long int lastStatusSent = getMs();

  bool readingAngles = true;
  std::vector<double> servoAngles;
  std::vector<double> servoCurrents(12);

  while (ros::ok()){
/*
    if (getMs() - lastStatusSent > 1000){ // Send status every second
      std::vector<dyret_common::ServoStatus> servoStatuses;
      servoStatuses.resize(12);

      std::vector<int> servoTemperatures = readServoTemperature();
      std::vector<int> servoVoltage = readServoVoltage();
      //readServoError();

      for (int i = 0; i < 12; i++){
        servoStatuses[i].id = i;
        if (servoTemperatures.size() == 12) servoStatuses[i].temperature = servoTemperatures[i]; else servoStatuses[i].temperature = -1;
        if (servoVoltage.size() == 12) servoStatuses[i].voltage = float(servoVoltage[i]/10.0); else servoStatuses[i].voltage = -1;
        servoStatuses[i].status = servoErrors[i];
      }

      dyret_common::ServoStatusArray servoStatusArrayMsg;
      servoStatusArrayMsg.servoStatuses = servoStatuses;
      servoStatuses_pub.publish(servoStatusArrayMsg);

      lastStatusSent = getMs();
    }*/

    boost::array<dyret_common::ServoState,12> servoStates;

    // Update servo angles and current every other run:
    if (readingAngles == true) servoAngles = readServoAngles(); else servoCurrents = readServoCurrent();
    readingAngles = !readingAngles;

    if (servoAngles.size() != 0 && servoCurrents.size() != 0){

      // Build servoStates array
      for (int i = 0; i < servoAngles.size(); i++){
          servoStates[i].position = dyn2rad(servoAngles[i]);
          servoStates[i].current = fabs(servoCurrents[i]);
      }

      if (servoAngles.size() != 0){

          bool invalidData = false;

          for (int i = 0; i < servoStates.size(); i++){
            if (servoStates[i].position == 0 || servoStates[i].position == 4095 || servoStates[i].position == 4096){
              invalidData = true;
            }
          }

          if (invalidData == false){

            dyret_common::ServoStateArray servoStateArrayMsg;
            servoStateArrayMsg.revolute = servoStates;
            servoStates_pub.publish(servoStateArrayMsg);

/*            if (servoLoggingEnabled == true){
                ros::Time currentTime = ros::Time::now();

                fprintf(servoLog, "%llu", std::chrono::duration_cast< std::chrono::milliseconds >(system_clock::now().time_since_epoch()).count() - startTime);

                for (int i = 0; i < 12; i++){
                    fprintf(servoLog, ", %.2f", servoStates[i].position);

                    // Invert physical too
                    if (i == 1 || i == 5 || i == 8 || i == 10){ // Invert
                        fprintf(servoLog, ", %u", invertServoAngle(commandedPositions[i]));
                    } else {
                        fprintf(servoLog, ", %f", commandedPositions[i]);
                    }

                    fprintf(servoLog, ", %f", servoCurrents[i]);

                }
                fprintf(servoLog, "\n");
            }
*/
          } else ROS_WARN("%llu: Corrupt angle read!",(getMs()/1000) - startTime);
      }
    }


    ros::spinOnce();
  }

  // Close port
  portHandler->closePort();

  fclose(servoLog);

  return 0;
}
