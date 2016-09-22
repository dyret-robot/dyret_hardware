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

#include "include/robotis/dynamixel_sdk.h" // Uses Dynamixel SDK library

#include "dyret_common/Pose.h"
#include "dyret_common/ServoState.h"
#include "dyret_common/ServoStateArray.h"
#include "dyret_common/ServoConfig.h"
#include "dyret_common/ServoConfigArray.h"

#define PROTOCOL_VERSION                1.0
#define ADDR_MX_D_GAIN                  26
#define ADDR_MX_I_GAIN                  27
#define ADDR_MX_P_GAIN                  28
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_MOVING_SPEED            32
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_CURRENT                 68

#define LEN_MX_D_GAIN                   1
#define LEN_MX_I_GAIN                   1
#define LEN_MX_P_GAIN                   1
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_MOVING_SPEED             2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_CURRENT                  2
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"

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

long long unsigned int startTime;

int invertServoAngle(int angleInDyn){
  return (-(angleInDyn - 2048)) + 2048;
}

long long unsigned int getMs(){
  return std::chrono::duration_cast< std::chrono::milliseconds > (std::chrono::system_clock::now().time_since_epoch()).count();
}

void printCommResult(int dxl_comm_result, std::string givenString){

  switch (dxl_comm_result){
    case COMM_SUCCESS:
      ROS_INFO("COMM_SUCCESS (%s)", givenString.c_str());
      break;
    case COMM_PORT_BUSY:
      ROS_ERROR("COMM_PORT_BUSY (%s)", givenString.c_str());
      break;
    case COMM_TX_FAIL:
      ROS_ERROR("COMM_TX_FAIL (%s)", givenString.c_str());
      break;
    case COMM_RX_FAIL:
      ROS_ERROR("COMM_RX_FAIL (%s)", givenString.c_str());
      break;
    case COMM_TX_ERROR:
      ROS_ERROR("COMM_TX_ERROR (%s)", givenString.c_str());
      break;
    case COMM_RX_WAITING:
      ROS_ERROR("COMM_RX_WAITING (%s)", givenString.c_str());
      break;
    case COMM_RX_TIMEOUT:
      ROS_ERROR("COMM_RX_TIMEOUT (%s)", givenString.c_str());
      break;
    case COMM_RX_CORRUPT:
      ROS_ERROR("COMM_RX_CORRUPT (%s)", givenString.c_str());
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
      if (msg->servoConfigs[0].configType == msg->servoConfigs[0].t_disableLog){
          servoLoggingEnabled = false;
      } else if (msg->servoConfigs[0].configType == msg->servoConfigs[0].t_enableLog){
          servoLoggingEnabled = true;
      }

  } else {
    for (int i = 0; i < msg->servoConfigs.size(); i++){
      bool dxl_addparam_result;

      if (msg->servoConfigs[i].configType == msg->servoConfigs[i].t_setSpeed){
        writeSpeeds = true;

        int convertedSpeed = round(1023.0 * msg->servoConfigs[i].parameters[0]);
        dxl_addparam_result = speedGroupSyncWriter->addParam((uint8_t) msg->servoConfigs[i].servoId, (uint8_t*) &convertedSpeed);

        if (dxl_addparam_result != true){
            ROS_ERROR("%llu: AddParam for speedGroupSyncWriter failed",(getMs()/1000) - startTime);
            speedGroupSyncWriter->clearParam();
            return;
        }

      } else if (msg->servoConfigs[i].configType == msg->servoConfigs[i].t_setPID){
        packetHandler->write1ByteTxOnly(portHandler, msg->servoConfigs[i].servoId, ADDR_MX_P_GAIN, (int) msg->servoConfigs[i].parameters[0]); // Write P
        packetHandler->write1ByteTxOnly(portHandler, msg->servoConfigs[i].servoId, ADDR_MX_I_GAIN, (int) msg->servoConfigs[i].parameters[1]); // Write I
        packetHandler->write1ByteTxOnly(portHandler, msg->servoConfigs[i].servoId, ADDR_MX_D_GAIN, (int) msg->servoConfigs[i].parameters[2]); // Write D
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

void dynCommandsCallback(const dyret_common::Pose::ConstPtr& msg) {

  for (int i = 0; i < msg->angle.size(); i++){
    // Normalize to -pi -> pi

    double modRes = fmod((msg->angle[i]+M_PI), 2*M_PI);
    if (modRes < 0) modRes = modRes + 2*M_PI;
    double normalizedInput = modRes - M_PI;

    int dynAngle = round(((normalizedInput / (2 * M_PI)) * 4095.0) + 2048.0);

    commandedPositions[i] = dynAngle;

    bool dxl_addparam_result = goalAddressGroupSyncWriter->addParam((uint8_t) msg->id[i], (uint8_t*) &dynAngle);
    if (dxl_addparam_result != true) ROS_ERROR("%llu: AddParam for goalAddressGroupSyncWriter failed",(getMs()/1000) - startTime);

  }

  // Syncwrite goal position
  int dxl_comm_result = goalAddressGroupSyncWriter->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printCommResult(dxl_comm_result, "syncWrite goal positions");

  // Clear syncwrite parameter storage
  goalAddressGroupSyncWriter->clearParam();

}

std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}

const char* getDynamixelSerialPort(){
  for (int i = 0; i < 9; i++){
      std::ostringstream stringStream;
      stringStream << "ls -al /sys/class/tty/ttyUSB" << i << "//device/driver";
      std::string res = exec(stringStream.str().c_str());
      if (res.find("ftdi") != std::string::npos){
          std::ostringstream retStream;
          retStream << "/dev/ttyUSB" << i;
          ROS_INFO("Found dynamixel device on %s", retStream.str().c_str());
          return retStream.str().c_str();
      }
  }

  ROS_FATAL("Could not find dynamixel device connected. Exiting!");
  fflush(stdout);
  exit(-1);
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
      dxl_getdata_result = posGroupBulkReader->isAvailable(i, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
      if (dxl_getdata_result == false) {
          ROS_ERROR("%llu: posGroupBulkReader is not available",(getMs()/1000) - startTime);
          vectorToReturn.clear();
          return vectorToReturn;
      } else {
          int dxl_present_position = posGroupBulkReader->getData(i, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

          if (i == 1 || i == 5 || i == 8 || i == 10){ // Invert
              vectorToReturn[i] = (double) invertServoAngle(dxl_present_position);
          } else {
              vectorToReturn[i] = (double) dxl_present_position;
          }
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
      dxl_getdata_result = currentGroupBulkReader->isAvailable(i, ADDR_MX_CURRENT, LEN_MX_CURRENT);
      if (dxl_getdata_result == false) {
          ROS_ERROR("%llu: currentGroupBulkReader is not available",(getMs()/1000) - startTime);
          vectorToReturn.clear();
          return vectorToReturn;
      } else {
          vectorToReturn[i] = (4.5 * ((float) currentGroupBulkReader->getData(i, ADDR_MX_CURRENT, LEN_MX_CURRENT) - 2048.0)) / 1000.0;
      }
  }

  return vectorToReturn;
}

bool checkServoAlive(int givenId){
  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_position = 0;              // Present position
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, givenId, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    return false;
    packetHandler->printRxPacketError(dxl_error);
  }

  return true;
}

bool verifyConnection(){

  bool returnValue = true;

  for (int i = 0; i < 12; i++){
      if (checkServoAlive(i) == false){
          ROS_FATAL("Servo %u not responding", i);
          returnValue = false;
      }
  }

  ROS_INFO("All servos connected");
  return returnValue;
}

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
    sleep(1);

    ROS_INFO("Dynamixel_server initialized");

    ros::Subscriber servoConfigs_sub = n.subscribe("servoConfigs", 10, servoConfigsCallback);
    ros::Subscriber dynCommands_sub = n.subscribe("dynCommands", 1, dynCommandsCallback);

    ros::Publisher servoStates_pub = n.advertise<dyret_common::ServoStateArray>("servoStates", 5);
    //ros::Rate loop_rate(50);

    std::string serialPort = getDynamixelSerialPort();
    portHandler = PortHandler::getPortHandler(serialPort.c_str());
    //portHandler = PortHandler::getPortHandler("/dev/ttyUSB0");
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    goalAddressGroupSyncWriter = new GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
    speedGroupSyncWriter = new GroupSyncWrite(portHandler, packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED);
    posGroupBulkReader = new GroupBulkRead(portHandler, packetHandler);
    currentGroupBulkReader = new GroupBulkRead(portHandler, packetHandler);
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

  // Init posGroupBulkReader
  for (int i = 0; i < 12; i++){
    bool dxl_addparam_result = posGroupBulkReader->addParam(i, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    if (dxl_addparam_result != true) {
      ROS_FATAL("%llu: posGroupBulkReader is not available",(getMs()/1000) - startTime);
      return -1;
    }
  }

  // Init currentGroupBulkReader
    for (int i = 0; i < 12; i++){
      bool dxl_addparam_result = currentGroupBulkReader->addParam(i, ADDR_MX_CURRENT, LEN_MX_CURRENT);
      if (dxl_addparam_result != true) {
        ROS_FATAL("%llu: currentGroupBulkReader is not available",(getMs()/1000) - startTime);
        return -1;
      }
    }

  while (ros::ok()){

    std::vector<dyret_common::ServoState> servoStates;
    servoStates.resize(12);

    std::vector<double> servoAngles = readServoAngles();
    std::vector<double> servoCurrents = readServoCurrent();

    if (servoAngles.size() != 0 && servoCurrents.size() != 0){

      // Build servoStates array
      for (int i = 0; i < servoAngles.size(); i++){
          servoStates[i].id = i;
          servoStates[i].position = servoAngles[i];
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
            servoStateArrayMsg.servoStates = servoStates;
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
