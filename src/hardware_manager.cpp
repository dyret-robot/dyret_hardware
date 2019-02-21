#include "hardware_manager.h"

#include "ros/ros.h"
#include "ros/console.h"

#include "dyret_common/Pose.h"
#include "dyret_common/State.h"
#include "dyret_common/Configuration.h"
#include "dyret_common/angleConv.h"
#include "dyret_hardware/ActuatorBoardState.h"
#include "dyret_hardware/ActuatorBoardCommand.h"

#include "dyret_common/Configure.h"

#include "dynamixel_wrapper.h"

ros::Publisher actuatorCommandPub;
boost::array<float,8> prismaticPositions;
std::vector<float> prismaticCommands;
std::vector<float> revoluteCommands;
bool receivedCommand = false;

// Received a pose message:
void poseCommandCallback(const dyret_common::Pose::ConstPtr &msg) {

  // Handle revolute:
  if (!msg->revolute.empty()) {
    receivedCommand = true;
    revoluteCommands = msg->revolute;
  }

  // Handle prismatic:
  if (!msg->prismatic.empty()){
    dyret_hardware::ActuatorBoardCommand actuatorCommandMsg;

    actuatorCommandMsg.length.resize((msg->prismatic.size()));
    actuatorCommandMsg.length = msg->prismatic;

    if (msg->prismatic.size() == 2){
        prismaticCommands = {msg->prismatic[0], msg->prismatic[1],
                             msg->prismatic[0], msg->prismatic[1],
                             msg->prismatic[0], msg->prismatic[1],
                             msg->prismatic[0], msg->prismatic[1]};
    } else if (msg->prismatic.size() == 8) {
        prismaticCommands = msg->prismatic;
    }else {
        ROS_ERROR("Unsupported prismatic length!");
    }


    actuatorCommandPub.publish(actuatorCommandMsg);
  }

}

// Received an actuatorBoardState message:
void actuatorBoardStatesCallback(const dyret_hardware::ActuatorBoardState::ConstPtr &msg) {

  for (size_t i = 0; i < msg->position.size(); i++){
    prismaticPositions[i] = static_cast<float>(msg->position[i]);
  }
}

bool servoConfigCallback(dyret_common::Configure::Request  &req,
                         dyret_common::Configure::Response &res) {

  std::vector<int> servoIds;
  for (size_t i = 0; i < req.configuration.revolute.ids.size(); i++) servoIds.push_back(req.configuration.revolute.ids[i]);
  std::vector<float> parameters;
  for (size_t i = 0; i < req.configuration.revolute.parameters.size(); i++) parameters.push_back(req.configuration.revolute.parameters[i]);

  switch (req.configuration.revolute.type) {
    case dyret_common::RevoluteConfig::TYPE_DISABLE_TORQUE:
      for (size_t i = 0; i < req.configuration.revolute.ids.size(); i++){
        dynamixel_wrapper::disableTorque(req.configuration.revolute.ids[i]);
      }

      break;
    case dyret_common::RevoluteConfig::TYPE_ENABLE_TORQUE:
      for (size_t i = 0; i < req.configuration.revolute.ids.size(); i++){
        dynamixel_wrapper::enableTorque(req.configuration.revolute.ids[i]);
      }
      break;
    case dyret_common::RevoluteConfig::TYPE_SET_SPEED:
      if (dynamixel_wrapper::setServoSpeeds(servoIds, parameters)) {
        ROS_INFO("Servo speeds set");
      } else {
        ROS_ERROR("Servo speeds NOT set");
      }
      break;
    case dyret_common::RevoluteConfig::TYPE_SET_PID:
      if(dynamixel_wrapper::setServoPIDs(servoIds, parameters)){
        ROS_INFO("Servo PIDs set");
      } else {
        ROS_INFO("Servo PIDs NOT set");
      }
      break;
    default:
      ROS_ERROR("Unknown servo configType detected!");
      break;
  }

  res.status = res.STATUS_NOERROR;

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hardware_manager");
  ros::NodeHandle n;

  ros::Publisher servoStates_pub = n.advertise<dyret_common::State>("/dyret/state", 5);
  actuatorCommandPub = n.advertise<dyret_hardware::ActuatorBoardCommand>("/dyret/actuator_board/command", 1);

  ros::ServiceServer service = n.advertiseService("/dyret/configuration", servoConfigCallback);
  ros::Subscriber poseCommand_sub = n.subscribe("/dyret/command", 1, poseCommandCallback);
  ros::Subscriber actuatorBoardStates_sub = n.subscribe("/dyret/actuator_board/state", 1, actuatorBoardStatesCallback);

  std::vector<int> servoIds = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  if (dynamixel_wrapper::initializeServos()) {
    ROS_INFO("Successfully initialized servos");
  } else {
    ROS_FATAL("Could not initialize dynamixel connection");
    ros::shutdown();
    return 1;
  }

  prismaticCommands = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  revoluteCommands  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  int counter = 0;
  std::vector<float> servoTemperatures = dynamixel_wrapper::getServoTemperatures();
  std::vector<float> servoCurrents = dynamixel_wrapper::getServoCurrents();
  std::vector<float> servoVoltages = dynamixel_wrapper::getServoVoltages();
  std::vector<float> servoVelocities = dynamixel_wrapper::getServoVelocities();

  while (ros::ok()) {

    // Read information from servos. Angle each time, and the others one at a time:
    std::vector<float> servoAngles = dynamixel_wrapper::getServoAngles(servoIds);

    if (counter == 0) {
        servoTemperatures = dynamixel_wrapper::getServoTemperatures();
    } else if (counter == 1){
        servoCurrents = dynamixel_wrapper::getServoCurrents();
    } else if (counter == 2){
        servoVoltages = dynamixel_wrapper::getServoVoltages();
    } else if (counter == 3){
        servoVelocities = dynamixel_wrapper::getServoVelocities();
    }

    counter++;

    if (counter == 4) counter = 0;

    // Send angles to servos:
    if (receivedCommand) {
      dynamixel_wrapper::setServoAngles(revoluteCommands);
    }

    // Prepare and send message
    dyret_common::State servoStates;
    servoStates.header.stamp = ros::Time().now();

    // Set for revolute joints:
    for (size_t i = 0; i < servoAngles.size(); i++) {
      servoStates.revolute[i].position = servoAngles[i];
      servoStates.revolute[i].temperature = servoTemperatures[i];
      servoStates.revolute[i].current = servoCurrents[i];
      servoStates.revolute[i].voltage = servoVoltages[i];
      servoStates.revolute[i].set_point = revoluteCommands[i];
      servoStates.revolute[i].error = servoAngles[i] - revoluteCommands[i];
      servoStates.revolute[i].velocity = servoVelocities[i];
    }

    // Set for prismatic joints:
    for (size_t i = 0; i < prismaticPositions.size(); i++) {
      servoStates.prismatic[i].position = prismaticPositions[i];
      servoStates.prismatic[i].set_point = prismaticCommands[i];
      servoStates.prismatic[i].error = prismaticPositions[i] - prismaticCommands[i];
    }

    servoStates_pub.publish(servoStates);

    // SpinOnce to receive new messages from ROS
    ros::spinOnce();
  }

  dynamixel_wrapper::closeServoConnection();
  ros::shutdown();
  return 0;
}
