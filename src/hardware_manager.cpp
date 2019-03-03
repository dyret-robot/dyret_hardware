#include "../include/dyret_hardware/dynamixel_wrapper.hpp"
#include "dyret_hardware/MX106.hpp"

#include "ros/ros.h"
#include "ros/console.h"

#include "dyret_common/Pose.h"
#include "dyret_common/State.h"
#include "dyret_common/Configuration.h"
#include "dyret_common/angleConv.h"
#include "dyret_hardware/ActuatorBoardState.h"
#include "dyret_hardware/ActuatorBoardCommand.h"

#include "dyret_common/Configure.h"

ros::Publisher actuatorCommandPub;
boost::array<float,8> prismaticPositions;
std::vector<float> prismaticCommands;
std::vector<float> revoluteCommands;
bool receivedCommand = false;

std::vector<dynamixel_wrapper::ServoState> states;
std::vector<dynamixel_wrapper::WriteValue> goal_pos;
std::unique_ptr<dynamixel_wrapper::Wrapper> iface;

// Received a pose message:
void poseCommandCallback(const dyret_common::Pose::ConstPtr &msg) {

  // Handle revolute:
  if (!msg->revolute.empty()) {
    receivedCommand = true;
    goal_pos.clear();

    for (size_t i = 0; i < 12; i++) {
      dynamixel_wrapper::WriteValue v;
      v.id = (uint8_t) i;
      v.value = (uint32_t) round(((normalizeRad(msg->revolute[i]) / (2 * M_PI)) * 4095.0) + 2048.0);
      goal_pos.push_back(v);
    }
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

void setLowSpeed(){
  std::vector<dynamixel_wrapper::WriteValue> speeds;

  for (size_t i = 0; i < 12; i++) {
    dynamixel_wrapper::WriteValue v;
    v.id = (uint8_t) i;
    v.value = (uint32_t) 10.0;
    speeds.push_back(v);
  }

  iface.get()->set_velocity(speeds);
}

void restartServos(){
  iface.get()->restartServos();
  sleep(3);
  iface.get()->setTorque(true);
  usleep(1000);
  setLowSpeed();
  usleep(1000);
}

bool servoConfigCallback(dyret_common::Configure::Request  &req,
                         dyret_common::Configure::Response &res) {

  std::vector<int> servoIds;
  for (size_t i = 0; i < req.configuration.revolute.ids.size(); i++) servoIds.push_back(req.configuration.revolute.ids[i]);
  std::vector<float> parameters;
  for (size_t i = 0; i < req.configuration.revolute.parameters.size(); i++) parameters.push_back(req.configuration.revolute.parameters[i]);

  switch (req.configuration.revolute.type) {
    case dyret_common::RevoluteConfig::TYPE_DISABLE_TORQUE:
      iface.get()->setTorque(false);

      break;
    case dyret_common::RevoluteConfig::TYPE_ENABLE_TORQUE:
      iface.get()->setTorque(true);
      ROS_INFO("Enabled torque");
      break;
    case dyret_common::RevoluteConfig::TYPE_SET_SPEED: {
      std::vector<dynamixel_wrapper::WriteValue> speeds;

      for (size_t i = 0; i < req.configuration.revolute.ids.size(); i++) {
        dynamixel_wrapper::WriteValue v;
        v.id = (uint8_t) i;
        v.value = (uint32_t) (req.configuration.revolute.parameters[i] * 1023.0);
        speeds.push_back(v);
      }

      iface.get()->set_velocity(speeds);

      break;
    }
    case dyret_common::RevoluteConfig::TYPE_SET_PID:
      /*if(dynamixel_wrapper::setServoPIDs(servoIds, parameters)){
        ROS_INFO("Servo PIDs set");
      } else {
        ROS_INFO("Servo PIDs NOT set");
      }*/
      ROS_ERROR("Servo PIDs NOT set");
      break;
    case dyret_common::RevoluteConfig::TYPE_RESTART:
      ROS_ERROR("Restarting servos");
      restartServos();
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

  dynamixel_wrapper::Wrapper wrapper;

  iface = std::unique_ptr<dynamixel_wrapper::Wrapper>(new dynamixel_wrapper::Wrapper());

  ros::Publisher servoStates_pub = n.advertise<dyret_common::State>("/dyret/state", 5);
  actuatorCommandPub = n.advertise<dyret_hardware::ActuatorBoardCommand>("/dyret/actuator_board/command", 1);

  ros::ServiceServer service = n.advertiseService("/dyret/configuration", servoConfigCallback);
  ros::Subscriber poseCommand_sub = n.subscribe("/dyret/command", 1, poseCommandCallback);
  ros::Subscriber actuatorBoardStates_sub = n.subscribe("/dyret/actuator_board/state", 1, actuatorBoardStatesCallback);

  std::vector<int> servoIds = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  prismaticCommands = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  revoluteCommands  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  int counter = 0;

  iface.get()->setTorque(true);

  while (ros::ok()) {

    auto res = iface.get()->read_state(states);

    // Send angles to servos:
    if (receivedCommand) {
      iface.get()->set_goal_position(goal_pos);
    }

    // Prepare and send message
    dyret_common::State servoStates;
    servoStates.header.stamp = ros::Time().now();

    // Copy data
    for(size_t i = 0; i < states.size(); i++) {
      const size_t id = static_cast<size_t>(states[i].id);
      servoStates.revolute[id].position = dyn2rad(states[i].position);
      servoStates.revolute[id].velocity = mx106::velocity_from_raw(states[i].velocity);
      servoStates.revolute[id].current = mx106::current_from_raw(states[i].current);
      servoStates.revolute[id].voltage = mx106::voltage_from_raw(states[i].input_voltage);
      servoStates.revolute[id].temperature = mx106::temp_from_raw(states[i].temperature);
      if(goal_pos.size() > id) {
        servoStates.revolute[id].set_point = dyn2rad(goal_pos[i].value);
        servoStates.revolute[id].error = servoStates.revolute[id].position - servoStates.revolute[id].set_point;
      }
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

  //dynamixel_wrapper::closeServoConnection();
  ros::shutdown();
  return 0;
}
