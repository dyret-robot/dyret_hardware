#include "hardware_manager.h"

#include "ros/ros.h"
#include "ros/console.h"

#include "dyret_common/Pose.h"
#include "dyret_common/ServoState.h"
#include "dyret_common/ServoStateArray.h"
#include "dyret_common/ServoConfigs.h"
#include "dyret_common/angleConv.h"

#include "dynamixel_wrapper.h"

// Received a dynamixel message:
void dynCommandsCallback(const dyret_common::Pose::ConstPtr &msg) {

  if (msg->revolute.size() != 0) {
    dynamixel_wrapper::setServoAngles(msg->revolute);
  }

}

// Received a servo configuration message:
void servoConfigsCallback(const dyret_common::ServoConfigs::ConstPtr &msg) {
  std::vector<int> servoIds;
  for (int i = 0; i < msg->ids.size(); i++) servoIds.push_back(msg->ids[i]);
  std::vector<float> parameters;
  for (int i = 0; i < msg->parameters.size(); i++) servoIds.push_back(msg->parameters[i]);

  switch (msg->type) {
    case dyret_common::ServoConfigs::TYPE_DISABLE_LOG:
      // Disable logging here
      ROS_ERROR("Disabling servo logging not yet implemented!");
      break;
    case dyret_common::ServoConfigs::TYPE_ENABLE_LOG:
      // Enable logging here
      ROS_ERROR("Enabling servo logging not yet implemented!");
      break;
    case dyret_common::ServoConfigs::TYPE_DISABLE_TORQUE:
      ROS_ERROR("Setting servo torque not yet implemented!");
      break;
    case dyret_common::ServoConfigs::TYPE_ENABLE_TORQUE:
      ROS_ERROR("Setting servo torque not yet implemented!");
      break;
    case dyret_common::ServoConfigs::TYPE_SET_SPEED:
      dynamixel_wrapper::setServoSpeeds(servoIds, parameters);
      break;
    case dyret_common::ServoConfigs::TYPE_SET_PID:
      ROS_ERROR("Setting servo PIDs not yet implemented!");
      break;
    default:
      ROS_ERROR("Unknown servo configType detected!");
      break;
  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hardware_manager");
  ros::NodeHandle n;

  ros::Publisher servoStates_pub = n.advertise<dyret_common::ServoStateArray>("/dyret/servoStates", 5);

  ros::Subscriber servoConfigs_sub = n.subscribe("/dyret/servoConfigs", 10, servoConfigsCallback);
  ros::Subscriber dynCommands_sub = n.subscribe("/dyret/dynCommands", 1, dynCommandsCallback);

  std::vector<int> servoIds = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  sleep(3); // Delay to allow IMU node to start first before using USB serial ports

  if (dynamixel_wrapper::initializeServos(servoIds)) {
    ROS_INFO("Successfully initialized servos");
  } else {
    ROS_ERROR("Could not initialize dynamixel connection");
    ros::shutdown();
    return 1;
  }


  while (ros::ok()) {

    dyret_common::ServoStateArray servoStates;

    std::vector<float> servoAngles = dynamixel_wrapper::getServoAngles(servoIds);

    for (int i = 0; i < servoStates.revolute.size(); i++) {
      servoStates.revolute[i].position = servoAngles[i];
    }

    servoStates_pub.publish(servoStates);

    ros::spinOnce();
  }

  dynamixel_wrapper::closeServoConnection();
  ros::shutdown();
  return 0;
}
