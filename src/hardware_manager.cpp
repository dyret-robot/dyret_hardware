#include "dyret_hardware/MX106.hpp"
#include "dyret_hardware/dynamixel_wrapper.hpp"

#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "ros/console.h"
#include "ros/ros.h"

#include "dyret_common/Configuration.h"
#include "dyret_common/Configure.h"
#include "dyret_common/Pose.h"
#include "dyret_common/State.h"
#include "dyret_common/angleConv.h"
#include "dyret_hardware/ActuatorBoardCommand.h"
#include "dyret_hardware/ActuatorBoardState.h"

ros::Publisher actuatorCommandPub;
boost::array<float, 8> prismaticPositions;
std::vector<float> prismaticCommands;
std::vector<float> revoluteCommands;
bool receivedCommand = false;

std::vector<dynamixel_wrapper::ServoState> states;
std::vector<dynamixel_wrapper::WriteValue> goal_pos;
std::unique_ptr<dynamixel_wrapper::Wrapper> iface;
std::vector<std::pair<int, dynamixel_wrapper::HwError>> hw_status;

// Received a pose message:
void poseCommandCallback(const dyret_common::Pose::ConstPtr &msg) {

  // Handle revolute:
  if (!msg->revolute.empty()) {
    receivedCommand = true;
    goal_pos.clear();

    for (size_t i = 0; i < 12; i++) {
      dynamixel_wrapper::WriteValue v;
      v.id = (uint8_t)i;
      v.value = (uint32_t)round(
          ((normalizeRad(msg->revolute[i]) / (2 * M_PI)) * 4095.0) + 2048.0);
      goal_pos.push_back(v);
    }
  }

  // Handle prismatic:
  if (!msg->prismatic.empty()) {
    dyret_hardware::ActuatorBoardCommand actuatorCommandMsg;

    actuatorCommandMsg.length.resize((msg->prismatic.size()));
    actuatorCommandMsg.length = msg->prismatic;

    if (msg->prismatic.size() == 2) {
      prismaticCommands = {msg->prismatic[0], msg->prismatic[1],
                           msg->prismatic[0], msg->prismatic[1],
                           msg->prismatic[0], msg->prismatic[1],
                           msg->prismatic[0], msg->prismatic[1]};
    } else if (msg->prismatic.size() == 8) {
      prismaticCommands = msg->prismatic;
    } else {
      ROS_ERROR("Unsupported prismatic length!");
    }

    actuatorCommandPub.publish(actuatorCommandMsg);
  }
}

// Received an actuatorBoardState message:
void actuatorBoardStatesCallback(
    const dyret_hardware::ActuatorBoardState::ConstPtr &msg) {

  for (size_t i = 0; i < msg->position.size(); i++) {
    prismaticPositions[i] = static_cast<float>(msg->position[i]);
  }
}

void setLowSpeed() {
  std::vector<dynamixel_wrapper::WriteValue> speeds;

  for (size_t i = 0; i < 12; i++) {
    dynamixel_wrapper::WriteValue v;
    v.id = (uint8_t)i;
    v.value = (uint32_t)10.0;
    speeds.push_back(v);
  }

  iface->set_velocity(speeds);
}

void setServoPIDs() {
  std::vector<int> servoIds = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  std::vector<float> servoPIDs = {
      8.0, 0.0, 0.05, 6.6, 0.0, 0.05, 6.6, 0.0, 0.05, 8.0, 0.0, 0.05,
      6.6, 0.0, 0.05, 6.6, 0.0, 0.05, 8.0, 0.0, 0.05, 6.6, 0.0, 0.05,
      6.6, 0.0, 0.05, 8.0, 0.0, 0.05, 6.6, 0.0, 0.05, 6.6, 0.0, 0.05};

  if (iface->setServoPIDs(servoIds, servoPIDs) !=
      dynamixel_wrapper::ComError::Success) {
    ROS_ERROR("Could not set servo PIDs");
  } else {
    ROS_INFO("Servo PIDs set");
  }
}

void restartServos() {
  iface->restartServos();
  sleep(3);
  iface->setTorque(true);
  usleep(1000);
  iface->disableReplies();
  usleep(1000);
  setServoPIDs();
  usleep(1000);
  setLowSpeed();
  usleep(1000);
}

bool servoConfigCallback(dyret_common::Configure::Request &req,
                         dyret_common::Configure::Response &res) {

  std::vector<int> servoIds;
  for (size_t i = 0; i < req.configuration.revolute.ids.size(); i++)
    servoIds.push_back(req.configuration.revolute.ids[i]);
  std::vector<float> parameters;
  for (size_t i = 0; i < req.configuration.revolute.parameters.size(); i++)
    parameters.push_back(req.configuration.revolute.parameters[i]);

  switch (req.configuration.revolute.type) {
  case dyret_common::RevoluteConfig::TYPE_DISABLE_TORQUE:
    iface->setTorque(false);

    break;
  case dyret_common::RevoluteConfig::TYPE_ENABLE_TORQUE:
    iface->setTorque(true);
    ROS_INFO("Enabled torque");
    break;
  case dyret_common::RevoluteConfig::TYPE_SET_SPEED: {
    std::vector<dynamixel_wrapper::WriteValue> speeds;

    for (size_t i = 0; i < req.configuration.revolute.ids.size(); i++) {
      dynamixel_wrapper::WriteValue v;
      v.id = (uint8_t)i;
      v.value = (uint32_t)(req.configuration.revolute.parameters[i] * 1023.0);
      speeds.push_back(v);
    }

    iface->set_velocity(speeds);

    break;
  }
  case dyret_common::RevoluteConfig::TYPE_SET_PID:
    if (iface->setServoPIDs(servoIds, parameters) ==
        dynamixel_wrapper::ComError::Success) {
      ROS_INFO("Servo PIDs set");
    } else {
      ROS_INFO("Servo PIDs NOT set");
    }
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

void joint_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat,
                      const int id) {
  // Summaries can be overwritten by higher priority through `mergeSummary`
  // this is just the initial status
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Joint is good");
  const double temp = mx106::temp_from_raw(states[id].temperature);
  const double voltage = mx106::voltage_from_raw(states[id].input_voltage);
  stat.addf("current", "%.3f", mx106::current_from_raw(states[id].current));
  stat.addf("voltage", "%.1f", voltage);
  stat.addf("temperature", "%.1f", temp);
  // Add placement in body
  std::string joint, leg;
  switch (id % 3) {
  case 0:
    joint = "joint 0 (coxa)";
    break;
  case 1:
    joint = "joint 1 (femur)";
    break;
  case 2:
    joint = "joint 2 (tibia)";
    break;
  defualt:
    joint = "unknown";
    break;
  }
  switch (id / 3) {
  case 0:
    leg = "front left";
    break;
  case 1:
    leg = "front right";
    break;
  case 2:
    leg = "back right";
    break;
  case 3:
    leg = "back left";
    break;
  default:
    leg = "unknown";
    break;
  }
  stat.add("joint", joint);
  stat.add("leg", leg);
  // Add checks for warnings
  if (temp >= 50) {
    stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                       "Temperature is large: %.1f", temp);
  }
  if (temp >= 65) {
    stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                       "Temperature is very large: %.1f", temp);
  }
  if (11.1 > voltage || voltage > 14.8) {
    stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                       "Voltage is outside allowable range [11.1, 14.8]: %.1f",
                       voltage);
  }
  // Because there is a really limited number of entries in the
  // hardware error vector this is as fast as searching
  bool hw_status_found = false;
  for (const auto &pair : hw_status) {
    if (pair.first == id) {
      if (pair.second.bits.overload) {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                          "Overload detected");
      }
      if (pair.second.bits.electrical_shock) {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                          "Electrical shock detected");
      }
      if (pair.second.bits.encoder_error) {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                          "Encoder error detected");
      }
      if (pair.second.bits.overheating) {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                          "Temperature overheating detected");
      }
      if (pair.second.bits.input_voltage) {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                          "Input voltage error detected");
      }
      hw_status_found = true;
      break;
    }
  }
  if (!hw_status_found) {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
                      "No hardware status read");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hardware_manager");
  ros::NodeHandle n;

  dynamixel_wrapper::Wrapper wrapper;
  iface = std::unique_ptr<dynamixel_wrapper::Wrapper>(
      new dynamixel_wrapper::Wrapper());

  ros::Publisher servoStates_pub =
      n.advertise<dyret_common::State>("/dyret/state", 5);
  actuatorCommandPub = n.advertise<dyret_hardware::ActuatorBoardCommand>(
      "/dyret/actuator_board/command", 1);

  ros::ServiceServer service =
      n.advertiseService("/dyret/configuration", servoConfigCallback);
  ros::Subscriber poseCommand_sub =
      n.subscribe("/dyret/command", 1, poseCommandCallback);
  ros::Subscriber actuatorBoardStates_sub = n.subscribe(
      "/dyret/actuator_board/state", 1, actuatorBoardStatesCallback);

  std::vector<int> servoIds = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  // Create diagnostic updater instance which we will utilize to build error
  // monitoring
  diagnostic_updater::Updater diag;
  diag.setHardwareID("dyret");
  for (const int id : servoIds) {
    const auto name = "joint " + std::to_string(id);
    diag.add(name, [&id](diagnostic_updater::DiagnosticStatusWrapper &stat) {
      joint_diagnostic(stat, id);
    });
  }
  double min_freq = 100.;
  double max_freq = 150.;
  diagnostic_updater::TopicDiagnostic state_pub_freq(
      "state", diag,
      diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.0),
      diagnostic_updater::TimeStampStatusParam(-1., 1. / 100.));

  prismaticCommands = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  revoluteCommands = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  int counter = 0;

  iface->setTorque(true);
  usleep(1000);
  iface->disableReplies();
  usleep(1000);
  setServoPIDs();
  usleep(1000);

  // Last time we read status such as hardware error
  ros::Time last_status_read = ros::Time::now() - ros::Duration(1.0);

  while (ros::ok()) {

    const auto res = iface->read_state(states);
    if (res != dynamixel_wrapper::ComError::Success) {
      ROS_WARN_STREAM("Error reading state from servos: " << res);
    }

    // Send angles to servos:
    if (receivedCommand) {
      const auto res = iface->set_goal_position(goal_pos);
      if (res != dynamixel_wrapper::ComError::Success) {
        ROS_WARN_STREAM("Error writing goal position: " << res);
      }
    }

    // Prepare and send message
    dyret_common::State servoStates;
    servoStates.header.stamp = ros::Time::now();

    // Copy data
    for (size_t i = 0; i < states.size(); i++) {
      const size_t id = static_cast<size_t>(states[i].id);
      servoStates.revolute[id].position = dyn2rad(states[i].position);
      servoStates.revolute[id].velocity =
          mx106::velocity_from_raw(states[i].velocity);
      servoStates.revolute[id].current =
          mx106::current_from_raw(states[i].current);
      servoStates.revolute[id].voltage =
          mx106::voltage_from_raw(states[i].input_voltage);
      servoStates.revolute[id].temperature =
          mx106::temp_from_raw(states[i].temperature);
      if (goal_pos.size() > id) {
        servoStates.revolute[id].set_point = dyn2rad(goal_pos[i].value);
        servoStates.revolute[id].error = servoStates.revolute[id].position -
                                         servoStates.revolute[id].set_point;
      }
    }

    // Set for prismatic joints:
    for (size_t i = 0; i < prismaticPositions.size(); i++) {
      servoStates.prismatic[i].position = prismaticPositions[i];
      servoStates.prismatic[i].set_point = prismaticCommands[i];
      servoStates.prismatic[i].error =
          prismaticPositions[i] - prismaticCommands[i];
    }

    servoStates_pub.publish(servoStates);
    state_pub_freq.tick(servoStates.header.stamp);

    // Read additional status from servos
    if (servoStates.header.stamp - last_status_read > ros::Duration(0.1)) {
      const auto res = iface->read_hw_error(hw_status);
      if (res != dynamixel_wrapper::ComError::Success) {
        ROS_WARN_STREAM("Error reading hardware status: " << res);
      }
      last_status_read = servoStates.header.stamp;
    }

    // Potentially output diagnostic data, note that the updater
    // is internally rate limited and will not spew data
    diag.update();

    // SpinOnce to receive new messages from ROS
    ros::spinOnce();
  }

  // dynamixel_wrapper::closeServoConnection();
  ros::shutdown();
  return 0;
}
