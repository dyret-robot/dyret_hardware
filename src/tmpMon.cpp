#include "ros/ros.h"
#include "ros/console.h"

#include "dyret_common/State.h"

void servoStatesCallback(const dyret_common::State::ConstPtr &msg) {

    std::vector<double> temperatures;
    for (size_t i = 0; i < msg->revolute.size(); i++){
        temperatures.push_back(msg->revolute[i].temperature);
    }

    printf("Max temperature: %.1f\n  ", *std::max_element(std::begin(temperatures), std::end(temperatures)));
    for (size_t i = 0; i < temperatures.size(); i++){
        printf("%.2f ", temperatures[i]);
    }
    printf("\n\n");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tmpMon");
  ros::NodeHandle n;

  ros::Subscriber servoStates_sub = n.subscribe("/dyret/state", 1, servoStatesCallback);

  ros::Rate loop_rate(1);
  while(ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
  }

}
