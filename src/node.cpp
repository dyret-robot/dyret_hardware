#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "dyret_common/Configuration.h"
#include "dyret_common/Configure.h"
#include "dyret_common/Pose.h"
#include "dyret_common/State.h"
#include "dyret_common/angleConv.h"
#include "dyret_hardware/MX106.hpp"
#include "dyret_hardware/dynamixel_wrapper.hpp"

class SyncWritter {
	public:
		SyncWritter(ros::NodeHandle nh): _nh(nh) {
			servo_states = _nh.advertise<dyret_common::State>("/dyret/state", 5);
			servo_cmd = _nh.subscribe("/dyret/command", 10, &SyncWritter::command_callback, this);
			servo_config = _nh.advertiseService("/dyret/configuration", &SyncWritter::config_callback, this);
			// Enable torque for all servos
			iface.enable(true);
		}

		// Interact with Dynamixel servos
		bool interact() {
			bool result = true;
			// Read from all servos
			auto res = iface.read_state(states);
			if(res != dynamixel_wrapper::ComError::Success) {
				if(res == dynamixel_wrapper::ComError::NotAvailable) {
					ROS_WARN("State read only got %zu replies", states.size());
				} else {
					ROS_ERROR_STREAM("Error reading from servos: " << res);
				}
				result = false;
			}
			res = iface.set_goal_position(goal_pos);
			if(res != dynamixel_wrapper::ComError::Success) {
				ROS_ERROR_STREAM("Error writing to servos: " << res);
				result = false;
			}
			// If successful we publish new state
			if(result) {
				// TODO remove:
				goal_pos.clear();
				for(size_t i = 0; i < 12; i++) {
					dynamixel_wrapper::WriteValue v;
					v.id = dynamixel_wrapper::DYRET_SERVO_IDS[i];
					v.value = states[v.id].position;
					goal_pos.push_back(v);
				}
				this->publish();
			}
			return result;
		}
	protected:
		void publish() {
			dyret_common::State publish_state;
			// Copy data
			for(size_t i = 0; i < states.size(); i++) {
				const size_t id = static_cast<size_t>(states[i].id);
				publish_state.revolute[id].position = dyn2rad(states[i].position);
				publish_state.revolute[id].velocity = mx106::velocity_from_raw(states[i].velocity);
				publish_state.revolute[id].current = mx106::current_from_raw(states[i].current);
				publish_state.revolute[id].voltage = mx106::voltage_from_raw(states[i].input_voltage);
				publish_state.revolute[id].temperature = mx106::temp_from_raw(states[i].temperature);
				if(goal_pos.size() > id) {
					publish_state.revolute[id].set_point = dyn2rad(goal_pos[i].value);
					publish_state.revolute[id].error = publish_state.revolute[id].position - publish_state.revolute[id].set_point;
				}
			}
			servo_states.publish(publish_state);
		}

		void command_callback(const dyret_common::Pose::ConstPtr& msg) {
			std::vector<dynamixel_wrapper::WriteValue> to_write;
			if(msg->revolute.size() != 0 && msg->revolute.size() == 12) {
				for(size_t i = 0; i < 12; i++) {
					dynamixel_wrapper::WriteValue v;
					v.id = dynamixel_wrapper::DYRET_SERVO_IDS[i];
					v.value = rad2dyn(msg->revolute[i]);
					to_write.push_back(v);
				}
			} else if (msg->revolute.size() != 0) {
				ROS_WARN("Pose command with invalid number of revolute commands: %zu",
						msg->revolute.size());
				return;
			}
			goal_pos = to_write;
		}

		bool config_callback(dyret_common::Configure::Request& req,
				     dyret_common::Configure::Response& res) {
			return true;
		}

	private:
		// Handler into ROS
		ros::NodeHandle _nh;
		// Interface to Dynamixel
		dynamixel_wrapper::Wrapper iface;
		// ROS publisher and subscribers
		ros::Publisher servo_states;
		ros::Subscriber servo_cmd;
		ros::ServiceServer servo_config;
		// Vectors with target values
		std::vector<dynamixel_wrapper::ServoState> states;
		std::vector<dynamixel_wrapper::WriteValue> goal_pos;
};

int main(int argc, char** argv) {
	// Initialize ROS
	ros::init(argc, argv, "dyret_hardware");
	ros::NodeHandle nh;
	// Create Dynamixel interface
	SyncWritter writer(nh);
	// Create error counter, to many and we exit
	int num_errors = 0;
	// Read - write dynamixel loop
	while(ros::ok()) {
		if(writer.interact()) {
			num_errors = 0;
		} else {
			num_errors += 1;
		}
		if(num_errors >= 15) {
			ROS_FATAL("To many errors in a row, turning off for safety");
			break;
		}
		// Check callbacks, if there is none available this will return
		// at once, if there are callback(s) available take only one
		// and run it to completion
		ros::getGlobalCallbackQueue()->callOne(ros::WallDuration(0));
	}
	return num_errors != 0;
}
