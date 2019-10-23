#include <iostream>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <ros/publisher.h>

#include <ublox_msgs/NavRELPOSNED.h>
#include <geometry_msgs/PoseStamped.h>

class gpsLocationParser{
public:
    gpsLocationParser(ros::NodeHandle nh): _nh(nh){
        initialize();
    }

    void relGpsPosCallback(const ublox_msgs::NavRELPOSNED::ConstPtr &msg) {
        double currentPosNInMMN = (msg->relPosN + msg->relPosHPN*0.01)*10.0;
        double currentPosNInMME = (msg->relPosE + msg->relPosHPE*0.01)*10.0;

        printf("Current position: %.1fmm N, %.1fmm E\n", currentPosNInMMN, currentPosNInMME);

        geometry_msgs::PoseStamped posMsg;

        posMsg.header.stamp = ros::Time::now();

        posMsg.pose.position.x = currentPosNInMMN/1000.0;
        posMsg.pose.position.y = currentPosNInMME/1000.0;
        posMsg.pose.position.z = 0.0;

        _pub_position.publish(posMsg);

    }

    void initialize(){

        // Get node name
        _name = ros::this_node::getName();

        // Publishers
        _pub_position = _nh.advertise<geometry_msgs::PoseStamped>("/dyret/sensor/pose", 1);

        // Subscribers
        _sub_relGpsPos = _nh.subscribe<ublox_msgs::NavRELPOSNED>("/dyret_gps/navrelposned", 100, &gpsLocationParser::relGpsPosCallback, this);

        // Inform initialized
        ROS_INFO("%s: node initialized.",_name.c_str());
    }



    void spin(){
        ros::spin();
    }

private:

    // Node
    ros::NodeHandle _nh;
    std::string _name;

    // Publishers
    ros::Publisher _pub_position;

    // Subscribers
    ros::Subscriber _sub_relGpsPos;

    // Logging

    // Other

};

int main(int argc,char** argv){

    // Initialize ROS
    ros::init(argc,argv,"gpsLocationParser");
    ros::NodeHandle nh("~");

    gpsLocationParser glp(nh);
    glp.spin();

    return 0;
}