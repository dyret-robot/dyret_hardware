// ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/server.h>
#include <dyret_hardware/CalibrateOptoforceSensors.h>

class optoforceFilter{
public:
    optoforceFilter(ros::NodeHandle nh): _nh(nh){
        initialize();
    }

    void initialize(){

        calibrating = false;
        for (int i = 0; i < 4; i++) _measurementCounter[i] = -1;
        for (int i = 0; i < 12; i++) _means[i] = 0;

        // Get node name
        _name = ros::this_node::getName();

        // Publishers
        _pub_optoforce[0] = _nh.advertise<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/fl", 100);
        _pub_optoforce[1] = _nh.advertise<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/fr", 100);
        _pub_optoforce[2] = _nh.advertise<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/br", 100);
        _pub_optoforce[3] = _nh.advertise<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/bl", 100);

        // Subscribers
        _sub_optoforce[0] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/raw/contact/fl", 100, boost::bind(&optoforceFilter::optoforceCallback, this, _1, "fl", 0));
        _sub_optoforce[1] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/raw/contact/fr", 100, boost::bind(&optoforceFilter::optoforceCallback, this, _1, "fr", 1));
        _sub_optoforce[2] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/raw/contact/br", 100, boost::bind(&optoforceFilter::optoforceCallback, this, _1, "br", 2));
        _sub_optoforce[3] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/raw/contact/bl", 100, boost::bind(&optoforceFilter::optoforceCallback, this, _1, "bl", 3));

        // Services
        _ser_calibration = _nh.advertiseService("/dyret/dyret_hardware/CalibrateOptoforceSensors", &optoforceFilter::calibrateCallback, this);

        // Inform initialized
        ROS_INFO("%s: node initialized.",_name.c_str());
    }

    float median(std::vector<float> v){
        size_t n = v.size() / 2;
        nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    }

    void optoforceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, const std::string &topic, int legIndex) {
        std::array<double, 3> measurements = {msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z};

        if (_measurementCounter[legIndex] != -1 && _measurementCounter[legIndex] < calibrationMeasurements){
            _means[legIndex*3]   += measurements[0];
            _means[legIndex*3+1] += measurements[1];
            _means[legIndex*3+2] += measurements[2];
            _measurementCounter[legIndex] += 1;
        }

        if (!calibrating){
            _measurements[legIndex*3].push_back(measurements[0] - _means[legIndex*3]);
            _measurements[legIndex*3+1].push_back(measurements[1] - _means[legIndex*3+1]);
            _measurements[legIndex*3+2].push_back(measurements[2] - _means[legIndex*3+2]);

            if (_measurements[legIndex*3].size() > medianFilterSize) _measurements[legIndex*3].erase(_measurements[legIndex*3].begin());
            if (_measurements[legIndex*3+1].size() > medianFilterSize) _measurements[legIndex*3+1].erase(_measurements[legIndex*3+1].begin());
            if (_measurements[legIndex*3+2].size() > medianFilterSize) _measurements[legIndex*3+2].erase(_measurements[legIndex*3+2].begin());

            geometry_msgs::WrenchStamped newMsg;
            newMsg.header = msg->header;
            newMsg.wrench = msg->wrench;

            newMsg.wrench.force.x = median(_measurements[legIndex*3]);
            newMsg.wrench.force.y = median(_measurements[legIndex*3+1]);
            newMsg.wrench.force.z = median(_measurements[legIndex*3+2]);

            _pub_optoforce[legIndex].publish(newMsg);

        }
    }

    bool calibrateCallback(dyret_hardware::CalibrateOptoforceSensors::Request &req,
                           dyret_hardware::CalibrateOptoforceSensors::Response &res) {

        printf("Received calibration service call!\n");

        for (int i = 0; i < 4; i++) _measurementCounter[i] = 0;
        for (int i = 0; i < 12; i++) _means[i] = 0;
        calibrating = true;

        while (_measurementCounter[0] < calibrationMeasurements &&
               _measurementCounter[1] < calibrationMeasurements &&
               _measurementCounter[2] < calibrationMeasurements &&
               _measurementCounter[3] < calibrationMeasurements) ros::spinOnce();

        for (int i = 0; i < 4; i++){
            _means[i*3]   /=  _measurementCounter[i];
            _means[i*3+1] /=  _measurementCounter[i];
            _means[i*3+2] /=  _measurementCounter[i];
        }

        printf("Means:");
        for (int i = 0; i < 12; i++){
            printf(" %.2f", _means[i]);
            if (i != 11) printf(",");
        }
        printf("\n");

        calibrating = false;

        for (int i = 0; i < 4; i++) _measurementCounter[i] = -1;


        return true;
    }

    void spin(){
        ros::spin();
    }

private:

    // Node
    ros::NodeHandle _nh;
    std::string _name;

    // Publishers
    std::array<ros::Publisher, 4> _pub_optoforce;

    // Subscribers
    std::array<ros::Subscriber, 4> _sub_optoforce;

    // Services
    ros::ServiceServer _ser_calibration;

    // Other
    std::array<std::vector<float>, 12> _measurements;
    std::array<float, 12> _means;
    std::array<int, 4> _measurementCounter;
    bool calibrating;

    // Config
    const int calibrationMeasurements = 1000;
    const unsigned int medianFilterSize = 5;
};

int main(int argc,char** argv){

    // Initialize ROS
    ros::init(argc,argv,"optoforceFilter");
    ros::NodeHandle nh("~");

    optoforceFilter of(nh);
    of.spin();

    return 0;
}