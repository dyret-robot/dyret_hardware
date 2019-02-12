#ifndef DYRET_HARDWARE_DYNAMIXEL_WRAPPER_H
#define DYRET_HARDWARE_DYNAMIXEL_WRAPPER_H

#endif //DYRET_HARDWARE_DYNAMIXEL_WRAPPER_H

namespace dynamixel_wrapper {

// This initializes the port and all servos
bool initializeServos();

// Closes the serial port
void closeServoConnection();

// This sends commands to the servos to the angles specified in
// anglesInRad, starting at servo id 0
void setServoAngles(std::vector<float> anglesInRad);

// Set servo speeds as floats between 0 and 1
bool setServoSpeeds(std::vector<int> servoIds, std::vector<float> servoSpeeds);

// Set servo PIDS, three parameters per servo ID
bool setServoPIDs(std::vector<int> servoIds, std::vector<float> servoPIDs);

// Get servo angles in radians from the servos stated in servoIds vector
std::vector<float> getServoAngles(std::vector<int> servoIds);

// Get servo voltage in volt from the servos
std::vector<float> getServoVoltages();

// Get servo current in ampere from the servos
std::vector<float> getServoCurrents();

// Get servo temperature in celsius from the servos
std::vector<float> getServoTemperatures();

// Get servo velocities in rpm from the servos
std::vector<float> getServoVelocities();

bool enableTorque(uint8_t givenServoId);

bool disableTorque(uint8_t givenServoId);

}