#ifndef DYRET_HARDWARE_DYNAMIXEL_WRAPPER_H
#define DYRET_HARDWARE_DYNAMIXEL_WRAPPER_H

#endif //DYRET_HARDWARE_DYNAMIXEL_WRAPPER_H

namespace dynamixel_wrapper {

// This initializes the port and all servos
bool initializeServos(std::vector<int> givenServoIds);

// Closes the serial port
void closeServoConnection();

// This sends commands to the servos to the angles specified in
// anglesInRad, starting at servo id 0
void setServoAngles(std::vector<float> anglesInRad);

// Set servo speeds as floats between 0 and 1
bool setServoSpeeds(std::vector<int> servoIds, std::vector<float> servoSpeeds);

// Get servo angles in radians from the servos stated in servoIds vector
std::vector<float> getServoAngles(std::vector<int> servoIds);

// Get servo voltage in volt from the servos stated in servoIds vector
std::vector<float> getServoVoltages();

// Get servo current in ampere from the servos stated in servoIds vector
std::vector<float> getServoCurrents();

// Get servo temperature in celsius from the servos stated in servoIds vector
std::vector<int> getServoTemperatures();

}