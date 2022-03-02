// #include "trumman/trumman.h"
// #include <ros/ros.h>
// #include <std_msgs/Float64.h>
#include "modbus.h"
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <boost/thread/mutex.hpp>

std::shared_ptr<Motor::SerialModbus> p_motor{ new Motor::SerialModbus{"/dev/ttyUSB0",115200} };


int main(int argc, char **argv)
{
    return 0;
}
