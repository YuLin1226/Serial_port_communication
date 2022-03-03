// #include "trumman/trumman.h"
// #include <ros/ros.h>
// #include <std_msgs/Float64.h>
#include "modbus.h"
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <boost/thread/mutex.hpp>

// std::shared_ptr<Motor::SerialModbus> p_motor{ new Motor::SerialModbus{"/dev/ttyUSB0",115200} };
std::shared_ptr<Motor::SerialModbus> p_motor{ new Motor::SerialModbus{"/dev/pts/6",115200} };


int main(int argc, char **argv)
{
    // 開啟通訊
    p_motor->open();

    uint8_t _ID = 0x01;
    uint8_t _FC = 0x03;
    uint16_t _ADDR = 0x0001;
    uint16_t _DATA = 0x0001;
    int RESPONSE_DELAY_US = 1500;
    int expected_bytes = 10;

    while(1){
        p_motor->writeOnly(_ID, _FC, _ADDR, _DATA);
        sleep(3);
    }







    // 關閉通訊
    p_motor->close();
    return 0;
}

