// #include "trumman/trumman.h"
// #include <ros/ros.h>
// #include <std_msgs/Float64.h>
#include "include/modbus.h"
#include "include/motor_driver.h"
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <boost/thread/mutex.hpp>



int main(int argc, char **argv)
{
    // std::shared_ptr<Motor::SerialModbus> p_motor = std::make_shared<Motor::SerialModbus>(argc > 1 ? argv[1] : "/dev/pts/8", 115200);

    std::shared_ptr<Motor::MotorDriver> p_motor = std::make_shared<Motor::MotorDriver>(argc > 1 ? argv[1] : "/dev/ttyUSB0", 115200);

    // 開啟通訊
    p_motor->open();

    if(true){
        try
        {
            p_motor -> JG(300, false);
            sleep(5);
            // p_motor -> SetMotorSpeed_RPM(0, false);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
    }

    // 關閉通訊
    // p_motor->close();
    return 0;
}

