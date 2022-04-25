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

    // if(true){
    //     try
    //     {
    //         p_motor -> JG(300, false);
    //         // p_motor -> NULL_TO_ECHO(true);
    //         sleep(5);
    //         // p_motor -> SetMotorSpeed_RPM(0, false);
    //     }
    //     catch(const std::exception& e)
    //     {
    //         std::cerr << e.what() << '\n';
    //     }
        
    // }
    
    p_motor -> CS(0, 0, false); // init encoder count
    sleep(1);

    p_motor -> JG(300, false);
    // p_motor -> CMR(10, 5000, false);
    sleep(1);
    for(auto i=0; i<10; i++){
        try
        {
            // p_motor -> NULL_TO_ECHO(true);
            double enc_data_ = p_motor->get_encoder();
            std::cout << std::dec << "No. " << i+1 << " loop, data: " << enc_data_ << "\n" << std::endl;
            sleep(1);
            // // p_motor -> SetMotorSpeed_RPM(0, false);
            
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

