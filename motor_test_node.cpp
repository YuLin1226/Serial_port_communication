// #include "trumman/trumman.h"
// #include <ros/ros.h>
// #include <std_msgs/Float64.h>
#include "modbus.h"
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <boost/thread/mutex.hpp>



int main(int argc, char **argv)
{
    // std::shared_ptr<Motor::SerialModbus> p_motor{ new Motor::SerialModbus{"/dev/ttyUSB0",115200} };
    std::shared_ptr<Motor::SerialModbus> p_motor{ new Motor::SerialModbus{argc > 1 ? argv[1] : "/dev/pts/8", 115200} };

    // 開啟通訊
    p_motor->open();

    uint8_t _ID = 0x01;
    uint8_t _FC = 0x03;
    uint16_t _ADDR = 0x0001;
    uint16_t _DATA = 0x0001;
    int RESPONSE_DELAY_US = 1500;
    int expected_bytes = 6;

    while(1){
        try
        {
            std::vector<char> resp = p_motor->read_and_write(_ID, _FC, _ADDR, _DATA, expected_bytes);
            std::cout << std::endl << "========== Data Received. Binary Below. ==========" << std::endl;
            for(auto i=0;i<resp.size();i++){
                printf("resp[%i] = ", i);
                std::bitset<8> x(resp[i]);
                std::cout << x << std::endl;
            }
            
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        sleep(3);
    }

    // 關閉通訊
    p_motor->close();
    return 0;
}

