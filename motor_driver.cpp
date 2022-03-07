#include "include/motor_driver.h"

namespace Motor{

    MotorDriver::MotorDriver(   const std::string _serial_port = "/dev/ttyUSB0", const int _baud_rate = 115200) : SerialModbus(_serial_port, _baud_rate){
        p_func_mutex = std::shared_ptr<boost::mutex>{new boost::mutex};
    }



}