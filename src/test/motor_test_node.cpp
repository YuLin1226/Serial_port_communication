#include "../../include/motor_driver/motor_driver.h"
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <boost/thread/mutex.hpp>

int main(int argc, char **argv)
{
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 9600;
    // Create object pointer.
    // std::shared_ptr<AMR::MotorDriver> test_node_motor = std::make_shared<AMR::MotorDriver>(argc > 1 ? argv[1] : "/dev/ttyUSB0", 115200);
    std::shared_ptr<AMR::MotorDriver> test_node_motor = std::make_shared<AMR::MotorDriver>(port, baud_rate);


    uint8_t id = 0x01;


    // // Open serial port.
    test_node_motor->openSerialPort();

    
    test_node_motor->enableMotorDriver(id);
    sleep(1);

    // test_node_motor->velocityControl(id);
    // sleep(5);

    // test_node_motor->stopVelocityControl(id);
    // sleep(5);

    // test_node_motor->positionControl(id);

    // Manually close serial port. Or you can alternatively let the destructor to do so.
    test_node_motor->closeSerialPort();
    return 0;
}

