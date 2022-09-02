#include "../../include/motor_driver/motor_driver.h"
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <boost/thread/mutex.hpp>

int main(int argc, char **argv)
{
    // Create object pointer.
    std::shared_ptr<AMR::MotorDriver> test_node_motor = std::make_shared<AMR::MotorDriver>(argc > 1 ? argv[1] : "/dev/ttyUSB0", 115200);

    // Open serial port.
    test_node_motor->openSerialPort();

    
    test_node_motor->enableMotorDriver();
    sleep(1);

    test_node_motor->velocityControl();
    sleep(5);

    test_node_motor->stopVelocityControl();
    sleep(5);

    // Manually close serial port. Or you can alternatively let the destructor to do so.
    test_node_motor->closeSerialPort();
    return 0;
}

