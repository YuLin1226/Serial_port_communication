#include "../../include/motor_driver/motor_driver.h"
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <boost/thread/mutex.hpp>

int main(int argc, char **argv)
{
    std::string port1 = "/dev/ttyS1";
    std::string port2 = "/dev/ttyS0";
    int baud_rate = 9600;
    // Create object pointer.
    // std::shared_ptr<AMR::MotorDriver> test_node_motor = std::make_shared<AMR::MotorDriver>(argc > 1 ? argv[1] : "/dev/ttyUSB0", 115200);
    std::shared_ptr<AMR::MotorDriver> test_node_motor1 = std::make_shared<AMR::MotorDriver>(port1, baud_rate);
    std::shared_ptr<AMR::MotorDriver> test_node_motor2 = std::make_shared<AMR::MotorDriver>(port2, baud_rate);

    uint8_t id1 = 0x01;
    uint8_t id2 = 0x02;

    // // Open serial port.
    test_node_motor1->openSerialPort();
    test_node_motor2->openSerialPort();

    for(auto i=0; i<10; i++)
    {
        // test_node_motor1->enableMotorDriver(id1);
        // test_node_motor2->enableMotorDriver(id2);
        test_node_motor1->readEncoder(id1);
        test_node_motor2->readEncoder(id2);
    }
    sleep(2);
    
    // test_node_motor1->printReadBuf();
    // test_node_motor2->printReadBuf();

    // Manually close serial port. Or you can alternatively let the destructor to do so.
    test_node_motor1->closeSerialPort();
    test_node_motor2->closeSerialPort();
    
    return 0;
}

