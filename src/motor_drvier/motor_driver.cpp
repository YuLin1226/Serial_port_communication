// #include "../../include/serial_port_communication.h"
#include "../../include/motor_driver/motor_driver.h"
#include <cmath>

namespace AMR
{

    union convertUint8AndUint16
    {
        // [0] Low 8 bits
        // [1] High 8 bits 
        uint8_t data8[2]; 
        uint16_t data16;
    };


    MotorDriver::MotorDriver(const std::string serial_port, const int baud_rate):
    Communication::SerialPort(serial_port, baud_rate)
    {
        func_mutex_ = std::shared_ptr<boost::mutex>{new boost::mutex};
    }

    MotorDriver::~MotorDriver()
    {

    }

    void MotorDriver::exampleModbusCommand(uint16_t data)
    {
        std::vector<uint8_t> data_uint8_vector;

        convertUint8AndUint16 data_converted;
        data_converted.data16 = data;
        
        data_uint8_vector.clear();

        data_uint8_vector.push_back(example_broadcast_);
        data_uint8_vector.push_back(example_function_code_);
        data_uint8_vector.push_back(exmaple_number_);
        data_uint8_vector.push_back(example_motor_id_);
        data_uint8_vector.push_back(example_motor_drive_cmd_);
        data_uint8_vector.push_back(data_converted.data8[1]);
        data_uint8_vector.push_back(data_converted.data8[0]);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[0]);
        
        std::vector<char> data_char_vector(data_uint8_vector.begin(), data_uint8_vector.end());
        writeDataThroughSerialPort(data_char_vector);

    }

    double MotorDriver::exampleGetEncoder()
    {

        // First: write command to ask motor driver to send back data.
        // This part takes "exampleModbusCommand()" to demonstrate "asking command".
        uint16_t ask_motor_driver_to_send_back_data = 0x0000;
        exampleModbusCommand(ask_motor_driver_to_send_back_data);

        // Then: async read data.
        std::vector<char> data_received;
        {
            usleep(Communication::RESPONSE_DELAY_US);
            try
            {
                const int expected_bytes = 8;
                data_received = asyncReadDataThroughSerialPort(expected_bytes);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        // Finally: process received data. This should follow motor driver's manual.
        double encoder_data;
        convertUint8AndUint16 encoder_turn, encoder_step;
        encoder_turn.data8[1] = data_received.at(2);
        encoder_turn.data8[0] = data_received.at(3);
        encoder_step.data8[1] = data_received.at(4);
        encoder_step.data8[0] = data_received.at(5);
        encoder_data = (encoder_turn.data16 + encoder_step.data16/10000.0)*2.0*M_PI;
        return encoder_data;
    }


    void MotorDriver::velocityControl()
    {
        std::vector<uint8_t> data_uint8_vector;

        data_uint8_vector.clear();

        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x10);
        data_uint8_vector.push_back(0x44);
        data_uint8_vector.push_back(0x20);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x04);
        // following 4 push_back are vel data.
        data_uint8_vector.push_back(0x03);
        data_uint8_vector.push_back(0xE8);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[1]);
        
        std::vector<char> data_char_vector(data_uint8_vector.begin(), data_uint8_vector.end());
        writeDataThroughSerialPort(data_char_vector);
    }

    void MotorDriver::enableMotorDriver()
    {
        std::vector<uint8_t> data_uint8_vector;

        data_uint8_vector.clear();

        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x10);
        data_uint8_vector.push_back(0x46);
        data_uint8_vector.push_back(0x57);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x01);
        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x01);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[1]);
        
        std::vector<char> data_char_vector(data_uint8_vector.begin(), data_uint8_vector.end());
        std::cout << "Send Command: ";
        for (auto i = 0; i < data_char_vector.size(); i++)
        {
            std::cout << std::hex << (int)data_char_vector[i] << " ";
        }
        std::cout << std::endl;

        
        writeDataThroughSerialPort(data_char_vector);

        std::vector<char> data_received;
        {
            usleep(Communication::RESPONSE_DELAY_US);
            try
            {
                const int expected_bytes = 4;
                data_received = asyncReadDataThroughSerialPort(expected_bytes);
                std::cout << "Received Command: ";
                for (auto i = 0; i < data_received.size(); i++)
                {
                    std::cout << std::hex << (int)data_received[i] << " ";
                }
                std::cout << std::endl;
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }

    void MotorDriver::stopVelocityControl()
    {
        std::vector<uint8_t> data_uint8_vector;

        data_uint8_vector.clear();

        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x10);
        data_uint8_vector.push_back(0x44);
        data_uint8_vector.push_back(0x20);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x04);
        // following 4 push_back are vel data.
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[1]);
        
        std::vector<char> data_char_vector(data_uint8_vector.begin(), data_uint8_vector.end());
        writeDataThroughSerialPort(data_char_vector);
    }



}