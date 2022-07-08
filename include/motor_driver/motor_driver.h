#include "../serial_port_communication.h"

#ifndef _MOTOR_DRIVER_H
#define _MOTOR_DRIVER_H


namespace AMR
{
    class MotorDriver : public Communication::SerialPort
    {
    private:
        std::shared_ptr<boost::mutex> func_mutex_;

        const uint8_t example_broadcast_       = 0x00;
        const uint8_t exmaple_number_          = 0x01;
        const uint8_t example_motor_id_        = 0x01;
        const uint8_t example_function_code_   = 0x65;
        const uint8_t example_motor_drive_cmd_ = 0x63;


        const int rcv_size                  =    8;


    public:
        /**
         * @brief Constructor.
         * @param serial_port port name, e.g. "/dev/ttyUSB0".
         * @param baud_rate baud rate, e.g. 115200.
         */
        MotorDriver(const std::string serial_port, const int baud_rate);

        /**
         * @brief Destructor.
         */
        ~MotorDriver();

        /**
         * @brief Example of using serial port communication & modbus protocol to write command to motor driver.
         * @param data Communication data.
         */

        void exampleModbusCommand(uint16_t data);


        /**
         * @brief Example of using serial port communication & modbus protocol to read data from motor driver.
         * 
         */
        double exampleGetEncoder();


        /**
         * @brief Computation of crc check code.
         * @param data_vector The data vector used to compute crc16 check code.
         */
        inline uint16_t computeCRC16(std::vector<uint8_t> data_vector)
        { 
            uint16_t crc = 0xFFFF;  
            for(auto idx=0; idx<data_vector.size(); idx++)
            {
                crc ^= (uint16_t)data_vector[idx];
                for(auto i=8; i!=0; i--)
                {
                    if((crc & 0x0001) != 0)
                    {
                        crc >>= 1;
                        crc ^= 0xA001;
                    }
                    else
                    {
                        crc >>= 1;
                    }
                }
            }
            return crc;
        }
    };
} // namespace Motor

#endif