#include "modbus.h"

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H


namespace Motor
{
    class MotorDriver : public SerialModbus
    {
    private:
        std::shared_ptr<boost::mutex> p_func_mutex;

        const uint8_t Broadcast         =  0x00;
        const uint8_t MOTOR_ID          =  0x01;
        const uint8_t FC_SetMotorSpeed  =  0x41;
        const uint8_t CMD_JG            =  0x01;


    public:
        MotorDriver(const std::string _serial_port, const int _baud_rate);
        virtual ~MotorDriver();

        void SetMotorSpeed_RPM(uint16_t _cmd_rpm);



    



    };
} // namespace Motor

#endif