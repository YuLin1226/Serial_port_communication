#include "modbus.h"

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H


namespace Motor
{
    class MotorDriver : public SerialModbus
    {
    private:
        std::shared_ptr<boost::mutex> p_func_mutex;

        const uint8_t Broadcast         = 0x00;
        const uint8_t MOTOR_ID          = 0x01;
        const uint8_t FC_MasterSendCMD  = 0x65;
        const uint8_t CMD_ISTOP_Echo    = 0x00;
        const uint8_t CMD_ISTOP_No_Echo = 0x64;
        const uint8_t CMD_JG_Echo       = 0x0A;
        const uint8_t CMD_JG_No_Echo    = 0x6E;
        const uint8_t CMD_FREE_Echo     = 0x05;
        const uint8_t CMD_FREE_No_Echo  = 0x69;
        const uint8_t CMD_SVON_Echo     = 0x06;
        const uint8_t CMD_SVON_No_Echo  = 0x6A;


    public:
        MotorDriver(const std::string _serial_port, const int _baud_rate);
        virtual ~MotorDriver();

        void JG(uint16_t _cmd_rpm, bool is_echo);
        void ISTOP(bool is_echo);
        void FREE(bool is_echo);



    



    };
} // namespace Motor

#endif