#include "modbus.h"




namespace Motor
{
    class MotorDriver : public SerialModbus
    {
    private:
        std::shared_ptr<boost::mutex> p_func_mutex;

        const uint8_t Broadcast         =  0;
        const uint8_t MOTOR_ID          =  1;
        const uint8_t FC_SetMotorSpeed  = 41;
        const uint8_t CMD_JG            =  1;


    public:
        MotorDriver(const std::string _serial_port, const int _baud_rate);
        virtual ~MotorDriver();

        void SetMotorSpeed_RPM(uint16_t _cmd_rpm);



    



    };
} // namespace Motor
