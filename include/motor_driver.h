#include "modbus.h"




namespace Motor
{
    class MotorDriver : public SerialModbus
    {
    private:
        std::shared_ptr<boost::mutex> p_func_mutex;
    public:
        MotorDriver(const std::string _serial_port, const int _baud_rate);
        virtual ~MotorDriver();
    };
} // namespace Motor
