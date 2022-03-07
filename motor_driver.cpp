#include "include/motor_driver.h"

namespace Motor{

    union unionType
    {
        uint8_t _data_byte[2];
        uint16_t _data;
    };


    MotorDriver::MotorDriver(   const std::string _serial_port = "/dev/ttyUSB0", const int _baud_rate = 115200) : SerialModbus(_serial_port, _baud_rate){
        p_func_mutex = std::shared_ptr<boost::mutex>{new boost::mutex};
    }




    void MotorDriver::SetMotorSpeed_RPM(uint16_t _cmd_rpm){


        /* ==============================================================================
            *     0 < _cmd_rpm < 4000  :  CW
            * -4000 < _cmd_rpm <    0  : CCW
            * 
            * Example: Set +300 rpm
            * 00 41 01 01 01 01 2C 00 00 23 E8
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t _num = 1;
        unionType _cmd_rpm_data, _echo_bit;
        _cmd_rpm_data._data = _cmd_rpm;
        _echo_bit._data = 0;

        p_data.clear();
        p_data.push_back(this->Broadcast);
        p_data.push_back(this->FC_SetMotorSpeed);
        p_data.push_back(_num);
        p_data.push_back(this->MOTOR_ID);
        p_data.push_back(this->CMD_JG);
        p_data.push_back(_cmd_rpm_data._data_byte[0]);
        p_data.push_back(_cmd_rpm_data._data_byte[1]);
        p_data.push_back(_echo_bit._data_byte[0]);
        p_data.push_back(_echo_bit._data_byte[1]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc >> 8);
        p_data.push_back(crc);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout << std::endl << "========== Data Print Check ==========" << std::endl;
        for(auto i=0;i<p_char.size();i++){
            printf("data[%i] = ", i);
            std::bitset<8> x(p_char[i]);
            std::cout << x << std::endl;
        }
    }





}