#include "modbus.h"

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H


namespace Motor
{
    class MotorDriver : public SerialModbus
    {
    private:
        std::shared_ptr<boost::mutex> p_func_mutex;

        const uint8_t broadcast_            = 0x00;
        const uint8_t motor_id_             = 0x01; // this shouldn't be const, cuz it's different for different motors.
        const uint8_t fc_master_send_cmd_   = 0x65;
        const uint8_t cmd_ISTOP_echo_       = 0x00;
        const uint8_t cmd_ISTOP_no_echo_    = 0x64;
        const uint8_t cmd_JG_echo_          = 0x0A;
        const uint8_t cmd_JG_no_echo_       = 0x6E;
        const uint8_t cmd_FREE_echo_        = 0x05;
        const uint8_t cmd_FREE_no_echo_     = 0x69;
        const uint8_t cmd_SVON_echo_        = 0x06;
        const uint8_t cmd_SVON_no_echo_     = 0x6A;
        const uint8_t cmd_SVOFF_echo_       = 0x07;
        const uint8_t cmd_SVOFF_no_echo_    = 0x6B;
        const uint8_t cmd_IMR_echo_         = 0x0B;
        const uint8_t cmd_IMR_no_echo_      = 0x6F;
        const uint8_t cmd_CS_echo_          = 0x0E;
        const uint8_t cmd_CS_no_echo_       = 0x72;
        const uint8_t cmd_CMR_echo_         = 0x0F;
        const uint8_t cmd_CMR_no_echo_      = 0x73;
        const uint8_t cmd_CMA_echo_         = 0x10;
        const uint8_t cmd_CMA_no_echo_      = 0x74;
        const uint8_t cmd_NULL_echo_        = 0x63;
        const uint8_t cmd_NULL_no_echo_     = 0x77;
        
        const int rcv_size                  =    8;


    public:
        MotorDriver(const std::string _serial_port, const int _baud_rate);
        virtual ~MotorDriver();

        // 這邊要更新成 id 也作為參數的版本
        void ISTOP(uint8_t motor_id, bool is_echo);
        void JG(uint8_t motor_id, uint16_t _cmd_rpm, bool is_echo);
        void FREE(uint8_t motor_id, bool is_echo);
        void SVON(uint8_t motor_id, bool is_echo);
        void SVOFF(uint8_t motor_id, bool is_echo);
        void IMR(uint8_t motor_id, uint16_t index, uint16_t step, bool is_echo);
        void CS(uint8_t motor_id, uint16_t index, uint16_t step, bool is_echo);
        void CMR(uint8_t motor_id, uint16_t index, uint16_t step, bool is_echo);
        void CMA(uint8_t motor_id, uint16_t index, uint16_t step, bool is_echo);
        void NULL_TO_ECHO(uint8_t motor_id, bool is_echo);


    private:
        
        const uint8_t fc_master_send_cmd_Lite_  = 0x41;
        const uint8_t cmd_ISTOP_Lite_           = 0x00;
        const uint8_t cmd_JG_Lite_              = 0x01;
        const uint8_t cmd_FREE_Lite_            = 0x05;
        const uint8_t cmd_SVON_Lite_            = 0x06;
        const uint8_t cmd_SVOFF_Lite_           = 0x07;
        const uint8_t cmd_ALM_RST_Lite_         = 0x08;
        const uint8_t cmd_BRAKE_Lite_           = 0x09;
        const uint8_t cmd_NULL_Lite_            = 0x63;
        const uint8_t echo_bit_Lite_            = 0x7F; 
        const uint8_t no_echo_bit_Lite_         = 0x00; 
        /*
        Echo data:
        1. Motor State
        2. Hall Encoder Count
        3. Motor Speed (rpm)
        4. Error Code
        5. I/O State
        6. Voltage (V)
        7. Current (A)
        */


    public:
        // 這邊要更新成 id 也作為參數的版本
        void ISTOP_Lite(uint8_t motor_id, bool is_echo);
        void JG_Lite(uint8_t motor_id, uint16_t _cmd_rpm, bool is_echo);
        void FREE_Lite(uint8_t motor_id, bool is_echo);
        void SVON_Lite(uint8_t motor_id, bool is_echo);
        void SVOFF_Lite(uint8_t motor_id, bool is_echo);
        void ALM_RST_Lite(uint8_t motor_id, bool is_echo);
        void BRAKE_Lite(uint8_t motor_id, bool is_echo);
        void NULL_Lite(uint8_t motor_id, bool is_echo);

    public:
        double get_encoder();
        double get_current();
        double get_voltage();
        bool FindSteeringHome(uint8_t motor_id);


    public:
        // For driving Wheels.
        void Multi_ISTOP_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_JG_Lite(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> data_list, bool is_echo);

        void Multi_FREE_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_SVON_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_SVOFF_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_ALM_RST_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_BRAKE_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_NULL_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);


    public:
        // For steering Wheels.
        void Multi_ISTOP(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_JG(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> data_list, bool is_echo);

        void Multi_FREE(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_SVON(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_SVOFF(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_NULL(uint8_t num, std::vector<uint8_t> id_list, bool is_echo);

        void Multi_IMR(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> index_list, std::vector<uint16_t> step_list, bool is_echo);

        void Multi_CS(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> index_list, std::vector<uint16_t> step_list, bool is_echo);

        void Multi_CMR(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> index_list, std::vector<uint16_t> step_list, bool is_echo);

        void Multi_CMA(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> index_list, std::vector<uint16_t> step_list, bool is_echo);

        

    };
} // namespace Motor

#endif