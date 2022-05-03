#include "include/motor_driver.h"
#include <cmath>

namespace Motor{

    union unionType
    {
        uint8_t data_byte[2]; // [1] 高8位,上位 ； [0] 低8位,下位
        uint16_t data;
    };


    MotorDriver::MotorDriver(const std::string _serial_port = "/dev/ttyUSB0", const int _baud_rate = 115200) : SerialModbus(_serial_port, _baud_rate){
        p_func_mutex = std::shared_ptr<boost::mutex>{new boost::mutex};
    }
    MotorDriver::~MotorDriver(){
        // STOP Motor
        this->ISTOP(motor_id_, false);
        // this->FREE(false);
    }


    // Motor Drive
    void MotorDriver::ISTOP(uint8_t motor_id = 0x01, bool is_echo = false){

        /* ==============================================================================
            * Example: No Echo
            * 0 65 1 1 64 0 0 0 0 ac 3e
            * 
            * If echo, receive 8 bytes data per message.
        ============================================================================== */
        
        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t cmd = is_echo ? this->cmd_ISTOP_echo_ : this->cmd_ISTOP_no_echo_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = 0x00;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(cmd);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send ISTOP Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::JG(uint8_t motor_id = 0x01, uint16_t cmd_rpm = 0x0000, bool is_echo = false){

        /* ==============================================================================
            *     0 < _cmd_rpm < 4000  :  CW
            * -4000 < _cmd_rpm <    0  : CCW
            * 
            * Example: Set +300 rpm / No Echo
            * 0 65 1 1 6e 0 0 1 2c 34 72
            * 
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t cmd = is_echo ? this->cmd_JG_echo_ : this->cmd_JG_no_echo_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = cmd_rpm;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(cmd);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send JG Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::FREE(uint8_t motor_id = 0x01, bool is_echo = false){

        /* ==============================================================================
            * Example: No Echo
            * 0 65 1 1 69 0 0 0 0 81 ff
            * 
            * If echo, receive 8 bytes data per message.
        ============================================================================== */
        
        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t cmd = is_echo ? this->cmd_FREE_echo_ : this->cmd_FREE_no_echo_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = 0x00;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(cmd);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send FREE Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::SVON(uint8_t motor_id = 0x01, bool is_echo = false){

        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t cmd = is_echo ? this->cmd_SVON_echo_ : this->cmd_SVON_no_echo_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = 0x00;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(cmd);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send SVON Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::IMR(uint8_t motor_id = 0x01, uint16_t index = 0x0000, uint16_t step = 0x0000, bool is_echo = false){
        
        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t cmd = is_echo ? this->cmd_IMR_echo_ : this->cmd_IMR_no_echo_;
        unionType data_1, data_2;
        data_1.data = index;
        data_2.data = step;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(cmd);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send IMR Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::CS(uint8_t motor_id = 0x01, uint16_t index = 0x0000, uint16_t step = 0x0000, bool is_echo = false){
        
        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t cmd = is_echo ? this->cmd_CS_echo_ : this->cmd_CS_no_echo_;
        unionType data_1, data_2;
        data_1.data = index;
        data_2.data = step;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(cmd);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send CS Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::CMR(uint8_t motor_id = 0x01, uint16_t index = 0x0000, uint16_t step = 0x0000, bool is_echo = false){
        
        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t cmd = is_echo ? this->cmd_CMR_echo_ : this->cmd_CMR_no_echo_;
        unionType data_1, data_2;
        data_1.data = index;
        data_2.data = step;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(cmd);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send CMR Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::CMA(uint8_t motor_id = 0x01, uint16_t index = 0x0000, uint16_t step = 0x0000, bool is_echo = false){
        
        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t cmd = is_echo ? this->cmd_CMA_echo_ : this->cmd_CMA_no_echo_;
        unionType data_1, data_2;
        data_1.data = index;
        data_2.data = step;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(cmd);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send CMA Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::NULL_TO_ECHO(uint8_t motor_id = 0x01, bool is_echo = true){
        
        /* ==============================================================================
            * If echo, receive 8 bytes data per message.
        ============================================================================== */

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t cmd = is_echo ? this->cmd_NULL_echo_ : this->cmd_NULL_no_echo_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = 0x00;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(cmd);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send NULL Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

        // if(is_echo){

        //     std::vector<char> rcv_char;
            
        //     {
        //         usleep(RESPONSE_DELAY_US);
        //         try
        //         {
        //             rcv_char = asyncRead(this->rcv_size);
        //         }
        //         catch(const std::exception& e)
        //         {
        //             std::cerr << e.what() << '\n';
        //         }
        //         std::cout <<  "Received Data: ";
        //         for(auto i=0;i<rcv_char.size();i++){            
        //             uint8_t a = rcv_char[i];
        //             std::cout << std::hex << +a << " ";
        //         }
        //         std::cout << std::endl;
                
        //     }

        // }
    }

    // Motor Drive Lite
    void MotorDriver::ISTOP_Lite(uint8_t motor_id = 0x01, bool is_echo = false){

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t echo_bit = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = echo_bit;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(this->cmd_ISTOP_Lite_);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send ISTOP_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::JG_Lite(uint8_t motor_id = 0x01, uint16_t cmd_rpm = 0x0000, bool is_echo = false){

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t echo_bit = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
        unionType data_1, data_2;
        data_1.data = cmd_rpm;
        data_2.data = echo_bit;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(this->cmd_JG_Lite_);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send JG_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::FREE_Lite(uint8_t motor_id = 0x01, bool is_echo = false){

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t echo_bit = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = echo_bit;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(this->cmd_FREE_Lite_);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send FREE_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::SVON_Lite(uint8_t motor_id = 0x01, bool is_echo = false){

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t echo_bit = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = echo_bit;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(this->cmd_SVON_Lite_);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send SVON_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::SVOFF_Lite(uint8_t motor_id = 0x01, bool is_echo = false){

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t echo_bit = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = echo_bit;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(this->cmd_SVOFF_Lite_);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send SVOFF_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::ALM_RST_Lite(uint8_t motor_id = 0x01, bool is_echo = false){

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t echo_bit = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = echo_bit;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(this->cmd_ALM_RST_Lite_);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send ALM_RST_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::BRAKE_Lite(uint8_t motor_id = 0x01, bool is_echo = false){

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t echo_bit = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = echo_bit;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(this->cmd_BRAKE_Lite_);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send BRAKE_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::NULL_Lite(uint8_t motor_id = 0x01, bool is_echo = false){

        std::vector<uint8_t> p_data;
        uint8_t num = 0x01;
        uint8_t echo_bit = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
        unionType data_1, data_2;
        data_1.data = 0x00;
        data_2.data = echo_bit;

        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);
        p_data.push_back(motor_id);
        p_data.push_back(this->cmd_NULL_Lite_);
        p_data.push_back(data_1.data_byte[1]);
        p_data.push_back(data_1.data_byte[0]);
        p_data.push_back(data_2.data_byte[1]);
        p_data.push_back(data_2.data_byte[0]);
        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send NULL_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    double MotorDriver::get_encoder(){
        const int expected_bytes = 8;

        this->NULL_TO_ECHO(0x01, true);        
        std::vector<char> response;
        {
            usleep(RESPONSE_DELAY_US);
            try
            {
                response = asyncRead(expected_bytes);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        double encoder_data;
        union unionType encoder_index, encoder_step;
        encoder_index.data_byte[1]    = response.at(2);
        encoder_index.data_byte[0]    = response.at(3);
        encoder_step.data_byte[1]     = response.at(4);
        encoder_step.data_byte[0]     = response.at(5);
        // std:: cout << "=========\n";
        // std:: cout << std::dec << encoder_index_._data << " " << encoder_step_._data <<"\n";
        encoder_data = (encoder_index.data + encoder_step.data/10000.0)*2.0*M_PI;
        return encoder_data;
    }

    double MotorDriver::get_current(){
        const int expected_bytes = 20;
        this->NULL_Lite(0x01, true);
        std::vector<char> response;
        {
            usleep(RESPONSE_DELAY_US);
            try
            {
                response = asyncRead(expected_bytes);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        double current_data;

        union unionType current_bit;
        current_bit.data_byte[1]    = response.at(16);
        current_bit.data_byte[0]    = response.at(17);
        // std:: cout << "=========\n";
        // std:: cout << std::dec << encoder_index_._data << " " << encoder_step_._data <<"\n";
        current_data = current_bit.data*0.01;
        return current_data;
    }

    double MotorDriver::get_voltage(){
        const int expected_bytes = 20;
        this->NULL_Lite(0x01, true);
        std::vector<char> response;
        {
            usleep(RESPONSE_DELAY_US);
            try
            {
                response = asyncRead(expected_bytes);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        double voltage_data;

        union unionType voltage_bit;
        voltage_bit.data_byte[1]    = response.at(14);
        voltage_bit.data_byte[0]    = response.at(15);
        // std:: cout << "=========\n";
        // std:: cout << std::dec << encoder_index_._data << " " << encoder_step_._data <<"\n";
        voltage_data = voltage_bit.data*0.01;
        return voltage_data;
    }

    bool MotorDriver::FindSteeringHome(uint8_t motor_id){
        /*
            尋 Home 應該要前、後輪分開做。
            * 1. 低速左轉：JG
            * 2. while 讀取IO-X1：NULL
            * 3. 低速右轉：JG
            * 4. while 讀取IO-X2：NULL
            * 5. 左轉固定角度：CMR
            * 6. 歸零：CS
        */

        const int expected_bytes = 20;
        int rpm = 300;
        this->JG(motor_id, -rpm, false);
        usleep(10000);
        bool io_x1_state = false;
        while (!io_x1_state)
        {
            try
            {
                // get X1 IO & update to io_x1_state.
                
                this->NULL_Lite(motor_id, true);
                std::vector<char> response;
                {
                    usleep(RESPONSE_DELAY_US);
                    try
                    {
                        response = asyncRead(expected_bytes);
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << '\n';
                    }
                }

                union unionType io_state;
                // io_state._data_byte[1] = response.at(12);
                io_state.data_byte[0] = response.at(13);
                io_x1_state = (io_state.data_byte[0]>>1)&1;
                usleep(10000);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        
        this->JG(motor_id, rpm, false);
        usleep(10000);
        bool io_x2_state = false;
        while (!io_x2_state)
        {
            try
            {
                this->NULL_Lite(motor_id, true);
                std::vector<char> response;
                {
                    usleep(RESPONSE_DELAY_US);
                    try
                    {
                        response = asyncRead(expected_bytes);
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << '\n';
                    }
                }

                union unionType io_state;
                // io_state._data_byte[1] = response.at(12);
                io_state.data_byte[0] = response.at(13);
                io_x2_state = (io_state.data_byte[0])&1;
                usleep(10000);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        this->JG(motor_id, 0, false);
        usleep(10000);
        this->CMR(motor_id, -20, 0, false);
        usleep(10000);
        this->CS(motor_id, 0, 0, false);
        usleep(10000);

        // 我覺得可以加入一個計時器，時間內沒有完成校正，就直接報錯。

        return true;
    }

    std::vector<double> MotorDriver::get_motors_data(std::vector<uint8_t> drive_motors_id_list, std::vector<uint8_t> steer_motors_id_list){
        /*
            * 先發送 Multi_NULL ，收轉向（位置）資料。
        */
        const int multi_null_expected_bytes = 8*steer_motors_id_list.size();
        this->Multi_NULL(steer_motors_id_list.size(), steer_motors_id_list, true);
        std::vector<char> multi_null_response;
        {
            usleep(RESPONSE_DELAY_US);
            try
            {
                multi_null_response = asyncRead(multi_null_expected_bytes);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        // 轉向輪位置
        double steer_index_1, steer_index_2;
        union unionType steer_index_bit_1;
        steer_index_bit_1.data_byte[1]    = multi_null_response.at(14);
        steer_index_bit_1.data_byte[0]    = multi_null_response.at(15);
        steer_index_1 = steer_index_bit_1.data;
        
        union unionType steer_index_bit_2;
        steer_index_bit_2.data_byte[1]    = multi_null_response.at(14);
        steer_index_bit_2.data_byte[0]    = multi_null_response.at(15);
        steer_index_2 = steer_index_bit_2.data;

        double steer_step_1, steer_step_2;
        union unionType steer_step_bit_1;
        steer_step_bit_1.data_byte[1]    = multi_null_response.at(14);
        steer_step_bit_1.data_byte[0]    = multi_null_response.at(15);
        steer_step_1 = steer_step_bit_1.data;
        
        union unionType steer_step_bit_2;
        steer_step_bit_2.data_byte[1]    = multi_null_response.at(14);
        steer_step_bit_2.data_byte[0]    = multi_null_response.at(15);
        steer_step_2 = steer_step_bit_2.data;

        /*
            * 再發送 Multi_NULL_Lite，收行走（位置、速度、電壓、電流）、轉向（__、__、電壓、電流）資料。
        */ 
        const int multi_null_lite_expected_bytes = 20*(drive_motors_id_list.size()+steer_motors_id_list.size());
        this->Multi_NULL_Lite(steer_motors_id_list.size(), steer_motors_id_list, true);
        std::vector<char> multi_null_lite_response;
        {
            usleep(RESPONSE_DELAY_US);
            try
            {
                multi_null_lite_response = asyncRead(multi_null_lite_expected_bytes);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        //行走輪位置
        double drive_position_1, drive_position_2;
        union unionType drive_position_bit_1;
        drive_position_bit_1.data_byte[1]    = multi_null_lite_response.at(14);
        drive_position_bit_1.data_byte[0]    = multi_null_lite_response.at(15);
        drive_position_1 = drive_position_bit_1.data;
        
        union unionType drive_position_bit_2;
        drive_position_bit_2.data_byte[1]    = multi_null_lite_response.at(14);
        drive_position_bit_2.data_byte[0]    = multi_null_lite_response.at(15);
        drive_position_2 = drive_position_bit_2.data;

        //行走輪速度
        double drive_velocity_1, drive_velocity_2;
        union unionType drive_velocity_bit_1;
        drive_velocity_bit_1.data_byte[1]    = multi_null_lite_response.at(14);
        drive_velocity_bit_1.data_byte[0]    = multi_null_lite_response.at(15);
        drive_velocity_1 = drive_velocity_bit_1.data;
        
        union unionType drive_velocity_bit_2;
        drive_velocity_bit_2.data_byte[1]    = multi_null_lite_response.at(14);
        drive_velocity_bit_2.data_byte[0]    = multi_null_lite_response.at(15);
        drive_velocity_2 = drive_velocity_bit_2.data;

        // 四輪電壓
        double drive_voltage_1, drive_voltage_2;
        union unionType drive_voltage_bit_1;
        drive_voltage_bit_1.data_byte[1]    = multi_null_lite_response.at(14);
        drive_voltage_bit_1.data_byte[0]    = multi_null_lite_response.at(15);
        drive_voltage_1 = drive_voltage_bit_1.data*0.01;
        
        union unionType drive_voltage_bit_2;
        drive_voltage_bit_2.data_byte[1]    = multi_null_lite_response.at(14);
        drive_voltage_bit_2.data_byte[0]    = multi_null_lite_response.at(15);
        drive_voltage_2 = drive_voltage_bit_2.data*0.01;

        double steer_voltage_1, steer_voltage_2;
        union unionType steer_voltage_bit_1;
        steer_voltage_bit_1.data_byte[1]    = multi_null_lite_response.at(14);
        steer_voltage_bit_1.data_byte[0]    = multi_null_lite_response.at(15);
        steer_voltage_1 = steer_voltage_bit_1.data*0.01;
        
        union unionType steer_voltage_bit_2;
        steer_voltage_bit_2.data_byte[1]    = multi_null_lite_response.at(14);
        steer_voltage_bit_2.data_byte[0]    = multi_null_lite_response.at(15);
        steer_voltage_2 = steer_voltage_bit_2.data*0.01;

        // 四輪電流
        double drive_current_1, drive_current_2;
        union unionType drive_current_bit_1;
        drive_current_bit_1.data_byte[1]    = multi_null_lite_response.at(14);
        drive_current_bit_1.data_byte[0]    = multi_null_lite_response.at(15);
        drive_current_1 = drive_current_bit_1.data*0.01;
        
        union unionType drive_current_bit_2;
        drive_current_bit_2.data_byte[1]    = multi_null_lite_response.at(14);
        drive_current_bit_2.data_byte[0]    = multi_null_lite_response.at(15);
        drive_current_2 = drive_current_bit_2.data*0.01;

        double steer_current_1, steer_current_2;
        union unionType steer_current_bit_1;
        steer_current_bit_1.data_byte[1]    = multi_null_lite_response.at(14);
        steer_current_bit_1.data_byte[0]    = multi_null_lite_response.at(15);
        steer_current_1 = steer_current_bit_1.data*0.01;
        
        union unionType steer_current_bit_2;
        steer_current_bit_2.data_byte[1]    = multi_null_lite_response.at(14);
        steer_current_bit_2.data_byte[0]    = multi_null_lite_response.at(15);
        steer_current_2 = steer_current_bit_2.data*0.01;

        /*
            * 照順序（前行>後行>前轉>後轉）堆疊資料（行走-pos, vel, volt, cur）、（轉向-idx, stp, volt, cur）到向量中，輸出。
        */ 
       std::vector<double> motors_data;
       motors_data.push_back(drive_position_1);
       motors_data.push_back(drive_velocity_1);
       motors_data.push_back(drive_voltage_1);
       motors_data.push_back(drive_current_1);

       motors_data.push_back(drive_position_2);
       motors_data.push_back(drive_velocity_2);
       motors_data.push_back(drive_voltage_2);
       motors_data.push_back(drive_current_2);

       motors_data.push_back(steer_index_1);
       motors_data.push_back(steer_step_1);
       motors_data.push_back(steer_voltage_1);
       motors_data.push_back(steer_current_1);

       motors_data.push_back(steer_index_2);
       motors_data.push_back(steer_step_2);
       motors_data.push_back(steer_voltage_2);
       motors_data.push_back(steer_current_2);

       return motors_data;
    }
    
    
    void MotorDriver::Multi_ISTOP_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            unionType data_i, echo_i;
            data_i.data = 0x00;
            echo_i.data = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
            p_data.push_back(id_list[i]);
            p_data.push_back(this->cmd_ISTOP_Lite_);
            p_data.push_back(data_i.data_byte[1]);
            p_data.push_back(data_i.data_byte[0]);
            p_data.push_back(echo_i.data_byte[1]);
            p_data.push_back(echo_i.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_ISTOP_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_JG_Lite(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> data_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);

        for(auto i=0; i<id_list.size(); i++){
            unionType data_i, echo_i;
            data_i.data = data_list[i];
            echo_i.data = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
            p_data.push_back(id_list[i]);
            p_data.push_back(this->cmd_JG_Lite_);
            p_data.push_back(data_i.data_byte[1]);
            p_data.push_back(data_i.data_byte[0]);
            p_data.push_back(echo_i.data_byte[1]);
            p_data.push_back(echo_i.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_JG_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::Multi_FREE_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);

        for(auto i=0; i<id_list.size(); i++){
            unionType data_i, echo_i;
            data_i.data = 0x00;
            echo_i.data = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
            p_data.push_back(id_list[i]);
            p_data.push_back(this->cmd_FREE_Lite_);
            p_data.push_back(data_i.data_byte[1]);
            p_data.push_back(data_i.data_byte[0]);
            p_data.push_back(echo_i.data_byte[1]);
            p_data.push_back(echo_i.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_FREE_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_SVON_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);

        for(auto i=0; i<id_list.size(); i++){
            unionType data_i, echo_i;
            data_i.data = 0x00;
            echo_i.data = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
            p_data.push_back(id_list[i]);
            p_data.push_back(this->cmd_SVON_Lite_);
            p_data.push_back(data_i.data_byte[1]);
            p_data.push_back(data_i.data_byte[0]);
            p_data.push_back(echo_i.data_byte[1]);
            p_data.push_back(echo_i.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_SVON_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_SVOFF_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);

        for(auto i=0; i<id_list.size(); i++){
            unionType data_i, echo_i;
            data_i.data = 0x00;
            echo_i.data = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
            p_data.push_back(id_list[i]);
            p_data.push_back(this->cmd_SVOFF_Lite_);
            p_data.push_back(data_i.data_byte[1]);
            p_data.push_back(data_i.data_byte[0]);
            p_data.push_back(echo_i.data_byte[1]);
            p_data.push_back(echo_i.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_SVOFF_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_ALM_RST_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);

        for(auto i=0; i<id_list.size(); i++){
            unionType data_i, echo_i;
            data_i.data = 0x00;
            echo_i.data = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
            p_data.push_back(id_list[i]);
            p_data.push_back(this->cmd_ALM_RST_Lite_);
            p_data.push_back(data_i.data_byte[1]);
            p_data.push_back(data_i.data_byte[0]);
            p_data.push_back(echo_i.data_byte[1]);
            p_data.push_back(echo_i.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_ALM_RST_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_BRAKE_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);

        for(auto i=0; i<id_list.size(); i++){
            unionType data_i, echo_i;
            data_i.data = 0x00;
            echo_i.data = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
            p_data.push_back(id_list[i]);
            p_data.push_back(this->cmd_BRAKE_Lite_);
            p_data.push_back(data_i.data_byte[1]);
            p_data.push_back(data_i.data_byte[0]);
            p_data.push_back(echo_i.data_byte[1]);
            p_data.push_back(echo_i.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_BRAKE_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_NULL_Lite(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_Lite_);
        p_data.push_back(num);

        for(auto i=0; i<id_list.size(); i++){
            unionType data_i, echo_i;
            data_i.data = 0x00;
            echo_i.data = is_echo ? this->echo_bit_Lite_ : this->no_echo_bit_Lite_;
            p_data.push_back(id_list[i]);
            p_data.push_back(this->cmd_NULL_Lite_);
            p_data.push_back(data_i.data_byte[1]);
            p_data.push_back(data_i.data_byte[0]);
            p_data.push_back(echo_i.data_byte[1]);
            p_data.push_back(echo_i.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_NULL_Lite Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_ISTOP(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            uint8_t cmd = is_echo ? this->cmd_ISTOP_echo_ : this->cmd_ISTOP_no_echo_;
            unionType data_1, data_2;
            data_1.data = 0x00;
            data_2.data = 0x00;
            p_data.push_back(id_list[i]);
            p_data.push_back(cmd);
            p_data.push_back(data_1.data_byte[1]);
            p_data.push_back(data_1.data_byte[0]);
            p_data.push_back(data_2.data_byte[1]);
            p_data.push_back(data_2.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_ISTOP Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_JG(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> data_list,bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            int8_t cmd = is_echo ? this->cmd_JG_echo_ : this->cmd_JG_no_echo_;
            unionType data_1, data_2;
            data_1.data = 0x00;
            data_2.data = data_list[i];
            p_data.push_back(id_list[i]);
            p_data.push_back(cmd);
            p_data.push_back(data_1.data_byte[1]);
            p_data.push_back(data_1.data_byte[0]);
            p_data.push_back(data_2.data_byte[1]);
            p_data.push_back(data_2.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_JG Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;
    }

    void MotorDriver::Multi_FREE(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            uint8_t cmd = is_echo ? this->cmd_FREE_echo_ : this->cmd_FREE_no_echo_;
            unionType data_1, data_2;
            data_1.data = 0x00;
            data_2.data = 0x00;
            p_data.push_back(id_list[i]);
            p_data.push_back(cmd);
            p_data.push_back(data_1.data_byte[1]);
            p_data.push_back(data_1.data_byte[0]);
            p_data.push_back(data_2.data_byte[1]);
            p_data.push_back(data_2.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_FREE Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_SVON(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            uint8_t cmd = is_echo ? this->cmd_SVON_echo_ : this->cmd_SVON_no_echo_;
            unionType data_1, data_2;
            data_1.data = 0x00;
            data_2.data = 0x00;
            p_data.push_back(id_list[i]);
            p_data.push_back(cmd);
            p_data.push_back(data_1.data_byte[1]);
            p_data.push_back(data_1.data_byte[0]);
            p_data.push_back(data_2.data_byte[1]);
            p_data.push_back(data_2.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_SVON Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_SVOFF(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            uint8_t cmd = is_echo ? this->cmd_ISTOP_echo_ : this->cmd_ISTOP_no_echo_;
            unionType data_1, data_2;
            data_1.data = 0x00;
            data_2.data = 0x00;
            p_data.push_back(id_list[i]);
            p_data.push_back(cmd);
            p_data.push_back(data_1.data_byte[1]);
            p_data.push_back(data_1.data_byte[0]);
            p_data.push_back(data_2.data_byte[1]);
            p_data.push_back(data_2.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_ISTOP Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_IMR(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> index_list, std::vector<uint16_t> step_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            uint8_t cmd = is_echo ? this->cmd_IMR_echo_ : this->cmd_IMR_no_echo_;
            unionType data_1, data_2;
            data_1.data = index_list[i];
            data_2.data = step_list[i];
            p_data.push_back(id_list[i]);
            p_data.push_back(cmd);
            p_data.push_back(data_1.data_byte[1]);
            p_data.push_back(data_1.data_byte[0]);
            p_data.push_back(data_2.data_byte[1]);
            p_data.push_back(data_2.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_IMR Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_CS(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> index_list, std::vector<uint16_t> step_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            uint8_t cmd = is_echo ? this->cmd_CS_echo_ : this->cmd_CS_no_echo_;
            unionType data_1, data_2;
            data_1.data = index_list[i];
            data_2.data = step_list[i];

            p_data.push_back(id_list[i]);
            p_data.push_back(cmd);
            p_data.push_back(data_1.data_byte[1]);
            p_data.push_back(data_1.data_byte[0]);
            p_data.push_back(data_2.data_byte[1]);
            p_data.push_back(data_2.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_CS Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_CMR(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> index_list, std::vector<uint16_t> step_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            uint8_t cmd = is_echo ? this->cmd_CMR_echo_ : this->cmd_CMR_no_echo_;
            unionType data_1, data_2;
            data_1.data = index_list[i];
            data_2.data = step_list[i];
            p_data.push_back(id_list[i]);
            p_data.push_back(cmd);
            p_data.push_back(data_1.data_byte[1]);
            p_data.push_back(data_1.data_byte[0]);
            p_data.push_back(data_2.data_byte[1]);
            p_data.push_back(data_2.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_CMR Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_CMA(uint8_t num, std::vector<uint8_t> id_list, std::vector<int16_t> index_list, std::vector<uint16_t> step_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            uint8_t cmd = is_echo ? this->cmd_CMA_echo_ : this->cmd_CMA_no_echo_;
            unionType data_1, data_2;
            data_1.data = index_list[i];
            data_2.data = step_list[i];
            p_data.push_back(id_list[i]);
            p_data.push_back(cmd);
            p_data.push_back(data_1.data_byte[1]);
            p_data.push_back(data_1.data_byte[0]);
            p_data.push_back(data_2.data_byte[1]);
            p_data.push_back(data_2.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_CMA Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }

    void MotorDriver::Multi_NULL(uint8_t num, std::vector<uint8_t> id_list, bool is_echo){

        std::vector<uint8_t> p_data;
        p_data.clear();
        p_data.push_back(this->broadcast_);
        p_data.push_back(this->fc_master_send_cmd_);
        p_data.push_back(num);


        for(auto i=0; i<id_list.size(); i++){
            uint8_t cmd = is_echo ? this->cmd_NULL_echo_ : this->cmd_NULL_no_echo_;
            unionType data_1, data_2;
            data_1.data = 0x00;
            data_2.data = 0x00;
            p_data.push_back(id_list[i]);
            p_data.push_back(cmd);
            p_data.push_back(data_1.data_byte[1]);
            p_data.push_back(data_1.data_byte[0]);
            p_data.push_back(data_2.data_byte[1]);
            p_data.push_back(data_2.data_byte[0]);
        }

        uint16_t crc = this->calculate_CRC(p_data);
        p_data.push_back(crc);
        p_data.push_back(crc >> 8);
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);

        std::cout <<  "Send Multi_NULL Command: ";
        for(auto i=0;i<p_char.size();i++){            
            std::cout << std::hex << (int)p_data[i] << " ";
        }
        std::cout << std::endl;

    }






}