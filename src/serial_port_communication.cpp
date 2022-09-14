#include "../include/serial_port_communication.h"

namespace Communication
{
    SerialPort::SerialPort(const std::string serial_port, const int baud_rate)
    {
        setParam(serial_port, baud_rate);
    }

    SerialPort::~SerialPort()
    {
        closeSerialPort();
    }
    
    void SerialPort::setParam(const std::string serial_port, const int baud_rate)
    {
        serial_port_   = serial_port;
        baud_rate_     = baud_rate;
    }

    int SerialPort::openSerialPort()
    {
        if(port_ != NULL)
        {
            return -1;
        }
        if(service_)
        {
            port_.reset();
            service_.reset();
        }

        mutex_     = std::shared_ptr<boost::mutex>{new boost::mutex};
        service_   = std::shared_ptr<io_service>{new io_service()};
        port_      = std::shared_ptr<serial_port>{ new serial_port(*service_) };
        timeout_   = std::shared_ptr<deadline_timer>{new deadline_timer(*service_)};

        try 
        {
            // Set port parameters.
            port_->open(serial_port_);
            port_->set_option(serial_port_base::baud_rate(baud_rate_));
            port_->set_option(serial_port_base::character_size(8));
            port_->set_option(serial_port_base::parity(serial_port_base::parity::none));
            port_->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            std::cout << ">>> Serial Port Connect." << std::endl;
        }
        catch (std::exception &ex)
        {
            std::cout << ">>> Open Exception : " << ex.what() << std::endl;
        }
        return 0;       
    }

    int SerialPort::closeSerialPort()
    {
        if(port_ && port_->is_open())
        {
            port_->close();
        }
        std::cout << ">>> Serial Port Disconnect." << std::endl;
        return 0;      
    }


    void SerialPort::writeDataThroughSerialPort(std::vector<char> data_vector)
    {
	    if (port_->is_open())
        {
            boost::mutex::scoped_lock lock(*mutex_);
            auto size = port_->write_some(buffer(data_vector));
            if(data_vector.size() != size)
            {
                throw ">>> Write Size Error.";
            }      
        }
        else
        {
            throw ">>> Serial Port Is Not Open.";
        }
    }    
    
    

    std::vector<char> SerialPort::asyncReadDataThroughSerialPort(size_t data_size)
    {
        service_->reset();
        data_available_ = false;
        std::vector<char> char_vector(data_size);
        try
        {
            /* =============================== 邏輯解釋 ===============================
                * scoped_lock 用於保護線程。
                * async_read 用於異步讀取資料，並且讀取到 "rcv_size" 長度的字節才會呼叫Lambda表達式，即讀取完畢。
                *   1) 讀取完畢的error_code  = 0 (False)
                * timeout expires_from_now 設定時間。
                * timeout async_wait 等待已設定的時間，其呼叫 Lambda 有兩種情況：
                *   1) 過期，此時error_code  = 0 (False)
                *   2) 取消，此時error_code != 0 (True)
                * service_run 阻塞後續工作，等待線程內的事項結束。
                ---------------------------------------------------------------------
                1-1) 讀取 -> 成功 -> 取消定時器 -> 定時器不做事。
                1-2) 讀取 -> 失敗 -> 拋出錯誤
                2-1) 定時器 -> 過期 -> 取消序列埠工作 -> 拋出錯誤              
               =============================== 邏輯解釋 ===============================*/
            boost::mutex::scoped_lock scoped_locker(*mutex_);
            async_read( *port_, 
                        boost::asio::buffer(char_vector, data_size),
                        [&](const boost::system::error_code &error, std::size_t bytes_transferred)
                        {
                            if (error)
                            {
                                data_available_ = false;
                                std::cerr << ">>> readCallback Error " << error << std::endl;
                            }
                            else
                            {
                                data_available_ = true;
                            }
                            timeout_->cancel();
                        });
            timeout_->expires_from_now(boost::posix_time::millisec(READ_TIME_OUT_MS));
            timeout_->async_wait(   [&](const boost::system::error_code &error)
                                    {
                                        if (!error)
                                        {
                                            data_available_ = false;
                                            port_->cancel(); 
                                            std::cerr << ">>> Read timeout." << std::endl;
                                        }
                                    });
            service_->run(); 
        }
        catch(const std::exception& ex)
        {
            std::cout << ">>> Read exception. " << ex.what() << std::endl;
        }
        if (data_available_)
        {
            std::cout << ">>> Read successfully. \n";
            return char_vector;
        }
        else
        {
            throw ">>> Serial port reading timeout";
        }
    }
    
    // void SerialModbus::write_to_single_register(uint8_t _id, uint8_t _function_code, uint16_t _addr, uint16_t _data)
    // {
    //     /* =====================================================================================
    //         * 建立Bytes資料容器
    //         * 存入 ID, FunctionCode, Address, Data, CRC
    //         * 轉換Char資料容器
    //         * 發送資料
    //        ===================================================================================== */
    //     std::vector<uint8_t> p_data;
    //     p_data.clear();
    //     p_data.push_back(_id);
    //     p_data.push_back(_function_code);
    //     p_data.push_back(_addr >> 8);
    //     p_data.push_back(_addr);
    //     p_data.push_back(_data >> 8);
    //     p_data.push_back(_data);
    //     uint16_t crc = this->calculate_CRC(p_data);
    //     p_data.push_back(crc >> 8);
    //     p_data.push_back(crc);
    //     std::vector<char> p_char(p_data.begin(), p_data.end());
    //     this->write(p_char);
    // }

    

    // std::vector<char> SerialModbus::read_and_write(uint8_t _ID, uint8_t _FC, uint16_t _ADDR, uint16_t _DATA, int expected_bytes){
    
    //     /* 這個功能之後會作為特定函數的模板，例如：get_MotorSpeed，那就是先寫入再讀取 */
    //     write_to_single_register(_ID, _FC, _ADDR, _DATA);    
    //     std::vector<char> response;
    //     {
    //         usleep(RESPONSE_DELAY_US);
    //         try
    //         {
    //             response = asyncRead(expected_bytes);
    //         }
    //         catch(const std::exception& e)
    //         {
    //             std::cerr << e.what() << '\n';
    //         }
    //     }
    //     return response;
    // }
}
