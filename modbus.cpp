
#include "modbus.h"

#define WRITE_DELAY 10000

namespace Motor
{
    // 建構元
    SerialModbus::SerialModbus(const std::string _serial_port, const int _baud_rate){
        set_param_serial(_serial_port, _baud_rate);
    }
    // 解構元：關閉port
    SerialModbus::~SerialModbus(){
        close();
    }

    void SerialModbus::set_param_serial(const std::string _serial_port, const int _baud_rate){
        p_serial_port = _serial_port;
        p_baud_rate = _baud_rate;
    }

    int SerialModbus::open(){
        if(p_port != NULL){
            return -1;
        }

        if(p_service){
            p_port.reset();
            p_service.reset();
        }
            
        p_mutex = std::shared_ptr<boost::mutex>{new boost::mutex};
        p_service = std::shared_ptr<io_service>{new io_service()};
        p_port = std::shared_ptr<serial_port>{ new serial_port(*p_service) };
        p_timeout = std::shared_ptr<deadline_timer>{new deadline_timer(*p_service)};

        try 
        {
            p_port->open(p_serial_port);
            p_port->set_option(serial_port_base::baud_rate(p_baud_rate));
            // 數據位
            p_port->set_option(serial_port_base::character_size(8));
            // 奇偶校驗
            p_port->set_option(serial_port_base::parity(serial_port_base::parity::none));
            // 停止位
            p_port->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            return 0;
        }
        catch (std::exception &ex){
            std::cout << "open exception : " << ex.what() << std::endl;
        }
        // 原本沒有return。
        return 0;           
    }

    int SerialModbus::close(){
        if(p_port && p_port->is_open()){
             p_port->close();
        }
        // 原本沒有return。
        return 0;           
    }


    void SerialModbus::write(std::vector<char> _data){
	    if (p_port->is_open()){
            boost::mutex::scoped_lock lock(*p_mutex);
            auto size = p_port->write_some(buffer(_data));
            // std::cout << "Writen size: " << size << std::endl;
            if(_data.size() != size){
                throw "Write Size Error!!!";
            }      
        }
        else
        {
            throw "Port not open";
        }
    }    
    
    void SerialModbus::single_register_write(uint8_t _id, uint8_t _function_code, uint16_t _addr, uint16_t _data)
    {
        // 建立資料的空陣列
        std::vector<uint8_t> p_data;
        p_data.clear();
        // 存入 _id, _func, _addr, _data 等資訊
        p_data.push_back(_id);
        p_data.push_back(_function_code);
        p_data.push_back(_addr >> 8);
        p_data.push_back(_addr);
        p_data.push_back(_data >> 8);
        p_data.push_back(_data);
        // 依照前面存入的資料計算 CRC 碼，並再次存入
        uint16_t crc = this->calculate_crc(p_data);
        p_data.push_back(crc >> 8);
        p_data.push_back(crc);
        
        // 發送訊息
        std::vector<char> p_char(p_data.begin(), p_data.end());
        this->write(p_char);
        // 後面可能需要去接受回覆，並且檢測CRC碼的動作，但目前先不加。

    }

    // min_rcv 規定讀取字節，照規格書上寫的去設定。
    std::vector<char> SerialModbus::asyncRead(size_t rcv_size)
    {
        p_service->reset();
        p_available = false;
        std::vector<char> p_char(rcv_size);
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
            boost::mutex::scoped_lock sl(*p_mutex);
            async_read( *p_port, 
                        boost::asio::buffer(p_char, rcv_size),
                        [&](const boost::system::error_code &error, std::size_t bytes_transferred)
                        {
                            if (error)
                            {
                                p_available = false;
                                std::cerr << "readCallback Error " << error << std::endl;
                            }
                            p_timeout->cancel();
                            p_available = true;
                        });
            p_timeout->expires_from_now(boost::posix_time::millisec(READ_TIME_OUT_MS));
            p_timeout->async_wait(  [&](const boost::system::error_code &error)
                                    {
                                        if (!error)
                                        {
                                            p_available = false;
                                            p_port->cancel(); 
                                            std::cerr << "Read timeout" << std::endl;
                                        }
                                    });
            p_service->run(); 
        }
        catch(const std::exception& ex)
        {
            std::cout << "Read exception. " << ex.what() << std::endl;
        }
        
        if (p_available)
        {
            return p_char;
        }
        else
        {
            throw "Serial port reading timeout";
        }
    }

    void SerialModbus::writeOnly(uint8_t _ID, uint8_t _FC, uint16_t _ADDR, uint16_t _DATA){
        const std::lock_guard<std::mutex> lock(p_std_mutex);
        try
        {
            single_register_write(_ID, _FC, _ADDR, _DATA);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    std::vector<char> SerialModbus::read_and_write(uint8_t _ID, uint8_t _FC, uint16_t _ADDR, uint16_t _DATA, int expected_bytes){
        // 寫入
        // writeOnly(_ID, _FC, _ADDR, _DATA);
        // 讀取
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
        return response;
    }




    /*
    ========================= 待刪除 =========================
    */



    // // 5引數：函數呼叫順序：{ single_register_read >> single_register_write >> write >> read } 即從port讀取資料
    // // 引數分別為：起始位(0)、站號(1,2)、功能碼(3,4)、地址、資料
    // // Q：地址（_address）和資料（_data）分別是什麼？
    // // A：暫存器地址 & 讀到的資料數據
    // std::vector<u_int16_t> SerialModbus::single_register_read(const char *_start_byte, const char *_id, const char *_function_code, const char *_address, const char *_data) {
    //     // auto 運算子會讓系統自動判別資料型態
    //     // p_read_data 是 SerialPort 讀到的東西
    //     // 引數的部份是因為需要計算LRC，確認傳輸無誤。
    //     auto p_read_data = single_register_write(_start_byte, _id, _function_code, _address, _data);
    //     // .at(i) 讀取第i個元素，此函數會自動作邊界處理，相對安全。
    //     // Q：p_read_data 第6、7格資料是？？
    //     char data_size[2]={p_read_data.at(5), p_read_data.at(6)};
    //     // 把 data_size 從 ascii 制轉成 16 進制
    //     int p_data_size = ascii_2_data(data_size);
    //     std::vector<u_int16_t> p_data_set;
    //     // 這一個迴圈在把資料作解碼（ascii > hex）
    //     // Q：但依照的規則是？
    //     for(int i=0; i < p_data_size / 2; i+=4) 
    //     {
    //         auto data = ascii4_2_data(&p_read_data.at(i+7));
    //         p_data_set.push_back(data);
    //     }
    //     // 回傳解碼完的 hex 資料。
    //     return p_data_set;
    // }
    // // 2引數：呼叫5引數的同名函數。
    // std::vector<u_int16_t> SerialModbus::single_register_read(const char *_address, const char *_data) {
    //     return single_register_read(":", MODBUS_ID, "03", _address, _data);
    // }

    // std::vector<char> SerialModbus::read() {
    //     try
    //     {
    //         const int max_char{ 64 };
    //         char buff[max_char]{ 0 };
    //         boost::mutex::scoped_lock sl(*p_mutex);
    //         size_t total_size = 0;
    //         std::vector<char> p_char;
    //         p_char.clear();
    //         while(p_port->is_open())
    //         {
    //             // 從port讀取資料
    //             auto read_size = p_port->read_some(boost::asio::buffer(buff, max_char));
    //             if(read_size > 0)
    //             {
    //                 // 複製buff一部分資料到p_char_tmp
    //                 std::vector<char> p_char_tmp(buff, buff+read_size);
    //                 // 把p_char_tmp資料插入到p_char後端（應該是後端吧？！）
    //                 p_char.insert(p_char.end(), p_char_tmp.begin(), p_char_tmp.end());
    //                 // 如果p_char_tmp的最末位為10，回傳p_char
    //                 if(p_char_tmp.at(p_char_tmp.size()-1)==10) {
    //                     return p_char;
    //                 }
    //             }
    //             else 
    //             {
    //                 usleep(1000);
    //                 continue;
    //             }                
    //         }  
    //         throw "Serial Port Is Not Open";
    //     }
    //     catch (const std::exception& ex)
    //     {
    //         std::cout << "Read exception " << ex.what() << std::endl;
    //     }
    // }
    // // 1引數Write：這應該是把資料寫進去的部份？？
    // std::vector<char> SerialModbus::write(std::vector<char> _data) {
    //     // 這個函數包含了寫入和讀取SerialPort，應該是在確認送出的資料無誤？？
	//     if (p_port->is_open()) { 
    //         for(auto i=0; i < _data.size(); i++) 
    //         {
    //             // toupper是把字元轉換為大寫的函數：a -> A
    //             _data[i] = (char)toupper(_data[i]);
    //         }
    //         {
    //             boost::mutex::scoped_lock lock(*p_mutex);
    //             auto size = p_port->write_some(buffer(_data));
    //             //printf("write_size: %d\n", (int)size);
    //             if(_data.size() != size) {
    //                 throw "Write Size Error!!!";
    //             }                    
    //         }      
    //         usleep(100);
    //         // 從port讀取資料
    //         auto p_read_data = read();
    //         // 資料的第四、第五位是功能碼
    //         if(p_read_data.at(3) == '0' && p_read_data.at(4) == '6') {
    //             for(auto i = p_read_data.size()-2; i < p_read_data.size(); i++) 
    //             {
    //                 // 資料異常確認：檢驗碼（末兩碼）確認（收到的資料和寫出的資料應該要相同）
    //                 if(p_read_data.at(i) != _data.at(i)) {
    //                     throw "Data Write Error";                        
    //                 }
    //             }
    //         }         
    //         return p_read_data;
    //     }
    //     throw "Port not open";
    // }

    // // 5引數Write
    // std::vector<char> SerialModbus::write(const char *_start_byte, const char *_id, const char *_function_code, const char *_data, const int _data_segment) {
    //     std::vector<char> p_data(0);

    //     // 堆疊資料p_data，應是follow Modbus協議的格式
    //     // 起始碼
    //     p_data.push_back(_start_byte[0]);
    //     // 站號
    //     p_data.push_back(_id[0]);
    //     p_data.push_back(_id[1]);
    //     // 功能碼
    //     p_data.push_back(_function_code[0]);
    //     p_data.push_back(_function_code[1]);
    //     // 資料內容
    //     for(auto i = 0; i < _data_segment; i++) {
    //         p_data.push_back(_data[i]);
    //     }
    //     // 檢驗碼
    //     char p_lrc_hi{0};
    //     char p_lrc_lo{0};
    //     lrc(_id, _function_code, _data, _data_segment, &p_lrc_hi, &p_lrc_lo);
    //     p_data.push_back(p_lrc_hi);
    //     p_data.push_back(p_lrc_lo);
    //     // \r：（回車）回到行首、\n：跳至下一行；加在一起就是跑到下一行行首，也就是ENTER的基本功能。
    //     // Q：不過為什麼需要這個？？
    //     // A：作為結束符，CR LF。
    //     p_data.push_back('\r');
    //     p_data.push_back('\n');
    //     // 呼叫1引數Write：return SerialPort讀到的東西
    //     return write(p_data);
    // }
    // // 5引數
    // std::vector<char> SerialModbus::single_register_write(const char *_start_byte, const char *_id, const char *_function_code, const char *_address, const char *_data) {
    //     const int p_data_segment{8};
    //     const int p_address_bytes{4};
    //     char p_data[8];
    //     // 把_address 內容複製到 p_data 的前4格。
    //     for(auto i = 0; i < p_address_bytes; i++) {
    //         p_data[i] = _address[i];
    //     }
    //     // 把_data 內容複製到 p_data 的後4格。
    //     for(auto i = 0; i < p_data_segment - p_address_bytes; i++) {
    //         p_data[i+4] = _data[i];
    //     }
    //     // return SerialPort 讀到的東西
    //     return write(_start_byte, _id, _function_code, p_data, p_data_segment);
    // }
    // // 2引數：呼叫5引數的同名函數。
    // std::vector<char> SerialModbus::single_register_write(const char *_address, const char *_data) {
    //     return single_register_write(":", MODBUS_ID, "06", _address, _data);
    // }

    /*
    ========================= 待刪除 =========================
    */



}
