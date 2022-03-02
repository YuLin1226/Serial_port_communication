#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


#define _DEBUG
#ifdef _DEBUG
#define __DEBUG__ true
#else
#define __DEBUG__ false
#endif


#ifndef __TRUMMAN_H__
#define __TRUMMAN_H__

using namespace boost::asio;

namespace Motor
{
    const int FAIL_READ_MAX_COUNT = 5;
    const int RESPONSE_DELAY_US = 1500; // Default 3300 300us(C3.5) + 3ms(Tb2)
    const int64_t READ_TIME_OUT_MS = 500; // 100ms for timeout
    const int16_t MAX_ENC_DELTA = 4096;
    const int16_t HALF_MAX_ENC_DELTA = 2048;
    const int16_t ENC_RESOLUTION = 4096;
    class SerialModbus
    {
        protected:
            /*
            shared_ptr 是C++11提供的一種智能指針類，可以在任何地方都不使用時自動刪除相關指針，從而幫助徹底消除內存泄漏和懸空指針的問題。
            */
            std::shared_ptr<io_service> p_service;
            std::shared_ptr<serial_port> p_port;
            std::shared_ptr<boost::mutex> p_mutex;
            std::shared_ptr<boost::mutex> p_func_mutex;
            std::string p_serial_port;
            int p_baud_rate;

        public:
            SerialModbus(const std::string _serial_port, const int _baud_rate);
            virtual ~SerialModbus();

        public:
            void set_param_serial(const std::string _serial_port, const int _baud_rate);
            int open();
            int close();


            void write(std::vector<char> _data);
            void single_register_write(uint8_t _id, uint8_t _function_code, uint16_t _addr, uint16_t _data);
            std::vector<char> asyncRead(size_t min_rcv);
            void readCallback(deadline_timer &timeout, const boost::system::error_code &error, std::size_t bytes_transferred);
            void timeoutCallback(serial_port &ser_port, const boost::system::error_code &error);

        protected:
            bool p_is_read_timeout;
            bool p_available;
            std::shared_ptr<deadline_timer> p_timeout;


            /*
            ========================= 待刪除 =========================
            */

            // std::vector<char> read();
            // std::vector<u_int16_t> single_register_read(const char *_start_byte, const char *_id, const char *_function_code, const char *_address, const char *_data);
            // std::vector<u_int16_t> single_register_read(const char *_address, const char *_data);

            // std::vector<char> write(std::vector<char> _data);
            // std::vector<char> write(const char *_start_byte, const char *_id, const char *_function_code, const char *_data, const int _data_segment);
            // std::vector<char> single_register_write(const char *_start_byte, const char *_id, const char *_function_code, const char *_address, const char *_data);
            // std::vector<char> single_register_write(const char *_address, const char *_data);

            /*
            ========================= 待刪除 =========================
            */

        protected:


            inline uint16_t calculate_crc(std::vector<uint8_t> _data){ 
                uint16_t crc = 0xFFFF;  
                for(auto idx=0; idx<_data.size(); idx++){
                    crc ^= (uint16_t)_data[idx];
                    for(auto i=8; i!=0; i--){

                        if((crc & 0x0001) != 0){
                            crc >>= 1;
                            crc ^= 0xA001;
                        }
                        else{
                            crc >>= 1;
                        }
                    }
                }
                return crc;
            }



            /*
            ================================== 待刪除 =====================================
            */

            // inline int ascii_2_data(const char *_ascii) {
            //     // 把_ascii的前兩個字元放到p_data裡
            //     const char p_data[] = {_ascii[0], _ascii[1], '\0'};
            //     // 搜尋p_data前面字串的數字部份，並以16進宣告該數字。
            //     int number = (int)strtol(p_data, NULL, 16);
            // }
            // inline int ascii4_2_data(const char *_ascii) {
            //     // 概念同上
            //     const char p_data[] = {_ascii[0], _ascii[1], _ascii[2], _ascii[3], '\0'};
            //     int number = (int)strtol(p_data, NULL, 16);
            // }
            // inline void data_2_ascii(int _data, char *_ascii) {
            //     /*
            //     這個函數就是把_data以16進存到_ascii裡面。
            //     */
            //     const int MAX_BYTES{4};
            //     // stringstream 可以想成是轉換字串的容器，也可以想成是轉換器。
            //     std::stringstream ss{};
            //     // setw 是設定字串長度，不足該長度，前面會補空格；setfill把空格改成用0補。
            //     // 所以字串前面是0，後面是16進制的_data，例如：000000001f4 這種概念。
            //     ss << std::setw(MAX_BYTES) << std::setfill('0') << std::hex << (int)_data;
            //     // 把ss的字串複製到str裡。
            //     std::string str = ss.str();
            //     // 再把str的字串一個一個放到ascii裡。
            //     for(auto i=0; i<MAX_BYTES; i++)
            //         _ascii[i] = str.c_str()[i];
            // }
            // // 四引數
            // inline unsigned char lrc(const char *_id, const char *_function_code, const char *_data, const int _data_segment) {
            //     std::vector<int> p_data;
            //     // p_data 會儲存從ascii轉譯為hex的 _id, _function_code, _data, _data_segment 資料，然後就可以拿來算 LRC 。
            //     p_data.push_back(ascii_2_data(_id));
            //     p_data.push_back(ascii_2_data(_function_code));                
            //     for(auto i = 0; i < _data_segment; i+=2) {
            //         p_data.push_back(ascii_2_data(_data+i));
            //     }
            //     // Q：這一部份要搞懂的是：p_lrc是用來幹麻的？ 
            //     // A：LRC 用來當檢查碼，因為沒有轉換器可以自動生成，所以需要自己計算。
            //     // 把accumulate函數把p_data作加總，加總後和0x00FF作按位與，再用256減去結果，可得LRC檢驗碼。
            //     unsigned char p_lrc = 256 -(std::accumulate(p_data.begin(), p_data.end(), 0) & 0x00FF);
            //     return p_lrc;
            // }
            // // 三引數
            // inline void lrc(const unsigned char _lrc, char *_lrc_hi, char *_lrc_lo) {
            //     const int MAX_BYTES{2};
            //     // stringstream 可以想成是轉換字串的容器，也可以想成是轉換器。
            //     std::stringstream ss{};
            //     // setw 是設定字串長度，不足該長度，前面會補空格；setfill把空格改成用0補。
            //     // 所以字串前面是0，後面是16進制的_lrc，例如：000000001f4 這種概念。
            //     ss << std::setw(MAX_BYTES) << std::setfill('0') << std::hex << (int)_lrc;
            //     // 把ss的字串複製到str裡。
            //     std::string str = ss.str();
            //     // 分別把str的前兩個字元取出、放入指標，作為通訊協定中的LRC高位與低位。
            //     *_lrc_hi = str.c_str()[0];
            //     *_lrc_lo = str.c_str()[1];
            // }
            // // 六引數
            // inline unsigned char lrc(const char *_id, const char *_function_code, const char *_data, const int _data_segment, char *_lrc_hi, char *_lrc_lo) {
            //     // 呼叫四引數lrc
            //     auto p_lrc = lrc(_id, _function_code, _data, _data_segment);
            //     // 呼叫三引數lrc
            //     lrc(p_lrc, _lrc_hi, _lrc_lo);
            //     // Q：這邊是不是少寫一個return阿？
            //     /* A： 
            //         > 應該沒差，三引數lrc要的是把p_lrc的資料寫進去_lrc_hi, _lrc_lo兩個裡面。
            //         > 所以它把這個函式當成過程，然後引數是指標，所以還是更動得到資料。
            //         > 我自己覺得，如果應要寫，大概是加個return;而已。
            //     */
            // }

            /*
            ================================== 待刪除 =====================================
            */


    };
}// namespace: Motor
#endif