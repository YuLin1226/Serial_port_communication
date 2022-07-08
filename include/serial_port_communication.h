#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#define _DEBUG
#ifdef _DEBUG
#define __DEBUG__ true
#else
#define __DEBUG__ false
#endif

#ifndef _SERIAL_PORT_COMMUNICATION_H_
#define _SERIAL_PORT_COMMUNICATION_H_

using namespace boost::asio;

namespace Communication
{
    // Some global param used for serial port communication
    const int FAIL_READ_MAX_COUNT       = 5;
    const int RESPONSE_DELAY_US         = 1500; // Default 3300 300us(C3.5) + 3ms(Tb2)
    const int64_t READ_TIME_OUT_MS      = 100;  // 100ms for timeout
    const int16_t MAX_ENC_DELTA         = 4096;
    const int16_t HALF_MAX_ENC_DELTA    = 2048;
    const int16_t ENC_RESOLUTION        = 4096;

    class SerialPort
    {
        
        private:
            // serial port name
            std::string serial_port_;
            
            // baud rate
            int baud_rate_;

            // service
            std::shared_ptr<io_service> service_;

            // port
            std::shared_ptr<serial_port> port_;

            // mutex
            std::shared_ptr<boost::mutex> mutex_;
            // std::shared_ptr<boost::mutex> func_mutex_;
            // std::mutex std_mutex_;

            // timer
            std::shared_ptr<deadline_timer> timeout_;

            // data available in async read
            bool data_available_;

        public:
            /**
             * @brief Constructor.
             * @param serial_port port name, e.g. "/dev/ttyUSB0".
             * @param baud_rate baud rate, e.g. 115200.
             */
            SerialPort(const std::string serial_port, const int baud_rate);

            /**
             * @brief Virtaul destructor, classes inherit this class should customize their own destructors.
             */
            virtual ~SerialPort();

            /**
             * @brief Setting parameters.
             * @param serial_port port name, e.g. "/dev/ttyUSB0".
             * @param baud_rate baud rate, e.g. 115200.
             */
            void setParam(const std::string serial_port, const int baud_rate);

            /**
             * @brief Function of opening port.
             */
            int openSerialPort();

            /**
             * @brief Function of closing port.
             */
            int closeSerialPort();

            /**
             * @brief Function of writing data to target through serial port.
             * @param data_vector a char vector containing data.
             */
            void writeDataThroughSerialPort(std::vector<char> data_vector);

            /**
             * @brief Function of receiving data in asynchronous way.
             * @param data_size size of received data.
             */
            std::vector<char> asyncReadDataThroughSerialPort(size_t data_size);


            // void write_to_single_register(uint8_t id, uint8_t function_code, uint16_t _addr, uint16_t _data);
            // std::vector<char> read_and_write(uint8_t _ID, uint8_t _FC, uint16_t _ADDR, uint16_t _DATA, int expected_bytes);
            

        // protected:
            

        //     inline uint16_t calculate_CRC(std::vector<uint8_t> _data){ 
        //         uint16_t crc = 0xFFFF;  
        //         for(auto idx=0; idx<_data.size(); idx++){
        //             crc ^= (uint16_t)_data[idx];
        //             for(auto i=8; i!=0; i--){
        //                 if((crc & 0x0001) != 0){
        //                     crc >>= 1;
        //                     crc ^= 0xA001;
        //                 }
        //                 else{
        //                     crc >>= 1;
        //                 }
        //             }
        //         }
        //         return crc;
        //     }
        //     inline bool validate_CRC(std::vector<char> _data){
        //         std::vector<uint8_t> rcv_data(_data.begin(), _data.end()-2);
        //         uint16_t crc = calculate_CRC(rcv_data);
        //         uint8_t calc_crc_hi = crc;
        //         uint8_t calc_crc_lo = crc >> 8;
        //         uint8_t data_crc_hi = 0xFF & _data.at(_data.size() - 1) ;
        //         uint8_t data_crc_lo = 0xFF & _data.at(_data.size() - 2);
        //         if (data_crc_hi != calc_crc_hi || data_crc_lo != calc_crc_lo)
        //         {
        //             fprintf(stderr, "Invalid CRC. Received: %02x%02x, Calculated: %02x%02x\n",data_crc_hi, data_crc_lo, calc_crc_hi, calc_crc_lo);
        //             return 0;
        //         }
        //         return 1;
        //     }

    };
}

#endif