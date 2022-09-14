// #include "../../include/serial_port_communication.h"
#include "../../include/motor_driver/motor_driver.h"
#include <cmath>

namespace AMR
{

    union convertUint8AndUint16
    {
        // [0] Low 8 bits
        // [1] High 8 bits 
        uint8_t data8[2]; 
        uint16_t data16;
    };


    MotorDriver::MotorDriver(const std::string serial_port, const int baud_rate, std::shared_ptr<io_service> service):
    Communication::SerialPort(serial_port, baud_rate, service), cmd_(CMD_NUMBER::doNothing)
    {
        thread_ = std::thread([this]()
        {
            setRunning(true);
            thread_body();
        });
    }

    MotorDriver::~MotorDriver()
    {
        stop();
    }

    void MotorDriver::thread_body()
    {
        while(getRunning())
        {
            if(cmd_ == CMD_NUMBER::doNothing)
            {
            }
            else if(cmd_ == CMD_NUMBER::enableServo)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                writeDataThroughSerialPort(write_data_vector_);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            else if(cmd_ == CMD_NUMBER::velocityControl)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                writeDataThroughSerialPort(write_data_vector_);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            else if(cmd_ == CMD_NUMBER::positionControl)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                writeDataThroughSerialPort(write_data_vector_);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            else if(cmd_ == CMD_NUMBER::readEncoder)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                if(!write_data_vector_.empty())
                {
                    writeDataThroughSerialPort(write_data_vector_);
                    write_data_vector_.clear();
                    // std::this_thread::sleep_for(std::chrono::microseconds(Communication::RESPONSE_DELAY_US));
                    try
                    {
                        const int expected_bytes = 9;
                        read_data_vector_ = asyncReadDataThroughSerialPort(expected_bytes);
                        std::cout << "Received Data: ";
                        for (auto i = 0; i < read_data_vector_.size(); i++)
                        {
                            std::cout << std::hex << (int)read_data_vector_[i] << " ";
                        }
                        std::cout << std::endl;
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << '\n';
                    }
                }
                else
                {
                    std::cout << "empty write data vector.\n";
                }
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            
            cmd_ = CMD_NUMBER::doNothing;
        }
        std::cout << ">>> Thread body is finished" << std::endl;
    }

    void MotorDriver::printReadBuf()
    {
        if(read_data_vector_.empty())
        {
            std::cout << "No received data.\n";
        }
        else
        {
            std::cout << "Received Data: ";
            for (auto i = 0; i < read_data_vector_.size(); i++)
            {
                std::cout << std::hex << (int)read_data_vector_[i] << " ";
            }
            std::cout << std::endl;
        }
    }

    void MotorDriver::setRunning(bool running)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        running_ = running;
    }

    bool MotorDriver::getRunning()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return running_;
    }

    void MotorDriver::stop()
    {
        setRunning(false);
        thread_.join();
    }

    void MotorDriver::exampleModbusCommand(uint16_t data)
    {
        std::vector<uint8_t> data_uint8_vector;

        convertUint8AndUint16 data_converted;
        data_converted.data16 = data;
        
        data_uint8_vector.clear();

        data_uint8_vector.push_back(example_broadcast_);
        data_uint8_vector.push_back(example_function_code_);
        data_uint8_vector.push_back(exmaple_number_);
        data_uint8_vector.push_back(example_motor_id_);
        data_uint8_vector.push_back(example_motor_drive_cmd_);
        data_uint8_vector.push_back(data_converted.data8[1]);
        data_uint8_vector.push_back(data_converted.data8[0]);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[0]);
        
        std::vector<char> data_char_vector(data_uint8_vector.begin(), data_uint8_vector.end());
        writeDataThroughSerialPort(data_char_vector);

    }

    double MotorDriver::exampleGetEncoder()
    {

        // First: write command to ask motor driver to send back data.
        // This part takes "exampleModbusCommand()" to demonstrate "asking command".
        uint16_t ask_motor_driver_to_send_back_data = 0x0000;
        exampleModbusCommand(ask_motor_driver_to_send_back_data);

        // Then: async read data.
        std::vector<char> data_received;
        {
            usleep(Communication::RESPONSE_DELAY_US);
            try
            {
                const int expected_bytes = 8;
                data_received = asyncReadDataThroughSerialPort(expected_bytes);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        // Finally: process received data. This should follow motor driver's manual.
        double encoder_data;
        convertUint8AndUint16 encoder_turn, encoder_step;
        encoder_turn.data8[1] = data_received.at(2);
        encoder_turn.data8[0] = data_received.at(3);
        encoder_step.data8[1] = data_received.at(4);
        encoder_step.data8[0] = data_received.at(5);
        encoder_data = (encoder_turn.data16 + encoder_step.data16/10000.0)*2.0*M_PI;
        return encoder_data;
    }


    void MotorDriver::velocityControl(uint8_t id)
    {
        std::vector<uint8_t> data_uint8_vector;

        data_uint8_vector.clear();

        data_uint8_vector.push_back(id);
        data_uint8_vector.push_back(0x10);
        data_uint8_vector.push_back(0x44);
        data_uint8_vector.push_back(0x20);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x04);
        // following 4 push_back are vel data.
        data_uint8_vector.push_back(0x03);
        data_uint8_vector.push_back(0xE8);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[1]);
        
        std::vector<char> data_char_vector(data_uint8_vector.begin(), data_uint8_vector.end());
        writeDataThroughSerialPort(data_char_vector);
    }

    void MotorDriver::enableMotorDriver(uint8_t id)
    {
        std::vector<uint8_t> data_uint8_vector;

        data_uint8_vector.clear();

        data_uint8_vector.push_back(id);
        data_uint8_vector.push_back(0x10);
        data_uint8_vector.push_back(0x46);
        data_uint8_vector.push_back(0x57);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x01);
        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x01);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[1]);
        
        std::vector<char> data_char_vector(data_uint8_vector.begin(), data_uint8_vector.end());
        std::cout << "Send Command: ";
        for (auto i = 0; i < data_char_vector.size(); i++)
        {
            std::cout << std::hex << (int)data_char_vector[i] << " ";
        }
        std::cout << std::endl;

        
        writeDataThroughSerialPort(data_char_vector);

        std::vector<char> data_received;
        {
            usleep(Communication::RESPONSE_DELAY_US);
            try
            {
                const int expected_bytes = 8;
                data_received = asyncReadDataThroughSerialPort(expected_bytes);
                std::cout << "Received Command: ";
                for (auto i = 0; i < data_received.size(); i++)
                {
                    std::cout << std::hex << (int)data_received[i] << " ";
                }
                std::cout << std::endl;
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }

    void MotorDriver::stopVelocityControl(uint8_t id)
    {
        std::vector<uint8_t> data_uint8_vector;

        data_uint8_vector.clear();

        data_uint8_vector.push_back(id);
        data_uint8_vector.push_back(0x10);
        data_uint8_vector.push_back(0x44);
        data_uint8_vector.push_back(0x20);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x04);
        // following 4 push_back are vel data.
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[1]);
        
        std::vector<char> data_char_vector(data_uint8_vector.begin(), data_uint8_vector.end());
        writeDataThroughSerialPort(data_char_vector);
    }

    void MotorDriver::positionGoHome(uint8_t id)
    {
        std::vector<uint8_t> data_uint8_vector;

        data_uint8_vector.clear();

        data_uint8_vector.push_back(id);
        data_uint8_vector.push_back(0x10);
        data_uint8_vector.push_back(0x44);
        data_uint8_vector.push_back(0x8F);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x01);
        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x01);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[1]);
        
        std::vector<char> data_char_vector(data_uint8_vector.begin(), data_uint8_vector.end());
        writeDataThroughSerialPort(data_char_vector);
    }

    void MotorDriver::positionControl(uint8_t id)
    {
        std::vector<uint8_t> data_uint8_vector;

        data_uint8_vector.clear();

        data_uint8_vector.push_back(id);
        data_uint8_vector.push_back(0x10);
        data_uint8_vector.push_back(0x43);
        data_uint8_vector.push_back(0xC6);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x04);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x00);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[1]);
        
        std::vector<char> data_char_vector(data_uint8_vector.begin(), data_uint8_vector.end());
        writeDataThroughSerialPort(data_char_vector);
        std::cout << "Send Command (Pos set): ";
        for (auto i = 0; i < data_char_vector.size(); i++)
        {
            std::cout << std::hex << (int)data_char_vector[i] << " ";
        }
        std::cout << std::endl;

        // |Above| set position
        // sleep between 2 commands 
        // |Below| start move
        usleep(25000);
        // sleep(1);

        data_uint8_vector.clear();

        data_uint8_vector.push_back(0x03);
        data_uint8_vector.push_back(0x10);
        data_uint8_vector.push_back(0x43);
        data_uint8_vector.push_back(0xBF);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x01);
        data_uint8_vector.push_back(0x02);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x01);

        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[1]);
        
        data_char_vector.assign(data_uint8_vector.begin(), data_uint8_vector.end());
        writeDataThroughSerialPort(data_char_vector);
        std::cout << "Send Command (Pos start): ";
        for (auto i = 0; i < data_char_vector.size(); i++)
        {
            std::cout << std::hex << (int)data_char_vector[i] << " ";
        }
        std::cout << std::endl;


        std::vector<char> data_received;
        {
            usleep(Communication::RESPONSE_DELAY_US);
            try
            {
                const int expected_bytes = 8;
                data_received = asyncReadDataThroughSerialPort(expected_bytes);
                std::cout << "Received Command: ";
                for (auto i = 0; i < data_received.size(); i++)
                {
                    std::cout << std::hex << (int)data_received[i] << " ";
                }
                std::cout << std::endl;
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }

    void MotorDriver::readEncoder(uint8_t id)
    {
        std::vector<uint8_t> data_uint8_vector;

        data_uint8_vector.clear();
        data_uint8_vector.push_back(id);
        data_uint8_vector.push_back(0x03);
        data_uint8_vector.push_back(0x42);
        data_uint8_vector.push_back(0xFF);
        data_uint8_vector.push_back(0x00);
        data_uint8_vector.push_back(0x02);

        convertUint8AndUint16 crc_code;
        crc_code.data16 = computeCRC16(data_uint8_vector);
        data_uint8_vector.push_back(crc_code.data8[0]);
        data_uint8_vector.push_back(crc_code.data8[1]);
    
        write_data_vector_.assign(data_uint8_vector.begin(), data_uint8_vector.end());
        
        std::cout << "Send Command: ";
        for(auto i=0; i<write_data_vector_.size(); i++)
        {
            std::cout << std::hex << (int)write_data_vector_[i] << " ";
        }
        std::cout << std::endl;

        cmd_ = CMD_NUMBER::readEncoder;



        // writeDataThroughSerialPort(data_char_vector);

        // std::vector<char> data_received;
        // {
        //     usleep(Communication::RESPONSE_DELAY_US);
        //     try
        //     {
        //         const int expected_bytes = 9;
        //         data_received = asyncReadDataThroughSerialPort(expected_bytes);
        //         std::cout << "Received Command: ";
        //         for (auto i = 0; i < data_received.size(); i++)
        //         {
        //             std::cout << std::hex << (int)data_received[i] << " ";
        //         }
        //         std::cout << std::endl;
        //     }
        //     catch(const std::exception& e)
        //     {
        //         std::cerr << e.what() << '\n';
        //     }
        // }
    }



}