#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <libserial/SerialPort.h>
#include <chrono>
#include <vector>
#include <cstring>
#include <thread>

using namespace LibSerial;
using namespace std::chrono_literals;

class ESP32Communicator : public rclcpp::Node
{
public:
    ESP32Communicator() : Node("esp32_communicator_cpp")
    {
        declare_parameter("serial_port", "/dev/ttyUSB0");
        declare_parameter("baud_rate", 115200);

        serial_port_name_ = get_parameter("serial_port").as_string();
        baud_rate_ = get_parameter("baud_rate").as_int();

        response_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("esp32_response", 10);

        if (!open_serial_port()) {
            RCLCPP_FATAL(get_logger(), "Konnte seriellen Port %s nicht öffnen", serial_port_name_.c_str());
            return;
        }

        timer_ = create_wall_timer(1s, std::bind(&ESP32Communicator::communicate, this));

        RCLCPP_INFO(get_logger(), "ESP32 Communicator gestartet (Port: %s, Baud: %d)",
                    serial_port_name_.c_str(), baud_rate_);
    }

    ~ESP32Communicator()
    {
        if (serial_port_.IsOpen()) serial_port_.Close();
    }

private:
    bool open_serial_port()
    {
        try {
            serial_port_.Open(serial_port_name_);
            serial_port_.SetBaudRate(BaudRate::BAUD_115200);
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            serial_port_.SetParity(Parity::PARITY_NONE);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
            std::this_thread::sleep_for(2s);
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Fehler beim Öffnen des Ports: %s", e.what());
            return false;
        }
    }

    void communicate()
    {
        int32_t tx_data[4] = {42, 123, 456, 789};
        int32_t rx_data[2] = {0};

        auto t0 = std::chrono::high_resolution_clock::now();
        if (exchange_data(tx_data, rx_data)) {
            auto t1 = std::chrono::high_resolution_clock::now();
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

            std_msgs::msg::Int32MultiArray msg;
            msg.data.assign(rx_data, rx_data + 2);
            response_pub_->publish(msg);

            RCLCPP_INFO(get_logger(),
                        "TX: [%d, %d, %d, %d] | RX: [%d, %d] | Zeit: %ld µs",
                        tx_data[0], tx_data[1], tx_data[2], tx_data[3],
                        rx_data[0], rx_data[1], us);
        } else {
            RCLCPP_WARN(get_logger(), "Keine gültige Antwort vom ESP32");
        }
    }

    bool exchange_data(const int32_t* tx, int32_t* rx)
    {
        try {
            // Start-Marker
            send_markers({0xFF, 0xFE});

            // Nutzdaten senden
            std::vector<uint8_t> tx_bytes(sizeof(int32_t) * 4);
            std::memcpy(tx_bytes.data(), tx, tx_bytes.size());
            serial_port_.Write(tx_bytes);

            // End-Marker
            send_markers({0xFD, 0xFC});

            // Antwort empfangen
            std::vector<uint8_t> rx_bytes(sizeof(int32_t) * 2);
            size_t i = 0;
            auto deadline = std::chrono::steady_clock::now() + 1s;

            while (i < rx_bytes.size() && std::chrono::steady_clock::now() < deadline) {
                if (serial_port_.IsDataAvailable()) {
                    uint8_t byte;
                    serial_port_.ReadByte(byte);
                    rx_bytes[i++] = byte;
                } else {
                    std::this_thread::sleep_for(1ms);
                }
            }

            if (i == rx_bytes.size()) {
                std::memcpy(rx, rx_bytes.data(), rx_bytes.size());
                return true;
            }

            return false;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Kommunikationsfehler: %s", e.what());
            return false;
        }
    }

    void send_markers(const std::initializer_list<uint8_t> &markers)
    {
        for (auto byte : markers)
            serial_port_.WriteByte(static_cast<unsigned char>(byte));
    }

    SerialPort serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr response_pub_;
    std::string serial_port_name_;
    int baud_rate_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ESP32Communicator>());
    rclcpp::shutdown();
    return 0;
}