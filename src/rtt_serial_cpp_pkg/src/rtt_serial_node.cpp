#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialStream.h>
#include <chrono>
#include <sstream>

using namespace LibSerial;
using namespace std::chrono;

class RTTSerialNode : public rclcpp::Node {
public:
    RTTSerialNode() : Node("rtt_serial_node") {
        try {
            serial_port_.Open("/dev/ttyUSB0");
            serial_port_.SetBaudRate(BaudRate::BAUD_115200);
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
            serial_port_.SetParity(Parity::PARITY_NONE);
            RCLCPP_INFO(this->get_logger(), "Serial port geöffnet.");
        } catch (const OpenFailed&) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Öffnen des Ports!");
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            1000ms,
            std::bind(&RTTSerialNode::perform_rtt_test, this)
        );
    }

private:
    SerialStream serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;

    void perform_rtt_test() {
        auto send_time = high_resolution_clock::now();
        std::ostringstream oss;
        oss << duration_cast<nanoseconds>(send_time.time_since_epoch()).count() << "\n";

        serial_port_ << oss.str();
        RCLCPP_INFO(this->get_logger(), "Gesendet: %s", oss.str().c_str());

        std::string line;
        std::getline(serial_port_, line);

        if (!line.empty()) {
            auto receive_time = high_resolution_clock::now();
            auto rtt = duration_cast<milliseconds>(receive_time - send_time).count();
            RCLCPP_INFO(this->get_logger(), "Antwort erhalten: %s | RTT: %ld micros", line.c_str(), rtt);
        } else {
            RCLCPP_WARN(this->get_logger(), "Keine Antwort erhalten.");
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RTTSerialNode>());
    rclcpp::shutdown();
    return 0;
}
