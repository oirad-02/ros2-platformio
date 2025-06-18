import rclpy
from rclpy.node import Node
import serial
import time

class RTTSerialNode(Node):
    def __init__(self):
        super().__init__('rtt_serial_node')
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info('Serial port geöffnet.')
        except serial.SerialException:
            self.get_logger().error('Fehler beim Öffnen des Ports!')
            rclpy.shutdown()
            return

        self.timer = self.create_timer(1.0, self.perform_rtt_test)

    def perform_rtt_test(self):
        send_time = time.time()
        msg = f"{send_time:.9f}\n"
        self.ser.write(msg.encode())
        self.get_logger().info(f'Gesendet: {msg.strip()}')

        try:
            response = self.ser.readline().decode().strip()
            if response:
                receive_time = time.time()
                rtt = (receive_time - send_time) * 1000
                self.get_logger().info(f'Antwort erhalten: {response} | RTT: {rtt:.2f} ms')
            else:
                self.get_logger().warn("Keine Antwort erhalten.")
        except Exception as e:
            self.get_logger().error(f'Fehler beim Lesen: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RTTSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()