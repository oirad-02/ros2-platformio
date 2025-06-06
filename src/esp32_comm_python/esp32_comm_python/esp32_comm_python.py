#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import struct
import time

class ESP32Communicator(Node):
    def __init__(self):
        super().__init__('esp32_communicator_python')
        
        # Parameter deklarieren
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Publisher für empfangene Daten
        self.response_publisher = self.create_publisher(
            Int32MultiArray, 
            'esp32_response', 
            10
        )
        
        # Seriellen Port öffnen
        if self.open_serial_port():
            # Timer für regelmäßige Kommunikation (alle 1 Sekunde)
            self.timer = self.create_timer(1.0, self.send_and_receive)
            
            self.get_logger().info('ESP32 Communicator Python gestartet')
            self.get_logger().info(f'Port: {self.serial_port_name}, Baudrate: {self.baud_rate}')
        else:
            self.get_logger().error(f'Konnte seriellen Port nicht öffnen: {self.serial_port_name}')
    
    def open_serial_port(self):
        """Öffnet den seriellen Port"""
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,  # 1 Sekunde Timeout
                write_timeout=1.0
            )
            
            # Kurz warten bis Port bereit ist
            time.sleep(2.0)
            
            # Buffer leeren
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f'Fehler beim Öffnen des Ports: {str(e)}')
            return False
    
    def send_and_receive(self):
        """Sendet 4 Integers und empfängt 2 Integers vom ESP32"""
        if not hasattr(self, 'serial_port') or not self.serial_port.is_open:
            self.get_logger().error('Serieller Port ist nicht geöffnet')
            return
        
        # Beispiel-Array mit 4 Integern
        send_data = [42, 123, 456, 789]
        
        start_time = time.time()
        
        try:
            receive_data = self.communicate_with_esp32(send_data)
            
            end_time = time.time()
            duration_ms = (end_time - start_time) * 1000  # in Millisekunden
            
            if receive_data is not None:
                # Empfangene Daten publishen
                msg = Int32MultiArray()
                msg.data = receive_data
                self.response_publisher.publish(msg)
                
                self.get_logger().info(
                    f'Gesendet: {send_data} | '
                    f'Empfangen: {receive_data} | '
                    f'Zeit: {duration_ms:.2f} ms'
                )
            else:
                self.get_logger().error('Kommunikation mit ESP32 fehlgeschlagen')
                
        except Exception as e:
            self.get_logger().error(f'Fehler bei Kommunikation: {str(e)}')
    
    def communicate_with_esp32(self, send_data):
        """
        Kommuniziert mit dem ESP32 via serieller Schnittstelle
        
        Args:
            send_data: Liste mit 4 Integern
            
        Returns:
            Liste mit 2 Integern oder None bei Fehler
        """
        try:
            # Buffer leeren
            self.serial_port.reset_input_buffer()
            
            # Start-Marker senden
            self.serial_port.write(bytes([0xFF, 0xFE]))
            
            # 4 Integers als Bytes senden (little-endian, 4 Bytes pro Int)
            send_bytes = struct.pack('<4i', *send_data)
            self.serial_port.write(send_bytes)
            
            # End-Marker senden
            self.serial_port.write(bytes([0xFD, 0xFC]))
            
            # 2 Integers empfangen (8 Bytes)
            receive_bytes = self.serial_port.read(8)
            
            if len(receive_bytes) == 8:
                # Bytes zu Integers konvertieren
                receive_data = list(struct.unpack('<2i', receive_bytes))
                return receive_data
            else:
                self.get_logger().error(f'Unvollständige Daten empfangen: {len(receive_bytes)}/8 Bytes')
                return None
                
        except serial.SerialTimeoutError:
            self.get_logger().error('Timeout bei serieller Kommunikation')
            return None
        except serial.SerialException as e:
            self.get_logger().error(f'Serieller Fehler: {str(e)}')
            return None
        except Exception as e:
            self.get_logger().error(f'Allgemeiner Fehler: {str(e)}')
            return None
    
    def destroy_node(self):
        """Schließt den seriellen Port beim Beenden"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serieller Port geschlossen')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    esp32_communicator = ESP32Communicator()
    
    try:
        rclpy.spin(esp32_communicator)
    except KeyboardInterrupt:
        pass
    finally:
        esp32_communicator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



