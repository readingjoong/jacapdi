import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand
from .lib import protocol_convert_func_lib as PCFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"

# 아두이노 장치 이름 (ls /dev/ttyA* 명령을 터미널 창에 입력하여 확인)
PORT='/dev/ttyACM0'

# Arduino driving.ino uses Serial.begin(115200)
BAUD_RATE=115200
#----------------------------------------------

class SerialSenderNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME):
    super().__init__('serial_sender_node')
    
    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('port', PORT)
    self.declare_parameter('baud_rate', BAUD_RATE)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.port = self.get_parameter('port').get_parameter_value().string_value
    self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
    
    self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
    time.sleep(1)
    
    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                             history=QoSHistoryPolicy.KEEP_LAST, 
                             durability=QoSDurabilityPolicy.VOLATILE, 
                             depth=1)
    
    self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, qos_profile)

  def data_callback(self, msg):
    steering = msg.steering
    left_speed = msg.left_speed
    right_speed = msg.right_speed

    serial_msg =  PCFL.convert_serial_message(steering, left_speed, right_speed)
    self.ser.write(serial_msg.encode())

  def stop(self):
    message = PCFL.convert_serial_message(0, 0, 0)
    self.ser.write(message.encode())
    self.ser.close()

def main(args=None):
  rclpy.init(args=args)
  node = SerialSenderNode()
  try:
      rclpy.spin(node)
      
  except KeyboardInterrupt:
      print("\n\nshutdown\n\n")
      steering = 0
      left_speed = 0
      right_speed = 0
      message = PCFL.convert_serial_message(steering, left_speed, right_speed)
      node.ser.write(message.encode())
      pass
    
  finally:
    node.stop()
    print('closed')
    
  node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
