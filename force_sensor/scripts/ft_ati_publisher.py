#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import InterfaceValue
import math

class FtAtiPublisher(Node):
    def __init__(self):
        super().__init__('ft_ati_publisher')
        self.publisher_ = self.create_publisher(InterfaceValue, '/ft_ati_controller/inputs', 1)
        timer_period = 1  # Pubblica ogni tot secondi
        self.counter = 0 #contatore per gestire il cambio valore
        self.current_value = -10.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):

        msg = InterfaceValue()
        msg.interface_names = [
            'analog_inputs/sensor_channel_0',
            'analog_inputs/sensor_channel_1',
            'analog_inputs/sensor_channel_2',
            'analog_inputs/sensor_channel_3',
            'analog_inputs/sensor_channel_4',
            'analog_inputs/sensor_channel_5',
            'analog_inputs/a_input_6',
            'analog_inputs/a_input_7',
            'analog_inputs/force_x',
            'analog_inputs/force_y',
            'analog_inputs/force_z',
            'analog_inputs/torque_x',
            'analog_inputs/torque_y',
            'analog_inputs/torque_z'
        ]

        #cambio valore ogni 5 secondi
        if self.counter < 10:
             self.current_value = -10.0 
        else :
             self.current_value = 100.0

        self.counter +=1
        if self.counter == 20 :
             self.counter = 0

        # assegno valore a tutte le interfacce
        msg.values = [float(self.current_value)] * len(msg.interface_names)
        
        # Log del messaggio pubblicato
        self.get_logger().info(f'Publishing: {msg}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ft_ati_publisher = FtAtiPublisher()
    rclpy.spin(ft_ati_publisher)
    ft_ati_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
