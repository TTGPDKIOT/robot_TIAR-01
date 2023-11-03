#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int64MultiArray
from math import *
from pymodbus.client.sync import ModbusSerialClient

class MotorController:
    def __init__(self):
        self.CHASSIS_RADIUS = 0.26
        self.WHEEL_RADIUS = 0.1
        self.jog = [0, 0, 0]
        self.client = self.initialize_modbus_client()

        rospy.init_node('cmd_vel_subscriber_node', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.encoder_pub = rospy.Publisher('encoder_pub', Int64MultiArray, queue_size=10)
        self.speed_pub = rospy.Publisher('speed_pub', Float32MultiArray, queue_size=10)
        self.alarm_pub = rospy.Publisher('alarm_pub', Int32MultiArray, queue_size=10)

    def initialize_modbus_client(self):
        return ModbusSerialClient(
            method="rtu",
            port="/dev/ttyS3",
            baudrate=19200,
            stopbits=1,
            parity="N",
            timeout=0.2
        )

    def connect_to_modbus(self):
        if not self.client.connect():
            rospy.logerr("Failed to connect to Modbus device")
            return False
        return True

    def setup_motors(self):
        for unit in [1, 2, 3]:
            self.client.write_registers(342, [0x0000, 0x0000], unit=unit)
            self.client.write_register(0x7C, 0x96, unit=unit)
        rospy.loginfo("Motors ready!")

    def cmd_vel_callback(self, data):
        linear_x = data.linear.x
        linear_y = data.linear.y
        angular_z = data.angular.z

        wheels = [
            (-sqrt(3) / 2 * linear_x + linear_y / 2 + self.CHASSIS_RADIUS * angular_z) / self.WHEEL_RADIUS,
            (0 * linear_x - linear_y + self.CHASSIS_RADIUS * angular_z) / self.WHEEL_RADIUS,
            (sqrt(3) / 2 * linear_x + linear_y / 2 + self.CHASSIS_RADIUS * angular_z) / self.WHEEL_RADIUS
        ]

        self.jog[0] = int(round((wheels[0] * 240 * 20) / (pi * 2), 0))
        self.jog[1] = int(round((wheels[1] * 240 * 20) / (pi * 2), 0))
        self.jog[2] = int(round((wheels[2] * 240 * 20) / (pi * 2), 0))

        self.publish_speed()

    def write_velocity(self):
        hex_values = []
        for value in self.jog:
            hex_value = f'0x{value & 0xFFFFFFFF:08X}'
            hex_values.append(hex_value)

        int_values = [int(hex_value, 16) for hex_value in hex_values]

        for unit, int_value in enumerate(int_values, start=1):
            self.client.write_registers(342, [int_value >> 16, int_value & 0xFFFF], unit=unit)

    def publish_speed(self):
        speed_data = Float32MultiArray() 
        speed_data.data = []  

        for unit in [1, 2, 3]:
            speed = self.client.read_input_registers(16, 1, unit=unit)
            if speed.registers[0] <= 32767:
                speed = round(speed.registers[0] * pi / 24000, 2)
            else:
                speed = round((speed.registers[0] - 2 ** 16) * pi / 24000, 2)
            speed_data.data.append(speed)

        self.speed_pub.publish(speed_data)

    def publish_encoders(self):
        encoder_data = Int64MultiArray() 
        encoder_data.data = [] 

        for unit in [1, 2, 3]:
            encoder = self.client.read_input_registers(10, 2, unit=unit)
            combined_hex = f"0x{encoder.registers[0]:04X}{encoder.registers[1]:04X}"
            combined_decimal = int(combined_hex, 16)

            if combined_decimal > (2**16 - 1):
                combined_decimal -= 2**16
            encoder_data.data.append(combined_decimal)

        self.encoder_pub.publish(encoder_data)

    def publish_alarm(self):
        alarm_data = Int32MultiArray() 
        alarm_data.data = []  

        for unit in [1, 2, 3]:
            alarm = self.client.read_input_registers(0, 2, unit=unit)
            alarm_data_value = int(f"{alarm.registers[0]:04X}{alarm.registers[1]:04X}", 16)
            alarm_data.data.append(alarm_data_value)

            if alarm_data_value != 0:
                rospy.logwarn(f"Driver {unit} has Warning!")

        self.alarm_pub.publish(alarm_data)

    def run(self):
        if not self.connect_to_modbus():
            return

        try:
            rospy.loginfo("Connected to Modbus device")
            self.setup_motors()
            while not rospy.is_shutdown():
                self.publish_alarm()
                self.publish_encoders()
                self.write_velocity()

        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")
        except KeyboardInterrupt:
            rospy.logwarn("KeyboardInterrupt: Turn off the motors!")
            for unit in [1, 2, 3]:
                self.client.write_registers(342, [0x0000, 0x0000], unit=unit)
                self.client.write_register(0x7C, 0xD8, unit=unit)
            self.client.close()

if __name__ == '__main__':
    motor_controller = MotorController()
    motor_controller.run()
