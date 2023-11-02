#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, Int16, Float32
from math import *
from pymodbus.client.sync import ModbusSerialClient

CHASSIS_RADIUS = 0.26
WHEEL_RADIUS = 0.1
jog = [0, 0, 0]

def decimal_to_hex(input_value):
    hex_value = hex(input_value & 0xFFFF).upper()
    return f'0x{hex_value[2:]:0>4}'

def write_registers(client, slave, direction, velocity):
    if direction == "clockwise":
        values = [0x0000, int(velocity, 16)]
    elif direction == "anticlockwise":
        values = [0xFFFF, int(velocity, 16)]
    else:
        values = [0x0000, 0x0000]
    client.write_registers(342, values, unit=slave)

def motor_control(client, slave):
    input_value = jog[(slave - 1)]
    # print(f"MOTOR {slave}: ", input_value)
    hex_value = decimal_to_hex(input_value)

    if input_value > 0:
        write_registers(client, slave, "clockwise", hex_value)
    elif input_value < 0:
        write_registers(client, slave, "anticlockwise", hex_value)
    else:
        write_registers(client, slave, "stop", "0x0000")

def cmd_vel_callback(data):
    global jog
    linear_x = data.linear.x
    linear_y = data.linear.y
    angular_z = data.angular.z

    wheels = [
        (-sqrt(3)/2 * linear_x + linear_y/2 + CHASSIS_RADIUS * angular_z) / WHEEL_RADIUS,
        (0 * linear_x - linear_y + CHASSIS_RADIUS * angular_z) / WHEEL_RADIUS,
        (sqrt(3)/2 * linear_x + linear_y/2 + CHASSIS_RADIUS * angular_z) / WHEEL_RADIUS
    ]

    jog[0] = int(round((wheels[0] * 240 * 20) / (pi * 2 * WHEEL_RADIUS), 0))
    jog[1] = int(round((wheels[1] * 240 * 20) / (pi * 2 * WHEEL_RADIUS), 0))
    jog[2] = int(round((wheels[2] * 240 * 20) / (pi * 2 * WHEEL_RADIUS), 0))

    # print(jog)

def publish_speed(client):
    speed_1 = client.read_input_registers(16, 1, unit=1)
    if speed_1.registers[0] <= 32767:
        speed_1_data = speed_1.registers[0] * pi / 24000
    else:
        speed_1_data = (speed_1.registers[0] - 2**16) * pi / 24000
    speed_1_pub.publish(speed_1_data)


    speed_2 = client.read_input_registers(16, 1, unit=2)
    if speed_2.registers[0] <= 32767:
        speed_2_data = speed_2.registers[0] * pi / 24000
    else:
        speed_2_data = (speed_2.registers[0] - 2**16) * pi / 24000
    speed_2_pub.publish(speed_2_data)


    speed_3 = client.read_input_registers(16, 1, unit=3)
    if speed_3.registers[0] <= 32767:
        speed_3_data = speed_3.registers[0] * pi / 24000 
    else:
        speed_3_data = (speed_3.registers[0] - 2**16) * pi / 24000
    speed_3_pub.publish(speed_3_data)

    
def publish_encoder(client):
    encoder_1 = client.read_input_registers(10, 2, unit=1)
    hex_1_1 = encoder_1.registers[0]
    hex_1_2 = encoder_1.registers[1]
    combined_hex_1 = f"0x{hex_1_1:04X}{hex_1_2:04X}"
    combined_decimal_1 = int(combined_hex_1, 16)
    if combined_decimal_1 > 2147483647:
        combined_decimal_1 = combined_decimal_1 - 2**32
    encoder_1_pub.publish(combined_decimal_1) 

    encoder_2 = client.read_input_registers(10, 2, unit=2)
    hex_2_1 = encoder_2.registers[0]
    hex_2_2 = encoder_2.registers[1]
    combined_hex_2 = f"0x{hex_2_1:04X}{hex_2_2:04X}"
    combined_decimal_2 = int(combined_hex_2, 16)
    if combined_decimal_2 > 2147483647:
        combined_decimal_2 = combined_decimal_2 - 2**32
    encoder_2_pub.publish(combined_decimal_2) 
    
    encoder_3 = client.read_input_registers(10, 2, unit=3)
    hex_3_1 = encoder_3.registers[0]
    hex_3_2 = encoder_3.registers[1]
    combined_hex_3 = f"0x{hex_3_1:04X}{hex_3_2:04X}"
    combined_decimal_3 = int(combined_hex_3, 16)
    if combined_decimal_3 > 2147483647:
        combined_decimal_3 = combined_decimal_3 - 2**32
    encoder_3_pub.publish(combined_decimal_3)
    print(combined_decimal_1)
    print(combined_decimal_2)
    print(combined_decimal_3)


def publish_alarm(client):
    alarm_1 = client.read_input_registers(0, 2, unit=1)
    alarm_1_data = f"{alarm_1.registers[0]}" + f"{alarm_1.registers[1]}"
    alarm_1_pub.publish(alarm_1_data)

    alarm_2 = client.read_input_registers(0, 2, unit=2)
    alarm_2_data = f"{alarm_2.registers[0]}" + f"{alarm_2.registers[1]}"
    alarm_2_pub.publish(alarm_2_data)

    alarm_3 = client.read_input_registers(0, 2, unit=3)
    alarm_3_data = f"{alarm_3.registers[0]}" + f"{alarm_3.registers[1]}"
    alarm_3_pub.publish(alarm_3_data)

def main():
    client = ModbusSerialClient(
        method="rtu",
        port="/dev/ttyS3",
        baudrate=19200,
        stopbits=1,
        parity="N",
        timeout=0.2,
    )

    if not client.connect():
        print("Failed to connect to Modbus device")
        return

    try:
        print("Connected to Modbus device")
        for slave in [1, 2, 3]:
            client.write_registers(0x7C, [0x0098, 0x0000], unit=slave)
            client.write_registers(0x7C, [0x00A5, 0x0000], unit=slave)
            client.write_registers(342, [0x0000, 0x0000], unit=slave)
            client.write_register(0x7C, 0x96, unit=slave)
        print("Motors ready!")

        while not rospy.is_shutdown():
            publish_speed(client)
            publish_encoder(client)
            publish_alarm(client)
            for slave in [1, 2, 3]:
                motor_control(client, slave)

    except Exception as e:
        print(f"An error occurred: {e}")
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Turn off the motors!")
        for slave in [1, 2, 3]:
            client.write_registers(342, [0x0000, 0x0000], unit=slave)
            client.write_register(0x7C, 0xD8, unit=slave)
        client.close()

if __name__ == '__main__':
    rospy.init_node('cmd_vel_subscriber_node', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    encoder_1_pub = rospy.Publisher("/encoder_1", Int64, queue_size=10)
    encoder_2_pub = rospy.Publisher("/encoder_2", Int64, queue_size=10)
    encoder_3_pub = rospy.Publisher("/encoder_3", Int64, queue_size=10)
    alarm_1_pub = rospy.Publisher("/alarm_1", Int16, queue_size=10)
    alarm_2_pub = rospy.Publisher("/alarm_2", Int16, queue_size=10)
    alarm_3_pub = rospy.Publisher("/alarm_3", Int16, queue_size=10)
    speed_1_pub = rospy.Publisher("/speed_1", Float32, queue_size=10)
    speed_2_pub = rospy.Publisher("/speed_2", Float32, queue_size=10)
    speed_3_pub = rospy.Publisher("/speed_3", Float32, queue_size=10)
    main()
