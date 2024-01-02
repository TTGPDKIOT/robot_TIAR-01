#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import serial
import binascii
import json
import paho.mqtt.client as mqtt
import time

data_sent1 = False
data_sent2 = False
data_sent3 = False
phan_tram_pin = None
dong_dien = None
a = 0
b = 0
c = 0
# Khoi tao client MQTT
mqtt_broker_address = "10.14.17.129"
mqtt_port = 1883
mqtt_user = "iot"
mqtt_password = "1"
mqtt_topic = "pin"
client = mqtt.Client()

def send_and_receive_data():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        hex_data = [0x7E, 0x32, 0x30, 0x30, 0x31, 0x34, 0x36, 0x34, 0x32, 0x45, 0x30, 0x30, 0x32, 0x30, 0x31, 0x46, 0x44, 0x33, 0x35, 0x0D]
        data_bytes = bytes(hex_data)
        # Gui du lieu toi pin
        ser.write(data_bytes)
        # du lieu pin tra ve 
        response = ser.read(150)  
        # Chuyen du lieu pin tra ve sang hex
        hex_response = binascii.hexlify(response).decode('ascii')
        # Tach chuoi thanh cac nhom co do dai 2 ki tu
        data_groups = [hex_response[i:i+2] for i in range(0, len(hex_response), 2)]
        # Goi ham dung luong pin
        dung_luong_pin_con_lai(data_groups)
        dong_dien(data_groups)
    except Exception as e:
        print(f"Error: {e}")

def dung_luong_pin_con_lai(data_groups):
    global phan_tram_pin
    # Lay kis tu 69 den 73 trong list du lieu pin tra ve
    selected_chars = data_groups[69:73]
    # Chuyen thanh chuoi du lieu
    result_string = ''.join(selected_chars)
    # tach chuoi du lieu thanh chuoi nho voi do dai moi chuoi 2 kis tu
    ascii_result = ''.join([chr(int(result_string[i:i+2], 16)) for i in range(0, len(result_string), 2)])
    # chuyen tu ket qua cua chuoi ascii tu hex sang thap phan
    hex_string = ascii_result
    decimal_result = int(hex_string, 16)
    phan_tram_pin = (decimal_result) // 100
    
def dong_dien(data_groups):
    global dong_dien
    # Lay ky tu thu 61 den 65
    selected_chars = data_groups[61:65]
    # print(selected_chars)
    # Chuyen danh sach ky tu thanh chuoi
    result_string = ''.join(selected_chars)
    # Chuyen chuoi hex thanh chuoi ASCII
    ascii_result = ''.join([chr(int(result_string[i:i+2], 16)) for i in range(0, len(result_string), 2)])
    # chuyen tu ket qua cua chuoi ascii tu hex sang thap phan
    decimal_result = int(ascii_result, 16)
    # print("dong dien:", decimal_result / 1000,"A")
    dong_dien = (decimal_result) / 1000
    
def pub_value(pub):
    global phan_tram_pin, c
    if phan_tram_pin != c:
        pub.publish(phan_tram_pin)
        c = phan_tram_pin

def pub_trang_thai(pub2):
    global data_sent1, dong_dien, data_sent2
    if dong_dien > 35 and not data_sent1:
        a = 0
        pub2.publish(a)
        data_sent1 = True
        data_sent2 = False
    elif dong_dien <= 35 and not data_sent2:
        a = 1
        pub2.publish(a)
        data_sent2 = True
        data_sent1 = False

def ep_kieu():
    global phan_tram_pin, dong_dien, a, b
    if phan_tram_pin != a and dong_dien != b:
        data = {
            "dung_luong_pin": f"{phan_tram_pin}", 
            "trang_thai_pin": "2" if phan_tram_pin == 100 else ("1" if dong_dien < 35 else "0")
        
        }
        # Chuyen doi san chuoi json
        json_data = json.dumps(data, ensure_ascii=False).encode('utf-8').decode()
        # in ra chuoi json
        print(json_data)
        # Gui du lieu len MQTT
        client.publish("Power", json_data)
        a = phan_tram_pin 
        b = dong_dien
    
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        global Connected
        Connected = True
    else:
        print("Connection failed")

def MQTT():
    # Khoi tao client MQTT
    global Connected
    Connected = False
    # tao 1 phien ban may khach MQTT
    client.on_connect = on_connect
    # Ket noi toi broker MQTT
    client.username_pw_set(mqtt_user, password=mqtt_password)
    client.connect(mqtt_broker_address, port=mqtt_port)
    client.loop_start()

def publisher():
    global data_sent1, data_sent2, phan_tram_pin 
    rospy.init_node('READ_PIN', anonymous=True)
    pub = rospy.Publisher('pin', Float32, queue_size=10, latch=True)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        send_and_receive_data()
        pin_thap(pub)
        dang_sac(pub)
        binh_thuong(pub)
        ep_kieu()
        rate.sleep()

if __name__ == '__main__':
    try:
        MQTT()
        publisher()
    except KeyboardInterrupt:
        # ngat ket noi MQTT va cong serial
        client.disconnect()
        ser.close()


