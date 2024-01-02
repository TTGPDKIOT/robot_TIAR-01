#!/usr/bin/env python3

import json
from pymodbus.client import ModbusSerialClient
import paho.mqtt.client as mqtt
import rospy
# from datetime import datetime
import time

# def current_time():
#     now = datetime.now().isoformat()
#     return now

# Khoiclient Modbus RTU
client_modbus = ModbusSerialClient(method='rtu', port='/dev/ttyUSB0', baudrate=9600, stopbits=1, bytesize=8, parity='N', timeout=1)

# Khoi tao client MQTT
Connected = False
mqtt_broker_address = "10.14.7.175"
port = 1883
user = "iot" 
password = "1"

# tao 1 phien ban may khach MQTT
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        global Connected
        Connected = True
    else:
        print("Connection failed")

# Ket noi toi broker MQTT
client.username_pw_set(user, password= password)
client.on_connect = on_connect
client.connect(mqtt_broker_address, port=port)
client.loop_start()

def read_sensor():
    # Kiem tra ket noi Modbus
    if client_modbus.connect():
        # Đoc thanh ghi modbus
        result = client_modbus.read_holding_registers(address=0, count=6, slave=1)
        co2 = result.registers[0]
        pm25 = result.registers[3]
        humidity = result.registers[4]
        temperature = result.registers[5]

        # Kiem tra loi doc thanh ghi
        if result.isError():
            print("Loi doc thanh ghi")
        else:
            # Tao dictionary ket qua doc duoc tu cam bien
            data = {
                "CO2": str(co2),
                "PM25": str(pm25),
                "Temperature": str(temperature/100),
                "Humidity": str(humidity/100),
                # "dateTime": current_time()
                }

            # Chuyen doi san chuoi json
            json_data = json.dumps(data, ensure_ascii=False).encode('utf-8').decode()

            # in ra chuoi json
            print(json_data)

            # Gui du lieu len MQTT
            client.publish("Air_Quality_Sensor", json_data)

    else:
        print("Ket noi that bai")

if __name__ == "__main__":
    try:
        while True:
            read_sensor()
            time.sleep(60)
            
    except KeyboardInterrupt:
        # ngat ket noi modbus
        client_modbus.close()
        # ngat ket noi MQTT
        client.disconnect()
        print(" Shutting down")
       
        

