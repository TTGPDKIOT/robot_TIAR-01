from pymodbus.client.sync import ModbusSerialClient
import time

def main():
    # Connect to Modbus device
    client = ModbusSerialClient(
        method="rtu",
        port="COM3",
        baudrate=19200,
        stopbits=1,
        parity="N",
        timeout=0.2
    )
    if not client.connect():
        print("Failed to connect to Modbus device")
        
        return

    print("Connected to Modbus device")

    try:
        print("motor clockwise")
        client.write_register(0x7C, 0x9F, unit=0x03)            #ME
        client.write_register(0x0152, 0x0258, unit=0x03)        #JA = 600
        client.write_register(0x0154, 0x0258, unit=0x03)        #JL = 600
        client.write_register(342, 0, unit=0x03)                #JS = 0
        client.write_register(343, 4800, unit=0x03)             #JS = 4800
        client.write_register(0x7C, 0x96, unit=0x03)            #CJ
        time.sleep(5)
        client.write_register(0x7C, 0xD8, unit=0x03)            #SM
        
        print("motor anticlockwise")
        # client.write_register(342, 0xFFFF, unit=0x03)           #JS = -1
        # client.write_register(343, 0xED40, unit=0x03)           #JS = -2400
        client.write_registers(342, [0xFFFF, 0xED40], unit=0x03)#Wrte 2 registers
        client.write_register(0x7C, 0x96, unit=0x03)            #CJ
        time.sleep(5)
        client.write_register(0x7C, 0xD8, unit=0x03)            #SM
        client.write_register(0x7C, 0x9E, unit=0x03)            #MD
        
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()

