from pymodbus.client.sync import ModbusSerialClient
import time

def decimal_to_hex(input_value):
        hex_value = hex(input_value & 0xFFFF).upper()
        if len(hex_value) < 6:
            hex_value = '0x' + '0' * (6 - len(hex_value)) + hex_value[2:]
        return hex_value

def main():
    # Connect to Modbus device
    client = ModbusSerialClient(
        method="rtu",
        port="COM7",
        baudrate=19200,
        stopbits=1,
        parity="N",
        timeout=0.2
    )
    if not client.connect():
        print("Failed to connect to Modbus device")
        
        return

    print("Connected to Modbus device")
    client.write_register(0x7C, 0x9F, unit=0x03)            #ME
    client.write_register(0x0152, 0x0258, unit=0x03)        #JA = 600
    client.write_register(0x0154, 0x0258, unit=0x03)        #JL = 600
    input_value = int(input("Velocity: "))
    hex_value = decimal_to_hex(input_value)
    try:
        if input_value >= 0:
            client.write_registers(342, [0x0000, int(hex_value, 16)], unit=0x03)
            print("motor clockwise")
            client.write_register(0x7C, 0x96, unit=0x03)         #CJ
            time.sleep(5)
            client.write_register(0x7C, 0xD8, unit=0x03)         #SM
        else:
            client.write_registers(342, [0xFFFF, int(hex_value, 16)], unit=0x03)
            print("motor anticlockwise")
            client.write_register(0x7C, 0x96, unit=0x03)         #CJ
            time.sleep(5)
            client.write_register(0x7C, 0xD8, unit=0x03)         #SM
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()

