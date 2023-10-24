from pymodbus.client import ModbusSerialClient

def decimal_to_hex(input_value):
    hex_value = hex(input_value & 0xFFFF).upper()
    if len(hex_value) < 6:
        hex_value = '0x' + '0' * (6 - len(hex_value)) + hex_value[2:]
    return hex_value

def main():
    # Connect to Modbus device
    client = ModbusSerialClient(
        method="rtu",
        port="/dev/ttyUSB0",
        baudrate=115200,
        stopbits=1,
        parity="N",
        timeout=0.2
    )
    if not client.connect():
        print("Failed to connect to Modbus device")
        return

    print("Connected to Modbus device")
    client.write_registers(342, [0x0000, 0], unit=0x01)
    client.write_register(0x7C, 0x96, unit=0x01)  # CJ
    client.write_registers(342, [0x0000, 0], unit=0x02)
    client.write_register(0x7C, 0x96, unit=0x02)  # CJ
    client.write_registers(342, [0x0000, 0], unit=0x03)
    client.write_register(0x7C, 0x96, unit=0x03)  # CJ
    try:
        while True:
            input_value = int(input("Velocity: "))
            hex_value = decimal_to_hex(input_value)

            if input_value >= 0:
                print("motor clockwise")
                client.write_registers(342, [0x0000, int(hex_value, 16)], unit=0x01)
                client.write_registers(342, [0x0000, int(hex_value, 16)], unit=0x02)
                client.write_registers(342, [0x0000, int(hex_value, 16)], unit=0x03)
            else:
                print("motor anticlockwise")
                client.write_registers(342, [0xFFFF, int(hex_value, 16)], unit=0x01)
                client.write_registers(342, [0xFFFF, int(hex_value, 16)], unit=0x02)
                client.write_registers(342, [0xFFFF, int(hex_value, 16)], unit=0x03)
                
        
    except Exception as e:
        print(f"An error occurred: {e}")

    client.close()

if __name__ == '__main__':
    main()
