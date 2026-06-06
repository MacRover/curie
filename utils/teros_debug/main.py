from machine import UART, Pin
import random
import time

# Setup UART to BROADCAST on GP4 (TX).
# Baudrate is 1200 to perfectly match the TEROS 12 DDI protocol.
teros_simulator = UART(1, baudrate=1200, tx=Pin(4))

def legacy_checksum(response_bytes):
    sum_val = 0
    for i, b in enumerate(response_bytes):
        sum_val += b
        if b == ord('\r'):
            if i + 1 < len(response_bytes):
                sum_val += response_bytes[i + 1]
            break
    return (sum_val % 64) + 32

def generate_random_teros_bytes(sensor_type=b'z'):
    # Generate semi-realistic soil values
    vwc = round(random.uniform(1500.0, 3500.0), 1)
    temp = round(random.uniform(10.0, 35.0), 1)
    ec = round(random.uniform(100.0, 900.0), 0)
    
    # Pack into the DDI format: \t [VWC] [Temp] [EC] \r [Type] [Checksum] \n
    payload = f"\t{vwc} {temp} {ec}".encode('utf-8')
    payload_with_cr = payload + b'\r' + sensor_type
    valid_checksum = legacy_checksum(payload_with_cr)
    
    return payload_with_cr + bytes([valid_checksum]) + b'\n'

print("Pico TEROS Simulator Active.")
print("Broadcasting DDI data on GP4 at 1200 baud...")

while True:
    raw_bytes = generate_random_teros_bytes()
    teros_simulator.write(raw_bytes)
    
    # Print to the Pico's console so you can verify what was sent
    print(f"Sent to A5: {raw_bytes}")
    
    # 2-second delay to give the Arduino plenty of time to parse
    time.sleep(2)