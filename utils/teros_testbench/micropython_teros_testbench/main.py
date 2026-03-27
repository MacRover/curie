from machine import UART, Pin
import time

# Setup UART 1
# TX = GP4, RX = GP5 (Connect the sensor's data wire to GP5)
teros_serial = UART(1, baudrate=1200, tx=Pin(4), rx=Pin(5))

print("System ready, waiting for DDI data...")

def legacy_checksum(response_bytes):
    # Exact translation of the documentation's checksum logic
    sum_val = 0
    
    for i, b in enumerate(response_bytes):
        sum_val += b
        if b == ord('\r'):
            # Found \r, the next character is the sensor type, which is included in the sum
            if i + 1 < len(response_bytes):
                sum_val += response_bytes[i + 1]
            break
            
    # Calculate final printable character
    return (sum_val % 64) + 32

def parse_data(raw_data_str):
    # The string up to \r looks like: "\t2749.0 23.8 660"
    # .split() automatically separates the values by spaces and tabs
    try:
        parts = raw_data_str.strip().split()
        
        if len(parts) >= 3:
            vwc = float(parts[0])
            temp = float(parts[1])
            ec = float(parts[2])
            
            print(f"VWC Counts: {vwc} \tTemperature: {temp} \tElectrical Conductivity: {ec}")
    except ValueError as e:
        print(f"Error parsing data: {e}")

while True:
    time.sleep(0.2)
    
    # Check if data is waiting in the UART buffer
    if teros_serial.any():
        # Read the incoming bytes
        raw_bytes = teros_serial.readline()
        
        if raw_bytes:
            # Find the carriage return '\r'
            cr_index = raw_bytes.find(b'\r')
            
            # Make sure \r exists and the byte array is long enough to hold the checksum
            if cr_index != -1 and len(raw_bytes) >= cr_index + 3:
                
                # In DDI, checksum is exactly two characters after \r
                received_checksum = raw_bytes[cr_index + 2] 
                calculated_checksum = legacy_checksum(raw_bytes)
                
                if calculated_checksum == received_checksum:
                    # Decode only the measurement part of the bytes into a string
                    data_str = raw_bytes[:cr_index].decode('utf-8')
                    parse_data(data_str)
                else:
                    print(f"Checksum mismatch! Calc: {chr(calculated_checksum)}, Recv: {chr(received_checksum)}")
                    
            # Clear out any leftover bytes in the buffer
            while teros_serial.any():
                teros_serial.read()
                
    time.sleep(1)