import random
import time

# --- YOUR EXACT CHECKSUM LOGIC ---
def legacy_checksum(response_bytes):
    sum_val = 0
    for i, b in enumerate(response_bytes):
        sum_val += b
        if b == ord('\r'):
            if i + 1 < len(response_bytes):
                sum_val += response_bytes[i + 1]
            break
    return (sum_val % 64) + 32

def parse_data(raw_data_str):
    try:
        parts = raw_data_str.strip().split()
        if len(parts) >= 3:
            vwc = float(parts[0])
            temp = float(parts[1])
            ec = float(parts[2])
            print(f"Parsed -> VWC: {vwc} | Temp: {temp} | EC: {ec}")
    except ValueError as e:
        print(f"Error parsing data: {e}")

# --- INFINITE RANDOM DATA GENERATOR ---
def generate_random_teros_bytes(sensor_type=b'z'):
    """Generates a random, valid TEROS byte string."""
    # Generate semi-realistic random values
    vwc = round(random.uniform(1500.0, 3500.0), 1)
    temp = round(random.uniform(10.0, 35.0), 1)
    ec = round(random.uniform(100.0, 900.0), 0)
    
    # 1. Format the measurements
    payload = f"\t{vwc} {temp} {ec}".encode('utf-8')
    
    # 2. Append carriage return and sensor type character
    payload_with_cr = payload + b'\r' + sensor_type
    
    # 3. Calculate checksum using your function
    valid_checksum = legacy_checksum(payload_with_cr)
    
    # 4. Pack it all together
    return payload_with_cr + bytes([valid_checksum]) + b'\n'

# --- THE CONTINUOUS TEST LOOP ---
if __name__ == "__main__":
    print("Starting infinite random TEROS data stream...")
    print("Press Ctrl+C to stop.\n" + "-" * 50)
    
    try:
        while True:
            # 1. Generate new random bytes
            raw_bytes = generate_random_teros_bytes()
            print(f"\nRaw UART incoming: {raw_bytes}")
            
            # 2. Feed it into your exact parsing routine
            if raw_bytes:
                cr_index = raw_bytes.find(b'\r')
                
                if cr_index != -1 and len(raw_bytes) >= cr_index + 3:
                    received_checksum = raw_bytes[cr_index + 2] 
                    calculated_checksum = legacy_checksum(raw_bytes)
                    
                    if calculated_checksum == received_checksum:
                        data_str = raw_bytes[:cr_index].decode('utf-8')
                        parse_data(data_str)
                        print("Status: ✅ Checksum Matched")
                    else:
                        print(f"Status: ❌ Mismatch! Calc: {chr(calculated_checksum)}, Recv: {chr(received_checksum)}")
            
            time.sleep(1) # Wait a second before the next reading
            
    except KeyboardInterrupt:
        print("\n\nTest loop terminated by user.")