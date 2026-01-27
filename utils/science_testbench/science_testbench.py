import serial  # pip install pyserial
import time
import keyboard  # pip install keyboard
import matplotlib.pyplot as plt
import datetime
import csv

"""

Press t to see temperature graph
Press h to see humidity graph
Press p to see pH graph
press e to see ec graph


"""


# Functions from Arduino

def crc16_2(buf: bytes) -> int:
    crc = 0xFFFF
    for b in buf:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1

    # byte swap (same as Arduino)
    crc = ((crc & 0x00FF) << 8) | ((crc & 0xFF00) >> 8)
    return crc

import time

def read_n(ser, length, timeout_ms=500) -> bytes:
    data = bytearray()
    start = time.time()

    while len(data) < length:
        if ser.in_waiting:
            data += ser.read(1)
        if (time.time() - start) * 1000 > timeout_ms:
            break

    return bytes(data)

def read_humiture_ecph(ser, command: bytes):
    """
    ser     : opened serial.Serial object
    command : 8-byte command frame (your Com array)
    """

    while True:
        time.sleep(0.1)

        # Send command
        ser.write(command)
        ser.flush()

        # Expect header 01 03 08
        ch = read_n(ser, 1)
        if len(ch) != 1 or ch[0] != 0x01:
            continue

        ch = read_n(ser, 1)
        if len(ch) != 1 or ch[0] != 0x03:
            continue

        ch = read_n(ser, 1)
        if len(ch) != 1 or ch[0] != 0x08:
            continue

        # Read remaining bytes
        rest = read_n(ser, 10)
        if len(rest) != 10:
            continue

        data = bytes([0x01, 0x03, 0x08]) + rest

        # CRC check
        crc_calc = crc16_2(data[:11])
        crc_recv = (data[11] << 8) | data[12]

        if crc_calc != crc_recv:
            continue

        # Parse values
        hum = (data[3] << 8 | data[4]) / 10.0
        tem = (data[5] << 8 | data[6]) / 10.0
        ec  = (data[7] << 8 | data[8])
        ph  = (data[9] << 8 | data[10]) / 10.0

        return hum, tem, ec, ph


# ---------------- UART Setup ----------------
PORT = "COM11"  # Change this to your COM port
BAUD = 9600
BYTESIZE = 8
PARITY = serial.PARITY_NONE
STOPBITS = 1
TIMEOUT = 0
ser = serial.Serial(port = PORT, baud = BAUD, bytesize = BYTESIZE, party = PARITY, stopbits = STOPBITS, timeout = TIMEOUT)

# Original 8 byte command

Com = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09])
# ---------------- Data storage ----------------
temp = []
hum = []
ph = []
ec = []
x = []

start = time.time()
last_plot = 0  # For key debounce

csv_file = "data_" + str(datetime.datetime.now()).replace(" ", "_").replace(":", ".") + ".csv"

with open(csv_file, mode="a", newline = "") as f:
    writer = csv.writer(f)
    writer.writerow(["Time (s)", "Temperature", "Humidity", "pH", "Electrical Conductivity"])

print("Listening... press 'a' to plot temperature snapshot.")

try:
    while True:
        # ---------- Read UART ----------
        # line = ser.readline().decode("utf-8", errors="ignore").strip()
        # if line:
        #     try:
        #         values = [float(v) for v in line.split(" ")]
        #         if len(values) == 4:
        #             a, b, c, d = values
        #             temp.append(a)
        #             hum.append(b)
        #             ph.append(c)
        #             ec.append(d)
        #             now = time.time() - start
        #             x.append(now)
        #             print("Temperature: " + str(a) + " | Humidity: " + str(b) + " | pH: " + str(c) + " | Electrical Conductivity: " + str(d))

                    # with open(csv_file, mode="a", newline = "") as f:
                    #     writer = csv.writer(f)
                    #     writer.writerow([now, a, b, c, d])


        #     except ValueError:
        #         pass  # ignore malformed lines

        hum_val, temp_val, ec_val, ph_val = read_humiture_ecph(ser, Com)
        print("Temperature: " + str(temp_val) + " | Humidity: " + str(hum_val) + " | pH: " + str(ec_val) + " | Electrical Conductivity: " + str(ph_val))

        # Append it all to the lists

        temp.append(temp_val)
        hum.append(hum_val)
        ec.append(ec_val)
        ph.append(ph_val)
        now = time.time() - start
        x.append(now)

        with open(csv_file, mode="a", newline = "") as f:
            writer = csv.writer(f)
            writer.writerow([now, temp_val, hum_val, ph_val, ec_val])


        # ---------- Check for 't' press ----------
        if keyboard.is_pressed("t") and time.time() - last_plot > 0.5:
            last_plot = time.time()
            # Pause main loop and show snapshot plot
            plt.figure()
            plt.plot(x, temp, color='red', lw=2)
            plt.xlabel("Time (s)")
            plt.ylabel("Temperature")
            plt.title("Temperature vs Time")
            plt.grid(True)
            plt.show()  # Blocks here until window closed
            print("Plot closed, resuming data collection...")

        if keyboard.is_pressed("h") and time.time() - last_plot > 0.5:
            last_plot = time.time()
            # Pause main loop and show snapshot plot
            plt.figure()
            plt.plot(x, hum, color='blue', lw=2)
            plt.xlabel("Time (s)")
            plt.ylabel("Humidity")
            plt.title("Humidity vs Time")
            plt.grid(True)
            plt.show()  # Blocks here until window closed
            print("Plot closed, resuming data collection...")

        if keyboard.is_pressed("p") and time.time() - last_plot > 0.5:
            last_plot = time.time()
            # Pause main loop and show snapshot plot
            plt.figure()
            plt.plot(x, ph, color='magenta', lw=2)
            plt.xlabel("Time (s)")
            plt.ylabel("pH")
            plt.title("pH vs Time")
            plt.grid(True)
            plt.show()  # Blocks here until window closed
            print("Plot closed, resuming data collection...")

        if keyboard.is_pressed("e") and time.time() - last_plot > 0.5:
            last_plot = time.time()
            # Pause main loop and show snapshot plot
            plt.figure()
            plt.plot(x, ec, color='orange', lw=2)
            plt.xlabel("Time (s)")
            plt.ylabel("Electrical Conductivity")
            plt.title("Electrical Conductivity vs Time")
            plt.grid(True)
            plt.show()  # Blocks here until window closed
            print("Plot closed, resuming data collection...")

except KeyboardInterrupt:
    print("Exiting...")
    ser.close()



