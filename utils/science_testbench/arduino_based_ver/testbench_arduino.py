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

# ---------------- UART Setup ----------------
PORT = "COM11"  # Change this to your COM port
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=1)

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
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line:
            try:
                values = [float(v) for v in line.split(" ")]
                if len(values) == 4:
                    a, b, c, d = values
                    temp.append(a)
                    hum.append(b)
                    ph.append(c)
                    ec.append(d)
                    now = time.time() - start
                    x.append(now)
                    print("Temperature: " + str(a) + " | Humidity: " + str(b) + " | pH: " + str(c) + " | Electrical Conductivity: " + str(d))

                    with open(csv_file, mode="a", newline = "") as f:
                        writer = csv.writer(f)
                        writer.writerow([now, a, b, c, d])


            except ValueError:
                pass  # ignore malformed lines

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
