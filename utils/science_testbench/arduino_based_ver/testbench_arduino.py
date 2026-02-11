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

Press o to change an offset
Press s to change a scale factor

"""

# CHANGE THE NUMBER OF MOVING AVERAGE POINTS HERE

MOVING_AVERAGE_POINTS = 20

# First time constants (you can also change these to change offsets)

TEMP_OFFSET_BASE = 0
HUMIDITY_OFFSET_BASE = 0
PH_OFFSET_BASE = 0
EC_OFFSET_BASE = 0

TEMP_SCALING_BASE = 1
HUMIDITY_SCALING_BASE = 1
PH_SCALING_BASE = 1
EC_SCALING_BASE = 1



temp_offset = TEMP_OFFSET_BASE
ph_offset = PH_OFFSET_BASE
humidity_offset = HUMIDITY_OFFSET_BASE
ec_offset = EC_OFFSET_BASE

temp_scaling = TEMP_SCALING_BASE
ph_scaling = PH_SCALING_BASE
humidity_scaling = HUMIDITY_SCALING_BASE
ec_scaling = EC_SCALING_BASE


def change_offset(temp_offset, humidity_offset, ph_offset, ec_offset):

    print("Enter a command")
    print("1: Change temperature offset")
    print("2: Change humidity offset")
    print("3: Change pH offset")
    print("4: Change electrical conductivity offset")
        

    cmd = int(input(""))


    if (cmd == 1):
        temp_offset = float(input("enter value: "))

    elif (cmd == 2):
        humidity_offset = float(input("enter value: "))

    elif (cmd == 3):
        ph_offset = float(input("enter value: "))

    elif (cmd == 4):
        ec_offset = float(input("enter value: "))

    return temp_offset, humidity_offset, ph_offset, ec_offset
    



def change_scaling(temp_scaling, humidity_scaling, ph_scaling, ec_scaling):

    print("Enter a command")
    print("1: Change temperature scaling")
    print("2: Change humidity scaling")
    print("3: Change pH scaling")
    print("4: Change electrical conductivity scaling")
    

    cmd = int(input(""))

    if (cmd == 1):
        temp_scaling = float(input("enter value: "))

    elif (cmd == 2):
        humidity_scaling = float(input("enter value: "))

    elif (cmd == 3):
        ph_scaling = float(input("enter value: "))

    elif (cmd == 4):
        ec_scaling = float(input("enter value: "))

    return temp_scaling, humidity_scaling, ph_scaling, ec_scaling


def offset_and_scale(scale, offset, value):

    return scale*value + offset


def moving_average(array, window):

    # Calculates the moving average very straightforwardly

    if (len(array)) < window:
        return sum(array) / len(array)
    else:
        return sum(array[-window:]) / window # Return the last window points




# OFFSET PROGRAMMING

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

                    # print("Stock values:")
                    # print("Temperature: " + str(a) + " | Humidity: " + str(b) + " | pH: " + str(c) + " | Electrical Conductivity: " + str(d))


                    a = offset_and_scale(temp_scaling, temp_offset, a)
                    b = offset_and_scale(humidity_scaling, humidity_offset, b)
                    c = offset_and_scale(ph_scaling, ph_offset, c)
                    d = offset_and_scale(ec_scaling, ec_offset, d)

                    # print("Offsets")
                    # print("Temperature: " + str(temp_scaling) + " | Humidity: " + str(humidity_scaling) + " | pH: " + str(ph_scaling) + " | Electrical Conductivity: " + str(ec_scaling))


                    temp.append(a)
                    hum.append(b)
                    ph.append(c)
                    ec.append(d)
                    now = time.time() - start
                    x.append(now)

                    # print("Offsetted Vaues")

                    print("Values: ")
                    print("Temperature: " + str(a) + " | Humidity: " + str(b) + " | pH: " + str(c) + " | Electrical Conductivity: " + str(d))


                    print("Moving averages:")

                    avg_temp = moving_average(temp, MOVING_AVERAGE_POINTS)
                    avg_hum = moving_average(hum, MOVING_AVERAGE_POINTS)
                    avg_ph = moving_average(ph, MOVING_AVERAGE_POINTS)
                    avg_ec = moving_average(ec, MOVING_AVERAGE_POINTS)

                    print("Temperature: " + str(avg_temp) + " | Humidity: " + str(avg_hum) + " | pH: " + str(avg_ph) + " | Electrical Conductivity: " + str(avg_ec))



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


        if keyboard.is_pressed("o") and time.time() - last_plot > 0.5:
            temp_offset, humidity_offset, ph_offset, ec_offset = change_offset(temp_offset, humidity_offset, ph_offset, ec_offset)

        if keyboard.is_pressed("s") and time.time() - last_plot > 0.5:
            temp_scaling, humidity_scaling, ph_scaling, ec_scaling = change_scaling(temp_scaling, humidity_scaling, ph_scaling, ec_scaling)

        

except KeyboardInterrupt:
    print("Exiting...")
    ser.close()
