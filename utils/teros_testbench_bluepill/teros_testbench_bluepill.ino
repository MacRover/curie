// Define the serial ports
// Serial (USB CDC) is for the Arduino Serial Monitor
// Serial2 (PA2 TX, PA3 RX) is for the TEROS sensor
#define DEBUG_SERIAL Serial
#define SENSOR_SERIAL Serial2

HardwareSerial Serial2(PA3, PA2);

char buffer[128];
int buf_idx = 0;

void setup() {
  // 1. Initialize USB Serial Monitor
  DEBUG_SERIAL.begin(115200);
  
  // Wait for the Serial Monitor to be opened before continuing
  while (!DEBUG_SERIAL) {
    delay(10);
  }
  DEBUG_SERIAL.println("System ready, waiting for DDI data from TEROS...");

  // 2. Initialize Sensor UART
  // TEROS DDI uses 1200 baud, 8 data bits, no parity, 1 stop bit
  SENSOR_SERIAL.begin(1200);
}

// Exact translation of the MicroPython checksum logic
char legacy_checksum(const char* response_bytes, int len) {
  int sum_val = 0;
  
  for (int i = 0; i < len; i++) {
    sum_val += response_bytes[i];
    if (response_bytes[i] == '\r') {
      // Found \r, add the next character (sensor type) to the sum
      if (i + 1 < len) {
        sum_val += response_bytes[i + 1];
      }
      break;
    }
  }
  
  // Calculate final printable character
  return (char)((sum_val % 64) + 32);
}

void parse_data(const char* data_str) {
  float vwc, temp, ec;
  
  // sscanf automatically skips whitespace and tabs to find the numbers
  int parsed = sscanf(data_str, "%f %f %f", &vwc, &temp, &ec);
  
  if (parsed == 3) {
    DEBUG_SERIAL.print("VWC Counts: "); 
    DEBUG_SERIAL.print(vwc);
    DEBUG_SERIAL.print(" \tTemperature: "); 
    DEBUG_SERIAL.print(temp);
    DEBUG_SERIAL.print(" \tElectrical Conductivity: "); 
    DEBUG_SERIAL.println(ec);
  } else {
    DEBUG_SERIAL.print("Error parsing data. Raw string: ");
    DEBUG_SERIAL.println(data_str);
  }
}

void loop() {
  // 1. Read incoming bytes into the buffer
  while (SENSOR_SERIAL.available() > 0) {
    char c = SENSOR_SERIAL.read();
    
    // Prevent buffer overflow
    if (buf_idx < sizeof(buffer) - 1) {
      buffer[buf_idx++] = c;
    }
  }

  // 2. Check if we have received a full message
  int cr_index = -1;
  for (int i = 0; i < buf_idx; i++) {
    if (buffer[i] == '\r') {
      cr_index = i;
      break;
    }
  }

  // Ensure \r exists and we have the 2 bytes that follow it (sensor type + checksum)
  if (cr_index != -1 && buf_idx >= cr_index + 3) {
    char received_checksum = buffer[cr_index + 2];
    char calculated_checksum = legacy_checksum(buffer, buf_idx);

    if (calculated_checksum == received_checksum) {
      // Isolate the measurement data by replacing \r with a null terminator
      buffer[cr_index] = '\0';
      parse_data(buffer);
    } else {
      DEBUG_SERIAL.print("Checksum mismatch! Calc: ");
      DEBUG_SERIAL.print(calculated_checksum);
      DEBUG_SERIAL.print(", Recv: ");
      DEBUG_SERIAL.println(received_checksum);
    }

    // Reset buffer for the next reading
    buf_idx = 0;
  }
  
  // Safety net: flush buffer if it gets full without finding a valid string
  if (buf_idx >= sizeof(buffer) - 1) {
    DEBUG_SERIAL.println("Buffer full, clearing. Is the sensor wired correctly?");
    buf_idx = 0; 
  }
}