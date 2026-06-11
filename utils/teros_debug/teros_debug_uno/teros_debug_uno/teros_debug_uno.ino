#include <SoftwareSerial.h>

// ---------------------------------------------------------
// HARDWARE ROUTING
// ---------------------------------------------------------
// DEBUG_SERIAL routes to your computer via the USB cable
#define DEBUG_SERIAL Serial

// SENSOR_SERIAL routes to Pin 10 using SoftwareSerial
// RX = Pin 10 (Connect to Pico GP4)
// TX = Pin 11 (Unused, we are only listening)
SoftwareSerial SENSOR_SERIAL(10, 11);

char buffer[128];
int buf_idx = 0;

void setup() {
  // 1. Initialize USB Serial Monitor so you can see the output
  DEBUG_SERIAL.begin(115200);
  
  // Wait for the Serial Monitor to open
  while (!DEBUG_SERIAL) {
    delay(10);
  }
  DEBUG_SERIAL.println("System ready. Keeping Serial Monitor open...");
  DEBUG_SERIAL.println("Listening for TEROS broadcast on Pin 10...");

  // 2. Initialize Sensor UART on Pin 10
  // TEROS DDI uses 1200 baud, 8 data bits, no parity, 1 stop bit
  SENSOR_SERIAL.begin(1200);
}

// ---------------------------------------------------------
// PARSING LOGIC (Untouched)
// ---------------------------------------------------------
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

// ---------------------------------------------------------
// UPDATED PARSING LOGIC (Floats Removed)
// ---------------------------------------------------------
void parse_data(char* data_str) {
  // strtok breaks the string into "tokens" separated by spaces or tabs
  char* token = strtok(data_str, " \t");
  
  if (token != NULL) {
    char* vwc_str = token; // Keep as text string
    token = strtok(NULL, " \t"); // Grab the next token
    
    if (token != NULL) {
      char* temp_str = token; // Keep as text string
      token = strtok(NULL, " \t"); // Grab the last token
      
      if (token != NULL) {
        char* ec_str = token; // Keep as text string
        
        // If we successfully grabbed all three, print the strings directly!
        DEBUG_SERIAL.print("VWC Counts: "); 
        DEBUG_SERIAL.print(vwc_str);
        DEBUG_SERIAL.print(" \tTemperature: "); 
        DEBUG_SERIAL.print(temp_str);
        DEBUG_SERIAL.print(" \tElectrical Conductivity: "); 
        DEBUG_SERIAL.println(ec_str);
        return; // Success! Exit the function.
      }
    }
  }
  
  // If we make it down here, it means we didn't find all 3 numbers
  DEBUG_SERIAL.print("Error parsing data. Raw string: ");
  DEBUG_SERIAL.println(data_str);
}

// ---------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------
void loop() {
  // 1. Read incoming bytes into the buffer from Pin 10
  while (SENSOR_SERIAL.available() > 0) {
    char c = SENSOR_SERIAL.read();
    
    // THE FIX: Ignore leftover newlines or blank spaces if the buffer is empty.
    // This ensures every fresh payload strictly starts with the first real character (the '\t').
    if (buf_idx == 0 && (c == '\n' || c == '\r' || c == ' ')) {
      continue;
    }
    
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