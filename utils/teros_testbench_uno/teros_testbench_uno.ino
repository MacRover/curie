#include <SoftwareSerial.h>

// Define the serial ports
#define DEBUG_SERIAL Serial

// Choose two digital pins for SoftwareSerial (e.g., Pin 10 as RX, Pin 11 as TX)
// Connect the TEROS data line (SDI-12 / DDI) to the RX pin (Pin 10)
SoftwareSerial SENSOR_SERIAL(10, 11); 

char buffer[128];
int buf_idx = 0;

void setup() {
  // 1. Initialize USB Serial Monitor
  DEBUG_SERIAL.begin(115200);
  
  // NOTE: Removed `while(!DEBUG_SERIAL)` so the Uno doesn't hang 
  // if running on external power without the Serial Monitor open.
  
  DEBUG_SERIAL.println("System ready, waiting for DDI data from TEROS...");

  // 2. Initialize Sensor SoftwareSerial at 1200 baud
  // TEROS DDI uses 1200 baud, 8 data bits, no parity, 1 stop bit
  SENSOR_SERIAL.begin(1200);
}

// Exact translation of the checksum logic
char legacy_checksum(const char* response_bytes, int len) {
  int sum_val = 0;
  
  for (int i = 0; i < len; i++) {
    sum_val += response_bytes[i];
    if (response_bytes[i] == '\r') {
      if (i + 1 < len) {
        sum_val += response_bytes[i + 1];
      }
      break;
    }
  }
  
  return (char)((sum_val % 64) + 32);
}

void parse_data(char* data_str) {
  char* token = strtok(data_str, " \t");
  
  if (token != NULL) {
    char* vwc_str = token; 
    token = strtok(NULL, " \t"); 
    
    if (token != NULL) {
      char* temp_str = token; 
      token = strtok(NULL, " \t"); 
      
      if (token != NULL) {
        char* ec_str = token; 

        float raw_vwc = atof(vwc_str);
        float temp_c = atof(temp_str);
        float ec_val = atof(ec_str);

        float vwc_m3_m3 = (0.0003879 * raw_vwc) - 0.6956; 
        float vwc_percent = vwc_m3_m3 * 100.0; 
        float ec_us_cm = ec_val * 1000.0; 
        
        DEBUG_SERIAL.print("VWC: "); 
        DEBUG_SERIAL.print(vwc_m3_m3);
        DEBUG_SERIAL.print(" m3/m3 \tTemp: "); 
        DEBUG_SERIAL.print(temp_c);
        DEBUG_SERIAL.print(" C \tEC: "); 
        DEBUG_SERIAL.print(ec_us_cm);
        DEBUG_SERIAL.println(" uS/cm");
        return; 
      }
    }
  }

  DEBUG_SERIAL.print("Error parsing data. Raw string: ");
  DEBUG_SERIAL.println(data_str);
}

void loop() {
  while (SENSOR_SERIAL.available() > 0) {
    char c = SENSOR_SERIAL.read();

    if (buf_idx == 0 && (c == '\n' || c == '\r' || c == ' ')) {
      continue;
    }
    
    if (buf_idx < sizeof(buffer) - 1) {
      buffer[buf_idx++] = c;
    }
  }

  int cr_index = -1;
  for (int i = 0; i < buf_idx; i++) {
    if (buffer[i] == '\r') {
      cr_index = i;
      break;
    }
  }

  if (cr_index != -1 && buf_idx >= cr_index + 3) {
    char received_checksum = buffer[cr_index + 2];
    char calculated_checksum = legacy_checksum(buffer, buf_idx);

    if (calculated_checksum == received_checksum) {
      buffer[cr_index] = '\0';
      parse_data(buffer);
    } else {
      DEBUG_SERIAL.print("Checksum mismatch! Calc: ");
      DEBUG_SERIAL.print(calculated_checksum);
      DEBUG_SERIAL.print(", Recv: ");
      DEBUG_SERIAL.println(received_checksum);
    }

    buf_idx = 0;
  }
  
  if (buf_idx >= sizeof(buffer) - 1) {
    DEBUG_SERIAL.println("Buffer full, clearing.");
    buf_idx = 0; 
  }
}