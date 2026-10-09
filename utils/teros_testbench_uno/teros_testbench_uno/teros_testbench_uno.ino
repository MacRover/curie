#include <SoftwareSerial.h>

#define DEBUG_SERIAL Serial
SoftwareSerial SENSOR_SERIAL(10, 11); 

char buffer[128];
int buf_idx = 0;

void setup() {
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("System ready, waiting for DDI data from TEROS...");
  SENSOR_SERIAL.begin(1200);
}

char legacy_checksum(const char* response_bytes, int len) {
  int sum_val = 0;
  
  for (int i = 0; i < len; i++) {
    sum_val += (uint8_t)response_bytes[i]; 
    if (response_bytes[i] == '\r') {
      if (i + 1 < len) {
        sum_val += (uint8_t)response_bytes[i + 1];
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
        
        DEBUG_SERIAL.print("VWC Counts: "); 
        DEBUG_SERIAL.print(vwc_m3_m3);
        DEBUG_SERIAL.print(" \tTemperature: "); 
        DEBUG_SERIAL.print(temp_c);
        DEBUG_SERIAL.print(" \tElectrical Conductivity: "); 
        DEBUG_SERIAL.println(ec_us_cm);
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

    if (c == '\t') {
      buf_idx = 0;
    }

    if (buf_idx == 0) {
      if (c != '\t' && !(c >= '0' && c <= '9')) {
        continue; 
      }
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
    
    // THE NEW FIX: Only process the checksum if the string is long enough
    // to be real sensor data. This silently filters out the short phantom 
    // bytes caused by the sensor going to sleep.
    if (cr_index >= 10) {
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
    }

    // Reset buffer for the next incoming reading
    buf_idx = 0;
  }
  
  if (buf_idx >= sizeof(buffer) - 1) {
    DEBUG_SERIAL.println("Buffer full, clearing. Is the sensor wired correctly?");
    buf_idx = 0; 
  }
}