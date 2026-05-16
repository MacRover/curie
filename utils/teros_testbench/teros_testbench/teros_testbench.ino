#include <SoftwareSerial.h>

#define RX_PIN 10
#define TX_PIN 11

SoftwareSerial terosSerial(RX_PIN, TX_PIN);

// CONNECT THE SENSOR TO 5V IT SHOULD BE FINE, might need a switch need to see if it only does DDI at power cycle

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  terosSerial.begin(1200);
  Serial.println("System ready, requesting data");





}

uint8_t legacyChecksum(const char * response) { 

  // Exact copy from documentation

  uint16_t length;
  uint16_t i;
  uint16_t sum = 0;

  length = strlen(response);

  // add characters

  for (i = 0; i < length; i++) {
    sum += response[i];
    if (response[i] == '\r') {
      break;
    }
  }

  sum += response[++i];
  sum = sum % 64 + 32;

  return sum;
}
  

void loop() {
  // put your main code here, to run repeatedly:


  delay(200);

  if (terosSerial.available()) {

    // Read the DDI response from teros

    String rawData = terosSerial.readStringUntil('\n');

    if (rawData.length() > 0) {
      char buf[64];
      rawData.toCharArray(buf, 64);

      int crIndex = rawData.indexOf('\r'); // finding /r

      // make sure it exists
      if (crIndex != -1 && rawData.length() >= crIndex + 3) {

        char receivedChecksum = rawData.charAt(crIndex + 2); // checksum is two chars after \r

        uint8_t calculated = legacyChecksum(buf);

        // Final check to make sure that the legacy checksum matches the checksum we receive parsed to an int

        if (calculated == (uint8_t) receivedChecksum) {
          parseData(rawData);

          // If this doesn't work there's a mismatched checksum, I don't want to print all of these cases
        }

      }

      while(terosSerial.available()) {
        terosSerial.read();
      }


    }
  }

  delay(1000);

}

void parseData(String data) {

  int firstSpace = data.indexOf(' ');
  int secondSpace = data.indexOf(' ', 1 + firstSpace);
  int crIndex = data.indexOf('\r');

  String vwcRaw = data.substring(1,firstSpace);
  String tempRaw = data.substring(firstSpace + 1, secondSpace);
  String ecRaw = data.substring(secondSpace + 1, crIndex);

  // Float conversions

  float vwc = vwcRaw.toFloat();
  float temp = tempRaw.toFloat();
  float ec = ecRaw.toFloat();
  
  // No scaling yet, want to see if this works first

  Serial.print("VWC Counts: ");
  Serial.print(vwc);

  Serial.print("\tTemperature: ");
  Serial.print(temp);

  Serial.print("\tElectrical Conductivity: ");
  Serial.print(ec);


}
