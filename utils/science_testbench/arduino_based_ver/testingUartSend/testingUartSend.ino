void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0)); // seed RNG with noise
}

void loop() {
  float tem;
  float hum;
  float ph;
  int ec;

  tem = random(0, 10000) / 100.0;
  hum = random(0, 10000) / 100.0;
  ph = random(0, 10000) / 100.0;
  ec = random(0, 10000) / 100.0;

  // Send over UART

  Serial.print(tem);
  Serial.print(" ");
  Serial.print(hum);
  Serial.print(" ");
  Serial.print(ph);
  Serial.print(" ");
  Serial.print(ec);
  Serial.println(); // newline = end of packet

  delay(500);

}
