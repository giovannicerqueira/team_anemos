#define analogPin A0

float VDD = 3.3;

void setup() {
  // Start the Serial communication at 9600 bits per second
  Serial.begin(9600);
}

void loop() {
  // Read the input on analog pin
  int sensorValue = analogRead(analogPin);

  float mappedVoltage = sensorValue/(1023/VDD);

  //int mappedVoltage = map(sensorValue, 0, 1023, 0, VDD);

  int DP = (750 * mappedVoltage / VDD) - 150;

  if (DP < 0) {
    DP = 0;
  }

  float windSpeed = sqrt(2 * DP / (1.204));

  // Print the value to the Serial Monitor
  Serial.print("Analog value: ");
  Serial.println(sensorValue);
  //Serial.println(mappedVoltage);
  Serial.print("Differential Pressure: ");
  Serial.println(DP);
  Serial.print("Calculated Wind Speed: ");
  Serial.println(windSpeed);

  
  // Wait a bit before the next loop
  delay(1000); // Delay in milliseconds
}
