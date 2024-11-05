// Define the pin for the LED
const int fanOutput = 4; // PWM pin (usually 3, 5, 6, 9, 10, or 11 on most Arduino boards)

void setup() {
  // Set the LED pin as an output
  Serial.begin(9600);
  pinMode(fanOutput, OUTPUT);
}

void loop() {
  // Fade the LED from off to full brightness
  for (int brightness = 0; brightness <= 63; brightness++) {
    analogWrite(fanOutput, brightness);  // Set the LED brightness using PWM
    Serial.println(brightness);
    delay(20);  // Delay to create a smooth fade
  }

  // Fade the LED from full brightness to off
  for (int brightness = 63; brightness >= 0; brightness--) {
    analogWrite(fanOutput, brightness);  // Set the LED brightness using PWM
    delay(20);  // Delay to create a smooth fade
  }
}
