#include"PWM_CTRL.h"
#include"PID_FUNCTION.h"
#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"

Arduino_GigaDisplayTouch touchDetector;
GigaDisplay_GFX display; // create the object

#define BLACK 0x0000

int desired_speed = 0;
//
int trigger_sensitivity = 5;
bool switch_1;
int counter;


// Input reading from the differential pressure sensor/pitot tube combo
const int diffPressurePin = A4;
// Output to PWM Fan
const int fanCtrlPin = D4;

double dt, last_time;
double kp, ki, kd;
double setpoint = 0.0;

double VDD = 3.3;


void setup() {
  // Tuning parameters for Proportional, Integral,
  // and Derivative Components Respectively
  kp = 0.8;
  ki = 0.20;
  kd = 0.001;

  last_time = 0;

  Serial.begin(9600);

  display.begin();

  if (touchDetector.begin()) {
    //Serial.print("Touch controller init - OK");
  } else {
    //Serial.print("Touch controller init - FAILED");
    while (1)
      ;
  }

  PWM_INIT(fanCtrlPin);

  delay(5000);
}

void loop() {
  uint8_t contacts;
  GDTpoint_t points[5];
  contacts = touchDetector.getTouchPoints(points);

  /*
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  SENSOR INPUT PROCESSING SECTION:
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  */


  int sensorValue = analogRead(diffPressurePin);

  double mappedVoltage = sensorValue/(1023/VDD);

  int DP = (750 * mappedVoltage / VDD) - 150;

  if (DP < 0) {
    DP = 0;
  }
  
  float windSpeed = sqrt(2 * DP / (1.204));
  // Calibration here:
  windSpeed = (1.25 * windSpeed) + 0.897;

  // Print the value to the Serial Monitor
  /*
  Serial.print("Analog value: ");
  Serial.println(sensorValue);
  //Serial.println(mappedVoltage);
  Serial.print("Differential Pressure: ");
  Serial.println(DP);
  Serial.print("Calculated Wind Speed: ");
  Serial.println(windSpeed);
  */


  /*
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  SCREEN PROCESSING SECTION:

  *Essentially just want to process input to create a setpoint
  for the PID control.
  *Also update screen data like windspeed, pressure, etc
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  */

  if (contacts == 1) {
      desired_speed++;
  }
  else if(contacts == 2){
    desired_speed--;
  }

  display.print("Analog value:  ");
  display.print(sensorValue);
  display.print("             ");
  display.print("Differential    Pressure: ");
  display.print(DP);
  display.print("     ");
  display.print("Calculated Wind Speed: ");
  display.print(windSpeed);
  display.print("  ");
  display.print("   Desired Speed:");
  display.print(desired_speed);
  delay (500);
  display.fillScreen(BLACK);
  display.setCursor(10,10); //x,y
  display.setTextSize(5);

  setpoint = desired_speed;

  /*
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  PID CONTROL SECTION:
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  */

  // Calculate dt for the derivative
  double now = millis();
  dt = (now - last_time)/1000.00;
  // Updated the last_time as the current time
  last_time = now;

  // Take Input signal and calculate error
  double actual = analogRead(diffPressurePin);
  double error = setpoint - actual;

  // Calculate what output to give to the fan
  output = pid(error, dt, previous, kp, ki, kd);

  // Send the output
  analogWrite(fanCtrlPin, output);

  // Setpoint VS Actual
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(actual);

  // Error
  //Serial.println(error);

  delay(300);
}

