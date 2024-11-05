// Input wind speed reading
const int INPUT_PIN = A4;
// Output to PWM Fan
const int OUTPUT_PIN = D4;

double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 75.00;

void setup()
{
  // Tuning parameters for Proportional, Integral,
  // and Derivative Components Respectively
  kp = 0.8;
  ki = 0.20;
  kd = 0.001;

  last_time = 0;
  Serial.begin(9600);
  analogWrite(OUTPUT_PIN, 0);
  for(int i = 0; i < 50; i++)
  {
    // Prints the setpoint to the Serial plotter 
    // for 5 seconds as an initialization state.
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }
  delay(100);
}

void loop()
{
  // Calculate dt for the derivative
  double now = millis();
  dt = (now - last_time)/1000.00;
  // Updated the last_time as the current time
  last_time = now;

  // Take Input signal and calculate error
  double actual = map(analogRead(INPUT_PIN), 0, 1024, 0, 255);
  double error = setpoint - actual;

  // Calculate what output to give to the fan
  output = pid(error);

  // Send the output
  analogWrite(OUTPUT_PIN, output);

  // Setpoint VS Actual
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(actual);

  // Error
  //Serial.println(error);

  delay(300);
}

double pid(double error)
{
  // Adjust simply proportional to error
  double proportional = error;

  // Compute the integral with a Riemann Approximation
  // Area = b * h = error * time_interval
  integral += error * dt;

  // Compute the derivative as the slope between the 
  // current error and the previous error term.
  double derivative = (error - previous) / dt;
  previous = error;

  // Scale each P, I, and D term by its weight
  // THIS WEIGHTS WILL NEED TO BE TUNED!
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);

  
  return output;
}




