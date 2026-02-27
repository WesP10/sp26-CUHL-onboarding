#include <Wire.h>
#include "Adafruit_VL6180X.h"
 
Adafruit_VL6180X vl = Adafruit_VL6180X();
int pwm_ctrl_pin = 9;
//Intial duty cycle [0, 255], 0 corresponds to 0% and 255 to 100%
int duty_cycle = 255;
//Amount the duty cycle will increase or decrease by when the distance exceeds the threshold
int duty_incr = 5;
// Target distance for stability (e.g., mid-range of acceptable values)
float target_distance = 17.0;
// VL6180X_ALS_GAIN_5: Specifies the gain setting for the light measurement.
// Gain helps amplify the sensor's readings depending on the intensity of light (e.g., low-light environments may require higher gain).
// VL6180X_ALS_GAIN_1 (no amplification),
// VL6180X_ALS_GAIN_2 (2x amplification), etc.
float amp = VL6180X_ALS_GAIN_5;
 
float Kp = 2.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 1.0;  // Derivative gain
 
float previous_error = 0;  // Stores previous error for derivative calculation
float integral = 0;        // Accumulates error for integral term
 
void setup() {
  Serial.begin(115200);
  // wait for serial port to open on native usb devices
  Serial.println("Attempting to initialize VL6180X sensor...");
  if (!vl.begin()) {
    Serial.println("Sensor initialization failed!");
    while(1);
  }
  Serial.println("Sensor initialized successfully!");
}
 
void loop() {
  //Get the lumens per square meter of the sensor
  float lux = vl.readLux(amp);
  Serial.print("Lux: "); Serial.println(lux);
   
  //Distance reading in mm
  uint8_t range = vl.readRange();
  //Status report of sensor
  uint8_t status = vl.readRangeStatus();
  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Range: "); Serial.println(range);
    updateDutyCycle(range);
    analogWrite(pwm_ctrl_pin, duty_cycle);
  } else {
    printErr(status);
  }
 
  //VL6180x max frequency is 4kHz or 400,00 readings a second so the delay can be lowered or even removed
  delay(500);
}
 
void updateDutyCycle(uint8_t distance) {
  if (distance > target_distance + 2) {
    // Increase duty cycle if the distance is above 7mm
    duty_cycle += duty_incr;  // Adjust increment value as needed
    if (duty_cycle > 255) {
      duty_cycle = 255;
    }
    Serial.print("Duty cycle increased");
  } else if (distance < target_distance - 2) {
    // Decrease duty cycle if the distance is below 3mm
    duty_cycle -= duty_incr;  // Adjust decrement value as needed
    if (duty_cycle < 0) {
      duty_cycle = 0;
    }
    Serial.print("Duty cycle decreased");
  } else {
    Serial.print("Duty cycle not changed");
  }
}
 
// void updateDutyCycle(uint8_t distance) {
//   // Calculate the error
//   float error = target_distance - distance;
//   // Proportional term
//   float proportional = Kp * error;
//   // Integral term
//   integral += error;  // Accumulate error over time
//   float integral_term = Ki * integral;
//   // Derivative term
//   float derivative = error - previous_error;  // Rate of change of error
//   float derivative_term = Kd * derivative;
//   // PID output
//   float pid_output = proportional + integral_term + derivative_term;
 
//   // Update duty cycle
//   duty_cycle += pid_output;
//   // Clamp the duty cycle to [0, 255]
//   if (duty_cycle > 255) {
//     duty_cycle = 255;
//   } else if (duty_cycle < 0) {
//     duty_cycle = 0;
//   }
 
//   // Update previous error
//   previous_error = error;
 
//   // Debugging information
//   Serial.print("PID Output: "); Serial.println(pid_output);
//   Serial.print("Updated Duty Cycle: "); Serial.println(duty_cycle);
// }
 
 
void printErr(uint8_t status) {
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }
}