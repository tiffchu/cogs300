


/**
 * Libraries
 */
#include <NewPing.h>

/**
 * Create variables to be used to run motor A
 * 
 * Label  Color  Arduino Pin
 * -------------------------
 * B-1A   o2     5
 * B-1B   w2     6
 * GND    b1     GND
 * VCC    r1     5V
 * A-1A   o1     9
 * A-1B   w1     10
 * 
 */
 
int motorAPin_A = 9;  //Arduino digital 9 is connected to HG7881's A-1A terminal
int motorAPin_B = 10; //Arduino digital 10 is connected to HG7881's A-1B terminal

int motorBPin_A = 5; //Arduino digital 5 is connected to HG7881's B-1A terminal
int motorBPin_B = 6; //Arduino digital 6 is connected to HG7881's B-1B terminal

int L_trig_pin = 7;  // Left trigger pin
int L_echo_pin = 8;  // Left echo pin
int R_trig_pin = 12; // Right trigger pin
int R_echo_pin = 13; // Right echo pin

int DIR_DELAY = 50;
unsigned long last_print = 0;
//Constants
const int pResistor_a = A5; // Photoresistor at Arduino analog pin A0
const int pResistor_b = A4; // Photoresistor at Arduino analog pin A1
const int pResistor_c = A3; // Photoresistor at Arduino analog pin A2

//Variables
int value_a;          // Store value from photoresistor A (0-1023)
int value_b;          // Store value from photoresistor B (0-1023)
int value_c;          // Store value from photoresistor C (0-1023)


void setup(){
  pinMode(pResistor_a, INPUT);// Set pResistor - A5 pin as an input (optional)
  pinMode(pResistor_b, INPUT);// Set pResistor - A4 pin as an input (optional)
  pinMode(pResistor_c, INPUT);// Set pResistor - A3 pin as an input (optional)
  /**
   * When program starts set Arduino pinmode for 8 and 9 digital to be OUTPUT
   * so we can use analogWrite to output values from 0 to 255 (0-5V) (PWM) 
   */
  pinMode(motorAPin_A, OUTPUT);
  pinMode(motorAPin_B, OUTPUT);
  
  pinMode(motorBPin_A, OUTPUT);
  pinMode(motorBPin_B, OUTPUT);

  // Ultrasonics
  pinMode(L_trig_pin, OUTPUT);
  pinMode(L_echo_pin, INPUT);
  pinMode(R_trig_pin, OUTPUT);
  pinMode(R_echo_pin, INPUT);

  Serial.begin(9600);
  delay(3000);
}

long getDistance(String sensor) {
  int trigPin = L_trig_pin;
  int echoPin = L_echo_pin;
  
  if (sensor == "R") {
    trigPin = R_trig_pin;
    echoPin = R_echo_pin;
  }
  
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  long cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  return cm;
}

// Moves Motor A forward at output 0-255
void A_forward(int output) {
  int A = motorAPin_A;
  int B = motorAPin_B;
  // set the motor speed and direction
  digitalWrite(A, LOW);      // direction = forward
  analogWrite(B, output);    // PWM speed = output
}

// Moves Motor A forward at output 0-255
void A_backward(int output) {
  int A = motorAPin_A;
  int B = motorAPin_B;
  // set the motor speed and direction
    digitalWrite(A, HIGH);      // direction = backward
    analogWrite(B, 255-output); // PWM speed = output
}

// Moves Motor A forward at output 0-255
void B_forward(int output) {
  int A = motorBPin_A;
  int B = motorBPin_B;
  // set the motor speed and direction
  digitalWrite(A, LOW);      // direction = forward
  analogWrite(B, output);    // PWM speed = output
}

// Moves Motor A forward at output 0-255
void B_backward(int output) {
  int A = motorBPin_A;
  int B = motorBPin_B;
  // set the motor speed and direction
    digitalWrite(A, HIGH);      // direction = backward
    analogWrite(B, 255-output); // PWM speed = output
}

void loop() {  
  value_a = analogRead(pResistor_a);
  value_b = analogRead(pResistor_b);
  value_c = analogRead(pResistor_c);
  Serial.println("-------------------------");
  Serial.println(value_a);
  Serial.println(value_b);
  Serial.println(value_c);
  
  if ((value_a > 100))
{
  A_forward(0); // stop motor b to turn R
} else {
  if (value_c > 175) {
    B_forward(0); // stop motor a to turn L
    }
  else {

  // set point for distance following. Too close and the sensors 
  // get wild readings, so keep it at 15-20 cm at minimum
  int set = 15; //cm         // TODO: change this value

  // p (proportional) values for PID control
  int p_a = 100;               // TODO: change this value
  int p_b = 100;               // TODO: change this value
  int motor_delay_p = 25;     // TODO: change this value
 
  // Min and max values for motor output and on-time delay
  int motor_delay_min = 10;  // TEST: may need to change this value
  int motor_delay_max = 500; // TEST: may need to change this value
  
  int min_out = 20;           // TODO: change this value
  int max_out = 180;           // TODO: change this value

  // Get distances for left and right
  // ultrasonic sensors
  int R_distance = int(getDistance("R"));
  int R_error = set - R_distance;
  delay(50);
  int L_distance = int(getDistance("L"));
  int L_error = set - L_distance;
  delay(50);

  // Calculate PID output for motor A
  int A_output = abs(p_a * R_error) + min_out;
  A_output = min(A_output, max_out); // clamp values to at most max_out;
  A_output = max(A_output, min_out); // and at least min_out;

  // if error is greater than 0, move backwards
  // if error is less than 0, move forwards
  // if error is exactly equal to zero, stay put
  if (R_error > 0) {        // TEST: may need to change the direction of ">"
    A_backward(A_output);   // TEST: may need to change A to B or backward to forward
  } else if (R_error < 0) { // TEST: may need to change the direction of "<"
    A_forward(A_output);    // TEST: may need to change A to B or backward to forward
  } else if (R_distance == set) {
    A_forward(0);
  }

  // Calculate PID output for motor B
  int B_output = abs(p_b * L_error) + min_out;
  B_output = min(B_output, max_out); // clamp values to at most max_out;
  B_output = max(B_output, min_out); // and at least 0;

  // if error is greater than 0, move backwards
  // if error is less than 0, move forwards
  // if error is exactly equal to zero, stay put
  if (L_error > 0) {         // TEST: may need to change the direction of ">"
    B_backward(B_output);    // TEST: may need to change B to A or backward to forward
  } else if (L_error < 0) {  // TEST: may need to change the direction of "<"
    B_forward(B_output);     // TEST: may need to change B to A or backward to forward
  } else if (L_distance == set) {
    B_forward(0);
  }
  
  // using PID control for motor delay as well by
  // taking average error between the sides
  int variable_motor_delay = motor_delay_min + motor_delay_p*int(abs(R_error) + abs(L_error)/2);
  variable_motor_delay = max(variable_motor_delay, motor_delay_min);
  variable_motor_delay = min(variable_motor_delay, motor_delay_max);
  delay(variable_motor_delay);
  Serial.println(variable_motor_delay);
  

  // Print at most twice a second
  if (millis() - last_print >= 500 || last_print < 0) {
    last_print = millis();
    Serial.println();
    Serial.println("Sensor\tdist\terror\toutput");
    Serial.println("-------------------------------------");
    Serial.println(
      "L\t" + 
      String(L_distance) + "\t" + 
      String(L_error) + "\t" + 
      String(B_output));
    Serial.println(
      "R\t" + 
      String(R_distance) + "\t" + 
      String(R_error) + "\t" + 
      String(A_output));

  // pause motors after each run
  A_forward(0);
  B_forward(0);
  delay(1000); // TEST: may need to change this value
  }
  }
}
}


// if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){forward();}   //if Right Sensor and Left Sensor are at White color then it will call forword function

// if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();} //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  

// if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();}  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function

// if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){Stop();} //if Right Sensor and Left Sensor are at Black color then it will call Stop function

