//libries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//Tasks
#define STATE_TASK1 1
#define STATE_TASK2 2
#define STATE_TASK3 3
#define STATE_TASK4 4
#define STATE_TASK5 5
#define STATE_TASK6 6
#define STATE_TASK7 7
#define STATE_TASK8 8

int currentState = STATE_TASK1;

//Pin definitions for the L298N Motor Driver
#define AIN1 8
#define BIN1 7
#define AIN2 9
#define BIN2 12
#define PWMA 10
#define PWMB 11

bool isBlackLine = 0;             //keep 1 in case of black line. In case of white line change this to 0
unsigned int lineThickness = 15;  //Enter line thickness in mm. Works best for thickness between 10 & 35
unsigned int numSensors = 7;      // Enter number of sensors as 5 or 7

//PID Values
int P, D, I, previousError, PIDvalue;
double error = 0.00;
int lsp, rsp;
int lfSpeed = 70;
int currentSpeed = 30;
int sensorWeight[10] = {5, 4, 2, 1, 0, 0, -1, -2, -4, -5};
int activeSensors;
float Kp = 0.007;
float Kd = 0.005;
float Ki = 0.00;

int onLine = 1;

//Define arrays for store IR Array readings
int minValues[10], maxValues[10], threshold[10], sensorValue[10], sensorArray[10];

// Define color sensor pins
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define sensorOut 2

int redMin = 17; // Red minimum value
int redMax = 145; // Red maximum value
int greenMin = 18; // Green minimum value
int greenMax = 155; // Green maximum value
int blueMin = 17; // Blue minimum value
int blueMax = 140; // Blue maximum value
//-----------------------------------------

int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int redValue;
int greenValue;
int blueValue;


//Define Array for store the barcode values(i dont know the exact numbers to add here so i allocate 10 elements check it before competion)
int barCode[20];
int currentSize = 0;
int vBoxPosition; 

//Box placed in task 2
int placed = 0;
const int pickBoxLed = 6;  //LED for task 2 virtual box pickup indication

// Constants for encoders
const int EN_LEFT_A_PIN = 2; 
const int EN_LEFT_B_PIN = 22; 
const int EN_RIGHT_A_PIN = 3; 
const int EN_RIGHT_B_PIN = 23; 
#define CPR 225          // Encoder Pulses Per Revolution
#define WHEEL_DIAMETER 0.064 // Wheel diameter in meters (example: 6.5 cm)


const float CIRCUMFERENCE = PI * WHEEL_DIAMETER; // Wheel circumference in meters
volatile int position_left = 0;                       // Encoder position (counts)
volatile int position_right = 0;   

// Speed control parameters
float basePWM = 70;             // Base PWM value
float correctionFactor = 2.0;    // Proportional control factor

// Variables for speed calculation
volatile int prev_position_left = 0;
volatile int prev_position_right = 0;
float speed_left = 0.0;
float speed_right = 0.0;

// Time tracking
unsigned long prevTime = 0;

int linecolor = 0;
float length = 0.00;
int distance_right = 0;
int distance_left = 0;
int distance1 = 0;
int distance2 = 0;
int distance3 = 0;

//ultrasonic
const int trig_pin = 4;
const int echo_pin = 5;
float timing = 0.0;
float distance = 0.0;

// Interrupt Service Routine for Channel A
void ISR_LEFT_A() {
    bool current_left_A = digitalRead(EN_LEFT_A_PIN);
    bool current_left_B = digitalRead(EN_LEFT_B_PIN);
    if (current_left_A == current_left_B)
        position_left++;
    else
        position_left--;
}

void ISR_RIGHT_A() {
    bool current_RIGHT_A = digitalRead(EN_RIGHT_A_PIN);
    bool current_RIGHT_B = digitalRead(EN_RIGHT_B_PIN);
    if (current_RIGHT_A == current_RIGHT_B)
        position_right--;
    else
        position_right++;
}

//Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void setup() {
  Serial.begin(9600);

  pinMode(EN_LEFT_A_PIN, INPUT_PULLUP);
  pinMode(EN_LEFT_B_PIN, INPUT_PULLUP);
  pinMode(EN_RIGHT_A_PIN, INPUT_PULLUP);
  pinMode(EN_RIGHT_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EN_LEFT_A_PIN), ISR_LEFT_A, RISING);
  attachInterrupt(digitalPinToInterrupt(EN_RIGHT_A_PIN), ISR_RIGHT_A, RISING);


  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Set S0 - S3 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // Set Sensor output as input
  pinMode(sensorOut, INPUT);
  
  // Set Frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);


  //LED out
  pinMode(13, OUTPUT);
  pinMode(pickBoxLed, OUTPUT);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.setTextSize(1);
  display.setTextColor(WHITE);

  
  calibrate();
}

void loop() {
  task1();

}

// Task 1: Barcode Reading
void task1() {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("Task1");
  display.display(); 
  motor1run(70);
  motor2run(70);
  
  
  readLine();
  int count = 0;
  for (int i = 0; i < 10; i++) {
    if (sensorArray[i]){                     //check this condition for if because i didn't remembere output in ir array high or low for white surface
      count++;
    }
  }
  if (count < 4){
    float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
    float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters

    float length = (distance_left + distance_right)/2;  //average value
    if (length>0.005){
    currentSize++;

    if (length > 0.045){
      barCode[currentSize-1] = 1; 
    }
    else{
      barCode[currentSize-1] = 0;
    }
    }
    position_left = 0;
    position_right = 0;
  }

  
  //Transition to Task 2 after barcode is read
  if (barcodeReadComplete()) {
    currentState = STATE_TASK2; 
  }
}

// Helper Functions to Check Task Completion
bool barcodeReadComplete() {
  if (barCode[currentSize-1]==barCode[currentSize-2]&& barCode[currentSize-2] == barCode[currentSize-3]){                 //check the flag for end the Barcode
    vBoxPosition = binaryToDecimal(barCode[currentSize], currentSize-3) % 5;    //calculate the virtual box position for task 2 (check do i need to replace the value in array to 10)
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("Barcode =");
    display.println(vBoxPosition);
    display.println("END");
    display.display(); 
    return true;
  }
  else{
    return false;
  }  
}

//maze completion condition already i wrote within the loop
//task 4 completion condition already i wrote within the loop

bool colourLineDone() {
  int count = 0;
  for (int i = 0; i < 10; i++) {
    if (sensorArray[i]){                     //check this condition for if because i didnt remembere output in ir array high or low for white surface
      count++;
    }
  }
  if(count>6){
    return true;
  }
  else{
    return false;
  } 
}

bool someOtherTaskComplete() {
  
  return true; 
}

//calibrate the IR Array
void calibrate() {
  for (int i = 0; i < 10; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 3000; i++) {
    motor1run(100);
    motor2run(-100);

    for (int i = 0; i < 10; i++) {
      if (analogRead(i) < minValues[i]) {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i]) {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for (int i = 0; i < 10; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
  delay(5000);
}

//Read the IR values in each and every loop
void readLine() {
  onLine = 0;
  if (numSensors == 10) {
    for (int i = 0; i < 10; i++) {
      if (isBlackLine) {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > 500;
      if (sensorArray[i]) onLine = 1;

      if (isBlackLine == 1 && sensorArray[i]) onLine = 1;
      if (isBlackLine == 0 && !sensorValue[i]) onLine = 1;
    }
  }
}

//Line follow using PID 
void linefollow() {
  error = 0;
  activeSensors = 0;

  if (numSensors == 10) {
    for (int i = 0; i < 10; i++) {
      error += sensorWeight[i] * sensorArray[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1run(lsp);
  motor2run(rsp);
}

// Function to synchronize motor speeds with direction control(Chatgpt) 
void synchronizeMotorSpeeds(int forward) {          //foward = 1 backward = 0
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - prevTime;

    if (deltaTime >= 100) { // Update every 100 ms
        prevTime = currentTime;

        // Calculate speeds
        noInterrupts(); // Prevent ISR interference
        int delta_left = position_left - prev_position_left;
        int delta_right = position_right - prev_position_right;
        prev_position_left = position_left;
        prev_position_right = position_right;
        interrupts();

        // Speed (in m/s)
        speed_left = (delta_left / (float)CPR) * CIRCUMFERENCE / (deltaTime / 1000.0);
        speed_right = (delta_right / (float)CPR) * CIRCUMFERENCE / (deltaTime / 1000.0);

        // Speed synchronization
        float speed_error = speed_left - speed_right;
        int left_pwm = constrain(basePWM, 0, 255);
        int right_pwm = constrain(basePWM + correctionFactor * speed_error, 0, 255);

        // Set motor directions
        if (forward) {
            digitalWrite(AIN1, HIGH);
            digitalWrite(BIN1, HIGH);
        } else {
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN2, LOW);
        }

        // Set motor speeds
        analogWrite(PWMA, left_pwm);
        analogWrite(PWMB, right_pwm);

        // Debugging output
        Serial.print("Speed Left: ");
        Serial.print(speed_left);
        Serial.print(" m/s, Speed Right: ");
        Serial.print(speed_right);
        Serial.print(" m/s, Error: ");
        Serial.println(speed_error);
    }
}

//Function to run Motor 1
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

//Function to run Motor 2
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}