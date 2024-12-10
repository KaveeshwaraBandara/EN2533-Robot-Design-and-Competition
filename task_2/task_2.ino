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
  // put your main code here, to run repeatedly:

}
