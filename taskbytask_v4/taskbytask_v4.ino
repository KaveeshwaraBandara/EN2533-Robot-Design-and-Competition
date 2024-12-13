//libries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>`
#include <Adafruit_VL53L0X.h>


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
unsigned int numSensors = 10;      // Enter number of sensors as 5 or 7

//PID Values
int P, D, I, previousError, PIDvalue;
double error = 0.00;
int lsp, rsp;
int lfSpeed = 80;
int currentSpeed = 80;
int sensorWeight[10] = {5, 4, 2, 1, 0, 0, -1, -2, -4, -5};
int activeSensors;
float Kp = 15;
float Kd = 5;
float Ki = 0.00;

int onLine = 1;

//Define arrays for store IR Array readings
int minValues[10], maxValues[10], threshold[10], sensorValue[10], sensorArray[10];

// Define color sensor pins
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define OUT 2
// #define VCC_color 7

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
const int pickBoxLed = 53;  //LED for task 2 virtual box pickup indication

// Constants for encoders
const int EN_LEFT_A_PIN = 2; 
const int EN_LEFT_B_PIN = 22; 
const int EN_RIGHT_A_PIN = 18; 
const int EN_RIGHT_B_PIN = 19; 
#define CPR 225          // Encoder Pulses Per Revolution
#define WHEEL_DIAMETER 0.064 // Wheel diameter in meters (example: 6.5 cm)

// Time tracking
unsigned long prevTime = 0;
unsigned long prevPIDTime = 0;

const float CIRCUMFERENCE = PI * WHEEL_DIAMETER; // Wheel circumference in meters
volatile int position_left = 0;                       // Encoder position (counts)
volatile int position_right = 0;  

// PID control parameters
float KpEncoder = 8.0;  // Proportional gain
float KiEncoder = 0.5;  // Integral gain
float KdEncoder = 4.0;  // Derivative gain

// Speed control parameters
float basePWM = 65;             // Base PWM value
float integral = 0.0;
float prevError = 0.0;
int baseleftPWM = basePWM;

// Variables for speed calculation
volatile int prev_position_left = 0;
volatile int prev_position_right = 0;
float speed_left = 0.0;
float speed_right = 0.0;

// Time tracking
//unsigned long prevTime = 0;

int linecolor = 0;
float length = 0.00;
int distance_right = 0;
int distance_left = 0;
int distance1 = 0;
int distance2 = 0;
int distance3 = 0;

//ultrasonic
const int trig_pin = 5;
const int echo_pin = 6;
float timing = 0.0;
float distance = 0.0;

//define of tof
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33

// set the pins to shutdown
#define SHT_LOX1 25
#define SHT_LOX2 26
#define SHT_LOX3 27
#define SHT_LOX4 28

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;

//varibales for hold tof values
int Tof1read;
int Tof2read;
int Tof3read;
int Tof4read;

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

int   pos_left_now = 0;
int   pos_right_now = 0;
int   pos_left_pre = 0;
int   pos_right_pre = 0;

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
  // pinMode(VCC_color, OUTPUT);
  
  // Set Sensor output as input
 pinMode(OUT, INPUT);
  
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

  //Setup for tof
pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);

  Serial.println(F("All in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  //setID();

  
  calibrate();
  position_left = 0;                       // Encoder position (counts)
  position_right = 0;
}

void loop() {
  switch (currentState) {
    case STATE_TASK1:
      task1();
      break;

    case STATE_TASK2:
      task2();
      break;

    case STATE_TASK3:
      task3();
      break;
    
    case STATE_TASK4:
      task4();
      break;

    case STATE_TASK5:
      task5();
      break;

    case STATE_TASK6:
      task4();
      break;

    case STATE_TASK7:
      task7();
      break;

    case STATE_TASK8:
      task8();
      break;

    default:
      Serial.println("Unknown state!");
      break;
  }
}



// Task 1: Barcode Reading
void task1() {
  display.setCursor(0, 10);
  display.println("Task1");
  display.display(); 
  
  readLine();
  int count = 0;
  for (int i = 0; i < 10; i++) {
    if (sensorArray[i]){                     //check this condition for if because i didn't remembere output in ir array high or low for white surface
      count++;
    }
  }
  digitalWrite(53,LOW);

    if(count<6){
      //digitalWrite(53,HIGH);
      display.clearDisplay();
      pos_left_now = position_left;
      pos_right_now = position_right;

    float distance_left = (pos_left_now - pos_left_pre); // Distance in meters
    float distance_right = (pos_right_now - pos_right_pre); // Distance in meters

    float length = (distance_left + distance_right)/2;  //average value
    if (length>20){
    currentSize++;
    digitalWrite(53,HIGH);


    if (length>50){
      // if(currentSize == 0){
      //   barCode[currentSize-] = 1;   
      // }
        //digitalWrite(53,HIGH);
        barCode[currentSize-1] = 1; 
        display.println("Value = 1");
        display.display(); 
      
      
    }
    else{
      // if(currentSize == 0){
      //   barCode[currentSize] = 0;
      //   display.println("Value = 0");
      //   display.display(); 
      // }
        barCode[currentSize-1] = 0;
        display.println("Value = 0");
        display.display(); 
        
    }

    // motor2run(0);
    // motor1run(0);
    // delay(3000);

    }
    pos_left_pre = pos_left_now;
    pos_right_pre = pos_right_now;
  }   

  // if (count < 4){
  //   pos_left_now = position_left;
  //   pos_right_now = position_right;

  //   float distance_left = (pos_left_now - pos_left_pre / CPR) * CIRCUMFERENCE; // Distance in meters
  //   float distance_right = (pos_right_now - pos_right_pre / CPR) * CIRCUMFERENCE; // Distance in meters

  //   float length = (distance_left + distance_right)/2;  //average value
  //   if (length>0.005){
  //   currentSize++;

  //   if (length > 0.045){
  //     barCode[currentSize-1] = 1; 
  //   }
  //   else{
  //     barCode[currentSize-1] = 0;
  //   }
  //   }
  //   pos_left_pre = pos_left_now;
  //   pos_right_pre = pos_right_now;
  // }

  synchronizeMotorSpeeds(1);
  //Transition to Task 2 after barcode is read
  if (barcodeReadComplete()) {
    currentState = STATE_TASK2;
    digitalWrite(53,LOW);
    
    motor2run(0);
    motor1run(0);
    delay(1000);
    for(int i=0; i<vBoxPosition;i++){
      digitalWrite(53,HIGH);
      delay(1000);
      digitalWrite(53,LOW);
      delay(1000);
    }
    motor2run(0);
    motor1run(0); 
  }
  
}

// Task 2: Maze navigation
void task2() {
  int colr;
  int wall = openWall();                            //first wall = 0 second wall = 1
  wall = 1;
  if(vBoxPosition == 0){                        //8,9 - right 0,1- left
    uTurn();
    avoidjunc();

    while(true){
      readLine();
      if(sensorArray[0] == 1 && sensorArray[1] == 1){
        break;
      }
      linefollow();
    }

    motor2run(0);
    motor1run(0);

    delay(500);
    pickBox();
    delay(500);

    if(wall == 0){
      inversejump();
      moveBack();
      moveForVB0();      
    
    // while(true){
    //   readLine();
    //   int colr = detectcolour();
    //   if(colr == 1 || colr == 2){
    //     break;
    //   }
    //   linefollow();
    // }
    // motor2run(0);
    // motor1run(0);

    // linecolor = colr;

    // placeBox();
    // placed = 1;
    }
    
    else if(wall == 1){
      inversejump();
      moveBack();
      inversejump();
      moveBack();
      //inversejump();
      moveForVB0();

    //   do{
    //   readLine();
    //   linefollow(); 
    //   int colr = detectcolour(); 
    // }while(colr == 1 || colr == 2);

    // motor2run(0);
    // motor1run(0);

    // placeBox();
    // placed = 1;
    }
  }

  //when vBox in pos 1 
  else if(vBoxPosition == 1){
    if(wall == 0){
      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }
      
      motor2run(0);
      motor1run(0);

      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackColour();

      placeBox();
      placed = 1;
    }
    else{
      uTurn();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();

      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      delay(500);
      pickBox();
      delay(500);

      jump();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      jump();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      delay(500);
      placeBox();
      delay(500);

      inversejump();
      moveBackAbove();
      inversejump();

      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      // Turnright();

      // while(true){
      //   readLine();
      //   if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
      //     break;
      //   }
      //   linefollow();
      // }

      // motor2run(0);
      // motor1run(0);

      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackColour();

      placeBox();
      placed = 1;
    }
  }

  else if(vBoxPosition == 2){
    if(wall == 0){
      Turnright();
      avoidjunc();
      
      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();

      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackAbove();

      motor2run(0);
      motor1run(0);

      delay(500);
      placeBox();
      delay(500);

      inversejump();
      moveBackAbove();
      inversejump();

      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);
      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackColour();

      delay(500);
      placeBox();
      delay(500);
      placed = 1;
    }

    else{
      Turnright();
      avoidjunc();
      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      delay(500);
      pickBox();
      delay(500);

      jump();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      delay(500);
      placeBox();
      delay(500);
      
      inversejump();
      moveBackAbove();
      inversejump();
      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright(); 
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      } 

      motor2run(0);
      motor1run(0);

      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackColour();

      delay(500);
      placeBox();
      delay(500);
      placed = 1;
    }
  }

  else if(vBoxPosition == 3){
    if(wall == 0){
      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      jump();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackAbove();
      inversejump();
      moveBackAbove();
      inversejump();

      motor2run(0);
      motor1run(0);
      
      delay(500);
      placeBox();
      delay(500);

      inversejump();
      moveBackAbove();
      inversejump();

      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);
      
      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackColour();

      delay(500);
      placeBox();
      delay(500);

      placed = 1;
    }
    else{
      jump();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);
      
      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackColour();
      inversejump();

      delay(500);
      placeBox();
      delay(500);
      placed = 1;
    }

  }

  else if(vBoxPosition == 4){
    if(wall == 0){
      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      jump();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      jump();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackAbove();
      inversejump();
      moveBackAbove();
      inversejump();
      moveBackAbove();
      inversejump();

      motor2run(0);
      motor1run(0);

      delay(500);
      placeBox();
      delay(500);

      inversejump();
      moveBackAbove();
      inversejump();

      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);
      
      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackColour();

      delay(500);
      placeBox();
      delay(500);

      placed = 1;
    }
    else{
      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      jump();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      jump();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      delay(500);
      pickBox();
      delay(500);
      inversejump();
      moveBackAbove();

      motor2run(0);
      motor1run(0);
      
      delay(500);
      placeBox();
      delay(500);

      inversejump();
      moveBackAbove(); 
      
      Turnleft();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();
      avoidjunc(); 

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      } 

      motor2run(0);
      motor1run(0);

      delay(500);
      pickBox();
      delay(500);

      inversejump();
      moveBackColour();

      delay(500);
      placeBox();
      delay(500);
      placed = 1;
    }
  }
  if(placed){
    currentState = STATE_TASK3;
  }  
}

// Task 3: colour line following
void task3() {
  while(true){
    int count;
    count = sensorCount();
    if(count<5){
      break;
    }
    synchronizeMotorSpeeds(1);
  }
  while(true){
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  //PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
    } 
    else {
      digitalWrite(13, LOW);
      if (error > 0) {
        motor1run(-100);
        motor2run(lfSpeed);
      } 
      else {
        motor1run(lfSpeed);
        motor2run(-100);
      }
    }
  if(colourLineDone()){
    motor2run(0);
    motor1run(0);
    currentState = STATE_TASK4;
    break;
  }
  } 
}

// Task 4: Dotted line following
void task4() {
  int count;
 //if all black we need to move foward until we found white dashed line  
  while(true){
    count = sensorCount();
    if(count==0){
      synchronizeMotorSpeeds(1);
    }
    else{
      linefollow();
    }

  //condition for state transition
  if(count>7){
    motor2run(0);
    motor1run(0);
    currentState = STATE_TASK5;
    break;
  }
  }
}

// Task 5: Portal Navigation
void task5(){
  if(gateDetected()){
    do{
      motor2run(0);
      motor1run(0);
    }while(gateDetected());
    do{
      synchronizeMotorSpeeds(1);
    }while(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1);
    currentState = STATE_TASK6;
    motor2run(0);
    motor1run(0);
  }
  else{
    do{
      motor2run(0);
      motor1run(0);
    }while(!gateDetected());
    do{
      motor2run(0);
      motor1run(0);
    }while(gateDetected());

    do{
      synchronizeMotorSpeeds(1);
    }while(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1);
    currentState = STATE_TASK6;
    
    motor2run(0);
    motor1run(0);
  }
}

void task6() {
   if (linecolor == 1){ //1 is blue
    while(true){
      inversereadline();
      linefollow();
      if (sensorArray[8] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
    }
    Turnleft();
    inverseavoidjunc();

// 1st box
    while(true){
      inversereadline();
      linefollow();
      if (Tof1read <= 50 ){//zero is black
          break;
      }
    }
    Grabbox();
    distance2 = Tof2read;
    distance3 = Tof3read;
    uTurn();
    inverseavoidjunc();
    while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
    }
    //largest box
    if(distance2 <= 100 && distance3 <= 100){
      Turnleft();
      inverseavoidjunc();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      jump();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnright();
      Placerealbox();
      Turnright();
      inverseavoidjunc();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnright();
    }
    //medium box
    if(distance2 <= 100 && distance3 > 100){
      Turnleft();
      inverseavoidjunc();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnright();
      inverseavoidjunc();
      Placerealbox();
      inversejump();
      uTurn();
    }

    //small box
    if(distance2 > 100 && distance3 > 100){
      Placerealbox();
      Turnleft();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnleft();
    }

//2nd box
    while(true){
      inversereadline();
      linefollow();
      if (Tof1read <= 50 ){//zero is black
          break;
      }
    }
    Grabbox();
    distance2 = Tof2read;
    distance3 = Tof3read;
    uTurn();
    inverseavoidjunc();
    while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
    }
    //large box
    if(distance2 <= 100 && distance3 <= 100){
      Turnleft();
      inverseavoidjunc();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnright();
      inverseavoidjunc();
      Placerealbox();
      inversejump();
      uTurn();
    }

    //medium box
    if(distance2 <= 100 && distance3 > 100){
        jump();
        Placerealbox();
        Turnleft();
        inverseavoidjunc();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnleft();
      }
    //small box
    if(distance2 > 100 && distance3 > 100){
        Turnright();
        inverseavoidjunc();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
        }
        }
        Turnleft();
        inverseavoidjunc();
        Placerealbox();
        Turnleft();
        inverseavoidjunc();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
        }
        }
        jump();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
        }
        }
        Turnleft();
    }

// 3rd box

    while(true){
      inversereadline();
      linefollow();
      if (Tof1read <= 50 ){//zero is black
          break;
      }
    }
    Grabbox();
    distance2 = Tof2read;
    distance3 = Tof3read;
    uTurn();
    inverseavoidjunc();
    while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
    }
    //large box
    if(distance2 <= 100 && distance3 <= 100){
        Placerealbox();
        Turnleft();
        while(true){
          inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1 && sensorArray[5] == 1){//zero is black
            break;
        }
        }
      
    }
    //medium box
    if(distance2 < 100 && distance3 > 100){
        Turnright();
        inverseavoidjunc();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
        Turnleft();
        Placerealbox();
        Turnleft();
        inverseavoidjunc();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
        jump();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
    }
    //small box
    if(distance2 > 100 && distance3 > 100){
      Turnright();
      inverseavoidjunc();
      while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
        jump();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
      Turnleft();
      Placerealbox();
      Turnleft();
      while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
        jump();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1 && sensorArray[5] == 1){//zero is black
            break;
        }
        }
   }
   }
  
  if (linecolor == 2){ //1 is blue
    while(true){
      inversereadline();
      linefollow();
      if (sensorArray[8] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
    }
    Turnleft();
    inverseavoidjunc();

// 1st box
    while(true){
      inversereadline();
      linefollow();
      if (Tof1read <= 50 ){//zero is black
          break;
      }
    }
    Grabbox();
    distance2 = Tof2read;
    distance3 = Tof3read;
    uTurn();
    inverseavoidjunc();
    while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
    }
    //small box
    if(distance2 > 100 && distance3 > 100){
      Turnleft();
      inverseavoidjunc();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      jump();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnright();
      Placerealbox();
      Turnright();
      inverseavoidjunc();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnright();
    }
    //medium box
    if(distance2 <= 100 && distance3 > 100){
      Turnleft();
      inverseavoidjunc();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnright();
      inverseavoidjunc();
      Placerealbox();
      inversejump();
      uTurn();
    }

    //large box
    if(distance2 <= 100 && distance3 <= 100){
      Placerealbox();
      Turnleft();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnleft();
    }

//2nd box
    while(true){
      inversereadline();
      linefollow();
      if (Tof1read <= 50 ){//zero is black
          break;
      }
    }
    Grabbox();
    distance2 = Tof2read;
    distance3 = Tof3read;
    uTurn();
    inverseavoidjunc();
    while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
    }
    //small box
    if(distance2 > 100 && distance3 > 100){
      Turnleft();
      inverseavoidjunc();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnright();
      inverseavoidjunc();
      Placerealbox();
      inversejump();
      uTurn();
    }

    //medium box
    if(distance2 <= 100 && distance3 > 100){
        jump();
        Placerealbox();
        Turnleft();
        inverseavoidjunc();
      while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
      }
      Turnleft();
      }
    //large box
    if(distance2 <= 100 && distance3 <= 100){
        Turnright();
        inverseavoidjunc();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
        }
        }
        Turnleft();
        inverseavoidjunc();
        Placerealbox();
        Turnleft();
        inverseavoidjunc();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
        }
        }
        jump();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
        }
        }
        Turnleft();
    }

// 3rd box

    while(true){
      inversereadline();
      linefollow();
      if (Tof1read <= 50 ){//zero is black
          break;
      }
    }
    Grabbox();
    distance2 = Tof2read;
    distance3 = Tof3read;
    uTurn();
    inverseavoidjunc();
    while(true){
      inversereadline();
      linefollow();
      if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
          break;
      }
    }
    //small box
    if(distance2 > 100 && distance3 > 100){
        Placerealbox();
        Turnleft();
        while(true){
          inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1 && sensorArray[5] == 1){//zero is black
            break;
        }
        }
      
    }
    //medium box
    if(distance2 < 100 && distance3 > 100){
        Turnright();
        inverseavoidjunc();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
        Turnleft();
        Placerealbox();
        Turnleft();
        inverseavoidjunc();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
        jump();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
    }
    //small box
    if(distance2 > 100 && distance3 > 100){
      Turnright();
      inverseavoidjunc();
      while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
        jump();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
      Turnleft();
      Placerealbox();
      Turnleft();
      while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
        jump();
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1){//zero is black
            break;
        }
        }
        while(true){
        inversereadline();
        linefollow();
        if (sensorArray[0] == 1 && sensorArray[9] == 1 && sensorArray[5] == 1){//zero is black
            break;
        }
        }
   }
   }
}


void task7(){
  Turnleft();
  do{
    inversereadline();
    linefollow();
  }while(sensorArray[0] == 1 && sensorArray[1] == 1);

  motor2run(0);
  motor1run(0);

  Turnleft();
  //move until found the box and pick and move back until met the junction
  //......

  motor2run(0);
  motor1run(0);

  Turnright();

  //Hidden Task....................



}

void task8(){
}


// Helper Functions to Check Task Completion
bool barcodeReadComplete() {
  if(currentSize >= 3){
    if (barCode[currentSize-1]==barCode[currentSize-2]&& barCode[currentSize-2] == barCode[currentSize-3]){                 //check the flag for end the Barcode
    vBoxPosition = binaryToDecimal(barCode, currentSize)%5;    //calculate the virtual box position for task 2 (check do i need to replace the value in array to 10)
    display.clearDisplay();
    display.setCursor(0, 10);
    display.print("Barcode =");
    display.println(vBoxPosition);

    display.println("Barcode Array:");
      for (int i = 0; i < currentSize; i++) { // Loop to print only a portion of the array
        display.print(barCode[i]);
        display.print(" ");
      }

    display.println("END");
    display.display(); 
    return true;
  }
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

  for (int i = 0; i < 1000; i++) {
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
void synchronizeMotorSpeeds(int forward) { 
        //foward - 1 backward - 0
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - prevPIDTime;

    if (deltaTime >= 100) { // Update every 100 ms
        prevPIDTime = currentTime;

        // Calculate speeds
        noInterrupts(); // Prevent ISR interference
        int delta_left = position_left - prev_position_left;
        int delta_right = position_right - prev_position_right;
        
        
        // Speed (in m/s)
        //speed_left = (delta_left / (float)CPR) * CIRCUMFERENCE / (deltaTime / 1000.0);
        //speed_right = (delta_right / (float)CPR) * CIRCUMFERENCE / (deltaTime / 1000.0);

        // PID calculation
        float error = abs(position_right) - abs(position_left);
        integral += error * 100 / 1000.0;
        float derivative = (error - prevError) / (100 / 1000.0);
        prevError = error;

        prev_position_left = position_left;
        prev_position_right = position_right;
        interrupts();


        float correction = (KpEncoder * error + KiEncoder * integral + KdEncoder * derivative)/100;
        baseleftPWM += (int)correction;
        // Calculate motor PWM values
        int left_pwm = constrain(baseleftPWM, 0, 120); // Base PWM for left motor
        int right_pwm = constrain(basePWM, 0, 120); // Adjusted PWM for right motor

        // Set motor directions
        if (forward == 1) {
            digitalWrite(AIN1, HIGH);  
            digitalWrite(BIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN2, LOW);
        } else {
            digitalWrite(AIN2, HIGH);
            digitalWrite(BIN2, HIGH);
            digitalWrite(AIN1, LOW);  
            digitalWrite(BIN1, LOW);
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
        Serial.println(error);
    }
} 
// void synchronizeMotorSpeeds(int forward) {        //foward - 1 backward - 0
//     unsigned long currentTime = millis();
//     unsigned long deltaTime = currentTime - prevPIDTime;

//     if (deltaTime >= 100) { // Update every 100 ms
//         prevPIDTime = currentTime;

//         // Calculate speeds
//         noInterrupts(); // Prevent ISR interference
//         //int delta_left = position_left - prev_position_left;
//         //int delta_right = position_right - prev_position_right;
        
        
//         // Speed (in m/s)
//         //speed_left = (delta_left / (float)CPR) * CIRCUMFERENCE / (deltaTime / 1000.0);
//         //speed_right = (delta_right / (float)CPR) * CIRCUMFERENCE / (deltaTime / 1000.0);

//         // PID calculation
//         float error = position_right - position_left;
//         integral += error * deltaTime / 1000.0;
//         float derivative = (error - prevError) / (deltaTime / 1000.0);
//         prevError = error;

//         prev_position_left = position_left;
//         prev_position_right = position_right;
//         interrupts();


//         float correction = (KpEncoder * error + KiEncoder * integral + KdEncoder * derivative)/100;
//         baseleftPWM += (int)correction;
//         // Calculate motor PWM values
//         int left_pwm = constrain(baseleftPWM, 0, 255); // Base PWM for left motor
//         int right_pwm = constrain(basePWM, 0, 255); // Adjusted PWM for right motor

//         // Set motor directions
//         if (forward == 1) {
//             digitalWrite(AIN1, HIGH);  
//             digitalWrite(BIN1, HIGH);
//             digitalWrite(AIN2, LOW);
//             digitalWrite(BIN2, LOW);
//         } else {
//             digitalWrite(AIN2, HIGH);
//             digitalWrite(BIN2, HIGH);
//             digitalWrite(AIN1, LOW);  
//             digitalWrite(BIN1, LOW);
//         }

//         // Set motor speeds
//         analogWrite(PWMA, left_pwm);
//         analogWrite(PWMB, right_pwm);

//         // Debugging output
//         Serial.print("Speed Left: ");
//         Serial.print(speed_left);
//         Serial.print(" m/s, Speed Right: ");
//         Serial.print(speed_right);
//         Serial.print(" m/s, Error: ");
//         Serial.println(error);
//     }
// }

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

//check this (Got from chatgpt)s
int binaryToDecimal(int arr[], int size) {
int decimalValue = 0;
  int effectiveSize = size - 3; // Omit last 3 digits
  int biarr[effectiveSize];
  for (int i = 0; i < effectiveSize; i++)
  {
    biarr[i] = arr[i];
  }

  // Convert the binary array (excluding the last 3 digits) to a decimal value
  for (int i = 0; i < effectiveSize; i++) {
    decimalValue = (decimalValue << 1) | biarr[i];
  }

  return decimalValue;
}

//turn right using encoders
void Turnright(){
  position_left = 0;
  position_right = 0;  
  while(position_left < 110 || position_right < 110){
    if (position_left < 110 && position_right < 110){
      motor2run(lfSpeed);
      motor1run(lfSpeed);
    }
    else if (position_left < 110 && position_right > 110){
      motor1run(lfSpeed);
      motor2run(0);      
    }
    else if (position_left > 110 && position_right < 110){
      motor1run(0);
      motor2run(lfSpeed);      
    }
    else{
      motor2run(0);
      motor1run(0);   
    }
  }
  position_left = 0;
  position_right = 0;  
  while (position_left < 150 || position_right > -150 ){
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("pos_left =");
    display.println(position_left);
    display.println("pos_right =");
    display.println(position_right);
    display.display(); 
    if (position_left < 150 && position_right > -150){
      motor1run(lfSpeed);
      motor2run(-(lfSpeed+10));
    }
    else if (position_left < 150 && position_right < -150){
      motor1run(lfSpeed);
      motor2run(0);      
    }
    else if (position_left > 150 && position_right > -150){
      motor1run(0);
      motor2run(-(lfSpeed+10));      
    }
    else{
      motor2run(0);
      motor1run(0);   
    }
  }
  motor2run(0);
  motor1run(0);   
}

//turn left using encoders
void Turnleft(){
  position_left = 0;
  position_right = 0; 
  while(position_left < 110 || position_right < 110){
    if (position_left < 110 && position_right < 110){
      motor2run(lfSpeed);
      motor1run(lfSpeed);
    }
    else if (position_left < 110 && position_right > 110){
      motor1run(lfSpeed);
      motor2run(0);      
    }
    else if (position_left > 110 && position_right < 110){
      motor1run(0);
      motor2run(lfSpeed);      
    }
    else{
      motor2run(0);
      motor1run(0);   
    }
  }
  position_left = 0;
  position_right = 0; 

  while (position_left > -140 || position_right < 140 ){
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("pos_left =");
    display.println(position_left);
    display.println("pos_right =");
    display.println(position_right);
    display.display(); 
    if (position_left > -150 && position_right < 150){
      motor1run(-(lfSpeed+10));
      motor2run(lfSpeed);
    }
    else if (position_left < -150   && position_right < 140){
      motor1run(0);
      motor2run(lfSpeed);      
    }
    else if (position_left > -150 && position_right > 150){
      motor1run(-(lfSpeed+10));
      motor2run(0);      
    }
    else{
      motor2run(0);
      motor1run(0);   
    }
  }
  motor2run(0);
  motor1run(0); 

  position_left = 0;
  position_right = 0; 
}

//identify junction left(1) or right(2) or straight(0)
int Junction(){
  readLine();
  if (sensorArray[0] == 0 && sensorArray[1] == 0){          //left
    return 1;
  }
  else if(sensorArray[8] == 0 && sensorArray[9] == 0){      //right
    return 2;                                                 
  }
  else{
    return 0;
  }
}

//avoid junction errors
void avoidjunc(){
  position_left = 0;
  position_right = 0;
  while(position_left < 110 && position_right < 110){
    readLine();
    linefollow();
  }
  motor2run(0);
  motor1run(0);
}

void inverseavoidjunc(){
  while(position_left < 110 && position_right < 110){
    inversereadline();
    linefollow();
  }
  motor2run(0);
  motor1run(0);
}

void inversejump(){
  position_left = 0;
  position_right = 0; 
  while(position_left > -110 || position_right > -110){
    if (position_left > -110 && position_right > -110){
      motor2run(-lfSpeed);
      motor1run(-lfSpeed);
    }
    else if (position_left > -110 && position_right < -110){
      motor1run(-lfSpeed);
      motor2run(0);      
    }
    else if (position_left < -110 && position_right > -110){
      motor1run(0);
      motor2run(-lfSpeed);      
    }
    else{
      motor2run(0);
      motor1run(0);   
    }
  }
}

//move little bit away from junction
void jump(){
  position_left = 0;
  position_right = 0; 
  while(position_left < 110 || position_right < 110){
    if (position_left < 110 && position_right < 110){
      motor2run(lfSpeed);
      motor1run(lfSpeed);
    }
    else if (position_left < 110 && position_right > 110){
      motor1run(lfSpeed);
      motor2run(0);      
    }
    else if (position_left > 110 && position_right < 110){
      motor1run(0);
      motor2run(lfSpeed);      
    }
    else{
      motor2run(0);
      motor1run(0);   
    }
  }
}

//detect the open wall firstone(0) or secondone(1)      8,9 - right  0,1 - left
int openWall(){
  //move to the maze from barcode 
  while(true){
    readLine();
    linefollow();
    if(sensorArray[8] == 1 && sensorArray[9] == 1){
      break;
    }
  }
    
  motor2run(0);
  motor1run(0);

  Turnright();

  avoidjunc();

  while(true){
    readLine();
    if(sensorArray[8] == 1 && sensorArray[9] == 1){
      break;
    }
    linefollow();
  }

  motor2run(0);
  motor1run(0);
  
  //checking conditions for box position 
  if(vBoxPosition == 0){
    delay(500);
    pickBox();
    delay(500);

    avoidjunc();

    while(true){
      readLine();
      if(sensorArray[8] == 1 && sensorArray[9] == 1){
        break;
      }
      linefollow();
    }
    
    motor2run(0);
    motor1run(0); 
    
    delay(500);
    placeBox();
    delay(500);

    uTurn();
    delay(500);
    
    avoidjunc();
    
    while(true){
      readLine();
      if(sensorArray[0] == 1 && sensorArray[1] == 1){
        break;
      }
      linefollow();
    }

    motor2run(0);
    motor1run(0); 
    
    Turnleft();

    avoidjunc();

    while(true){
      readLine();
      if(sensorArray[0] == 1 && sensorArray[1] == 1){
        break;
      }
      linefollow();
    }

    motor2run(0);
    motor1run(0); 

    Turnleft();
    avoidjunc();

    while(true){
      readLine();
      if(sensorArray[0] == 1 && sensorArray[1] == 1&&sensorArray[8] == 1 && sensorArray[9] == 1){
        break;
      }
      linefollow();
    }

    motor2run(0);
    motor1run(0);

    Turnright();
    delay(500);

    if(detectWall()){
      digitalWrite(53,HIGH);
      delay(2000);
      digitalWrite(53,LOW);
      return 0;
    }
    else{
      digitalWrite(53,HIGH);
      delay(2000);
      digitalWrite(53,LOW);
      delay(1000);
      digitalWrite(53,HIGH);
      delay(2000);
      digitalWrite(53,LOW);
      return 1;
    }

  }
  else{
    jump();

    while(true){
      readLine();
      if(sensorArray[8] == 1 && sensorArray[9] == 1){
        break;
      }
      linefollow();
    }
  

    motor2run(0);
    motor1run(0);

    Turnright();

    avoidjunc();

  //check if there is a need for move little bit above from junction.. here actually what we do is we avoid the first junction and move foward
    // while(true){
    //   readLine();
    //   if(sensorArray[8] == 1 && sensorArray[9] == 1){
    //     break;
    //   }
    //   linefollow();
    // }

    // motor2run(0);
    // motor1run(0);

    // Turnright();

    while(true){
      readLine();
      if(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1){
        break;
      }
      linefollow();
    }

    motor2run(0);
    motor1run(0);

    //Turnleft();

    // if(detectWall()){
    //   return 0;
    // }
    // else{
    //   return 1;
    // }
    if(detectWall()){
      digitalWrite(53,HIGH);
      delay(2000);
      digitalWrite(53,LOW);
      return 0;
    }
    else{
      digitalWrite(53,HIGH);
      delay(2000);
      digitalWrite(53,LOW);
      delay(1000);
      digitalWrite(53,HIGH);
      delay(2000);
      digitalWrite(53,LOW);
      return 1;
    }
  }
}

//pick Vbox
void pickBox(){
  digitalWrite(pickBoxLed, HIGH);  //turn on LED
}

//place Vbox
void placeBox(){
  digitalWrite(pickBoxLed, LOW); //turn off LED
}

//turn 180 degree
void uTurn(){
  position_left = 0;
  position_right = 0;  
  while (position_left > -325 || position_right < 325 ){ 
    if (position_left > -325 && position_right < 325){
      motor1run(-(lfSpeed+10));
      motor2run(lfSpeed);
    }
    else if (position_left < -325  && position_right < 325){
      motor1run(0);
      motor2run(lfSpeed);      
    }
    else if (position_left > -325 && position_right > 325){
      motor1run(-(lfSpeed+10));
      motor2run(0);      
    }
    else{
      motor2run(0);
      motor1run(0);   
    }
  }
  motor2run(0);
  motor1run(0); 
}

//detect the obstacle using ultrasonic return 1 when open and return 0 when close
int detectWall(){
  digitalWrite(trig_pin, LOW);
  delay(2);
  
  digitalWrite(trig_pin, HIGH);
  delay(10);
  digitalWrite(trig_pin, LOW);
  
  timing = pulseIn(echo_pin, HIGH);
  distance = (timing * 0.034) / 2;
  
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.print("cm | ");
  // Serial.print(distance / 2.54);
  // Serial.println("in");
  
    
  if (distance <= 15){
  	return 1;
  } else {
    return 0;
  }
}

//move backward until met a 4 way junction
void moveBack(){
  // do{
  //   readLine();
  //   synchronizeMotorSpeeds(0);
  // }while(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1);
  int initialPWM = basePWM;
  basePWM = 100;
  while(true){
    readLine();
    synchronizeMotorSpeeds(0);
    if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
      break;
    }
  basePWM = initialPWM;
  }
  
  motor2run(0);
  motor1run(0);  
}

//detect blue(1) or red(2) otherwise return 0 
int detectcolour(){
//   digitalWrite(S0, HIGH);
//   digitalWrite(S1, HIGH);
//   digitalWrite(VCC_color, HIGH);

//   // Read Red frequency
//   digitalWrite(S2, LOW);
//   digitalWrite(S3, LOW);
//  int  redFrequency = pulseIn(OUT, LOW);
// //  Serial.print("redFrequency :");
// //  Serial.print(redFrequency);
// //  Serial.print("  ");

//   // Read Blue frequency
//   digitalWrite(S2, LOW);
//   digitalWrite(S3, HIGH);
//   int blueFrequency = pulseIn(OUT, LOW);
// //  Serial.print("blueFrequency :");
// //  Serial.println(blueFrequency);

//   // Determine the color
//   if (redFrequency < 20 && blueFrequency < 20) {
//     return 0; // White
//   } else if (blueFrequency < redFrequency && blueFrequency < 40) {
//     digitalWrite(S0, LOW);
//     digitalWrite(S1, LOW);
//     digitalWrite(S0, LOW);
//     digitalWrite(S1, LOW);
//     digitalWrite(VCC_color, LOW);
//     return 1; // Blue
//   } else if(blueFrequency > redFrequency && redFrequency < 40){
//     digitalWrite(S0, LOW);
//     digitalWrite(S1, LOW);
//     digitalWrite(S0, LOW);
//     digitalWrite(S1, LOW);
//     digitalWrite(VCC_color, LOW);
//     return 2; // Red
//   }
//   else if (redFrequency > 40 && blueFrequency > 40){
//     return 0; // Black
// }
}



//move back with colour detection(we need to move back until colour sensor detect red or blue)
void moveBackColour(){
  int detected;
  // do{
  //   synchronizeMotorSpeeds(0);
  //   detected = detectcolour();
  // }while(detected==0);

  while(true){
    synchronizeMotorSpeeds(0);
    detected = detectcolour();

    if(detected==0){
      break;
    }
  }
  motor2run(0);
  motor1run(0); 
}

//this is for move back in maze above line(move back until met a junction)
void  moveBackAbove(){
  // do{
  //   readLine();
  //   synchronizeMotorSpeeds(0);
  // }while(sensorArray[0] == 1 && sensorArray[1] == 1);

  while(true){
    readLine();
    synchronizeMotorSpeeds(0);

    if(sensorArray[0] == 1 && sensorArray[1] == 1){
      break;
    }
  }
  
  motor2run(0);
  motor1run(0); 
}

// grabbing the box
void Grabbox(){

}

void Placerealbox(){
  distance_left = 0;
  distance_right = 0;
  do{
    inversereadline();
    linefollow();
    float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
    float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
    float length = (distance_left + distance_right)/2;  //average value
    }while(length > 30);
    Dropbox();
    movebackinverse();
    motor2run(0);
    motor1run(0);
    Turnleft();
}

// //linefollow in black line
// void inverselinefollow(){
// //online = 0
// }

//detect a black line
void inversereadline(){
//readline check the conditions and change to read black line for error correction
  onLine = 1;
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

//move back in black line with syncrhonization
void movebackinverse(){
  do{
    inversereadline();
    synchronizeMotorSpeeds(0);
  }while(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1);
  
  motor2run(0);
  motor1run(0);
}

void Dropbox(){
//drop the box for place the box using servo
}

// int Tof1read(){     //this is for detect the gate  return value is distance

// }

// int Tof2read(){

// }

// int Tof3read(){

// }



//detect gate closed or open (open retuns false.. close returns true)
bool gateDetected(){
  int distance;
    read_quad_sensors();
    distance = Tof4read;
    if(distance < 250){      //tof output is in millimeters(condition for distance less than 25cm)
      return true;
    }
    else{
      return false;
    }
}

//count the number of sensors detect white surface
int sensorCount(){
  readLine();
  int count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorArray[i]){                     //check this condition for if because i didnt remembere output in ir array high or low for white surface
      count++;
    }
  }
  return count;
}

void moveForVB0(){
    int colr;
    motor2run(0);
    motor1run(0);
    
    delay(500);
    placeBox();
    delay(500);

    

    uTurn();

    while(true){
      linefollow();
      if(sensorArray[8]==1 && sensorArray[9]==1){
        break;
      }
    }

  motor2run(0);
  motor1run(0);

  Turnright();

  avoidjunc();

  while(true){
    readLine();
    linefollow();
    if(sensorArray[0]==1 && sensorArray[1]==1 && sensorArray[8]==1 && sensorArray[9]==1){
      break;
    }
  }

    motor2run(0);
    motor1run(0);
    
    Turnright();
    
    avoidjunc();

    while(true){
      readLine();
      linefollow();
      if(sensorArray[8]==1 && sensorArray[9]==1){
        break;
      }
    }

    motor2run(0);
    motor1run(0);

    Turnright();

    while(true){
      readLine();
      linefollow();
      if(sensorArray[0]==1 && sensorArray[1]==1 && sensorArray[8]==1 && sensorArray[9]==1){
        break;
      }
    }

    motor2run(0);
      motor1run(0);

      delay(500);
      pickBox();
      delay(500);
    
    while(true){
      colr = detectcolour();
      readLine();
      linefollow();
      if(colr == 1 || colr == 2){
        break;
      }
    }
    placeBox();
    placed = 1;
}

//Function of tof
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);    
  digitalWrite(SHT_LOX4, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);    
  digitalWrite(SHT_LOX4, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX2
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  //initing LOX2
  if(!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_quad_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  if(measure1.RangeStatus != 4) {     // if not out of range
    Tof1read =measure1.RangeMilliMeter;
  } else {
    Tof1read = 1000;
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Tof2read = measure2.RangeMilliMeter;
  } else {
    Tof2read = 1000;
  }
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("3: "));
  if(measure3.RangeStatus != 4) {
    Tof3read = measure3.RangeMilliMeter;
  } else {
    Tof3read = 1000;
  }

  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("4: "));
  if(measure4.RangeStatus != 4) {
    Tof4read = measure4.RangeMilliMeter;
  } else {
    Tof4read = 1000;
  }
  Serial.println();
}