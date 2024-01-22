#include <Servo.h>
#include <SerialTransfer.h>
#include <Adafruit_PWMServoDriver.h>
#include "MPU9250.h"
#include <SharpIR.h>

// Servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Servo CTr6;
Servo FTi6;
const uint16_t numServos = 18;

// Serial communication
float pos[numServos+6+4];
float data[19];
String receivedString = "";
String msgReceived = "";
// Serial Transfer communication
SerialTransfer tx;

//Force Sensor
int forceSensorPin1 = A0; // pin for your sensor
int forceSensorPin2 = A1;
int forceSensorPin3 = A2;
int forceSensorPin4 = A3;
int forceSensorPin5 = A4;
int forceSensorPin6 = A5;
float forceSensorValue1;
float forceSensorValue2;
float forceSensorValue3;
float forceSensorValue4;
float forceSensorValue5;
float forceSensorValue6;

//IMU
MPU9250 mpu;
float imu_velx;
float imu_accelx;

//IR Sensor
#define IRPin A8
#define model 1080

float distance_ir;
SharpIR IR_Sensor = SharpIR(IRPin, model);

// STATE MACHINE
typedef enum e_State {
    STATE_START = 0, // first state to initialize
    STATE_READ_CMD,  // reading message with required position  
    STATE_MOVE_POS,  // moving Servo to the selected position
    STATE_SEND_CMD,  // sending message when position reached
    STATE_WAIT_ACK   // waiting acknowledge from message
} eState;

typedef enum e_Pos {
    ACTUAL_POS = 0,
    NEW_POS
} ePos;

//static eState LoopState = STATE_START; // to store the current state
static ePos ServoPos = ACTUAL_POS;  // to store the selected position

// FUNCTIONS
void checkForNewData();
void beginServos();
void servos(float angles[]);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  tx.begin(Serial);


  //beginServos();
  pwm.begin();
  pwm.setPWMFreq(50);

  //IMU
  Wire.begin();
  if(!mpu.setup(0x68)){
    while(1){
      delay(5000);
    }
  }

  CTr6.attach(7);
  FTi6.attach(8);
  CTr6.write(90);
  //FTi6.write(90);
  float p = map(30.0, 0.0, 180.0, 155.0, 535.0);
  pwm.setPWM(0, 0, p);
  delay(2000);
  Serial.println("HERE 3");
}

void loop() {

static eState LoopState = STATE_START; // to store the current state

    eState NextState = LoopState;

    switch (LoopState) {
    case STATE_START:
        // Safe setup - Setting the servos to their initial positions
        Serial.println("Done.");
        ServoPos = ACTUAL_POS;
        NextState = STATE_READ_CMD;
        break;
    case STATE_READ_CMD:
        checkForNewData();
        break;
    default:
        // when undefined state, restart
        NextState = STATE_START;
    }
    // define the state for the next loop
    LoopState = NextState;
}

// ------------ SERIAL COM ---------------
void checkForNewData(){

    if (tx.available())
    {
      for (uint16_t i = 0; i < tx.bytesRead / sizeof(float); i++) {
        uint16_t index = i * sizeof(float);
        // Use memcpy to copy the bytes into a float
        memcpy(&pos[i], &tx.packet.rxBuff[index], sizeof(float));
      }
      
      // send all received data back to Python
      /*for(uint16_t i=0; i < tx.bytesRead; i++)
      {
        tx.packet.txBuff[i] = tx.packet.rxBuff[i];
      }
      tx.sendData(tx.bytesRead);*/
      float p = 0;
      for (uint16_t i = 0; i < 16; i++)
      {
        p = map(pos[i], 0.0, 180.0, 155.0, 535.0);
        pwm.setPWM(i, 0, p);
      }
      CTr6.write(pos[16]);
      FTi6.write(pos[17]);
      //delay(100);
      
      //Force Sensor
      forceSensorValue1 = analogRead(forceSensorPin1);
      forceSensorValue2 = analogRead(forceSensorPin2);
      forceSensorValue3 = analogRead(forceSensorPin3);
      forceSensorValue4 = analogRead(forceSensorPin4);
      forceSensorValue5 = analogRead(forceSensorPin5);
      forceSensorValue6 = analogRead(forceSensorPin6);

      pos[18] = forceSensorValue1;
      pos[19] = forceSensorValue2;
      pos[20] = forceSensorValue3;
      pos[21] = forceSensorValue4;
      pos[22] = forceSensorValue5;
      pos[23] = forceSensorValue6;
      
      //IMU
      if(mpu.update()) {
        static uint32_t prev_ms = millis();
        if(millis() > prev_ms + 25){
          //Vel Angular em X
          imu_velx = mpu.getGyroX();

          //Accel Linear em X
          imu_accelx = mpu.getAccX();
          prev_ms = millis();
        }
      }

      pos[24] = imu_velx;
      pos[25] = imu_accelx;

      //IR Sensor
      distance_ir = IR_Sensor.distance();
      pos[26] = distance_ir;

      float get_p = pwm.getPWM(1);
      float read_p = map(get_p, 155.0, 535.0, 0.0, 180.0);

      pos[27] = read_p;

      // Calculate the size of the data to send
      uint16_t dataSize = (numServos * sizeof(float)) + 10 * sizeof(float);
      
      // Send the pos array data     
      tx.txObj(pos);

      // Echo the received data back to Python (optional)
      tx.sendData(dataSize);
    }
}
