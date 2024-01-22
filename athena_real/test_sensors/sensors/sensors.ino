//#include "MPU9250.h"
//#include <SharpIR.h>

////----------------------Sonar----------------------
//#define Echo_inPin 12
//#define Trigger_outPin 11
//
//int maximumRange = 300;
//int minimumRange = 2;
//float distance;
//float duration;

//----------------------Força-----------------------
const int numForceSensors = 2;
const int F_Pins[numForceSensors] = {A0, A1};
//const int F_Pins[numForceSensors] = {A0, A1, A2, A3, A4, A5};

////-----------------------IMU------------------------
//MPU9250 mpu;
//
////-----------------------IR------------------------
//#define IRPin A8
//#define model 1080
//
//int distance_cm;
//SharpIR mySensor = SharpIR(IRPin, model);

void setup() {
  Serial.begin (9600);
//  //----------------------Sonar----------------------
//  pinMode (Trigger_outPin, OUTPUT);
//  pinMode (Echo_inPin, INPUT);
//
//  //-----------------------IMU------------------------
//  Wire.begin();
//  if(!mpu.setup(0x68)){
//    while(1){
//      Serial.println("MPU connection failed");
//      delay(5000);
//    }
//  }
//  delay(2000);

}

void loop() {
//  //----------------------Sonar----------------------
//  digitalWrite (Trigger_outPin, HIGH);
//  delayMicroseconds (10);
//  digitalWrite (Trigger_outPin, LOW);
//
//  duration = pulseIn (Echo_inPin, HIGH);
//
//  distance = duration / 58.2;
//
//  if (distance >= maximumRange || distance <= minimumRange)
//  {
//    //Serial.print("d = ");
//    Serial.print(0);
//  }
//  else
//  {
//    //Serial.print("d = ");
//    Serial.print(distance);
//    Serial.print(",");
//  }

  //----------------------Força-----------------------
  int force_value[numForceSensors];
  for (int i = 0; i < numForceSensors; i++)
  {
    force_value[i] = analogRead(F_Pins[i]);
  }

  for (int i = 0; i < numForceSensors; i++)
  {
    Serial.print(force_value[i]);
    if (i < (numForceSensors-1))
    {
      Serial.print(",");
    }
  }
  Serial.println();
  delay(1000);

//  //-----------------------IMU------------------------
//  if(mpu.update()) {
//    static uint32_t prev_ms = millis();
//    if(millis() > prev_ms + 25){
//      print_roll_pitch_yaw();
//      prev_ms = millis();
//    }
//  }
//
//  //-----------------------IR------------------------
//  distance_cm = mySensor.distance();
//  Serial.println(distance_cm);
//
//  delay (1000);
}

//void print_roll_pitch_yaw() {
//  //Vel Angular em X
//  Serial.print(mpu.getGyroX(), 2);
//  Serial.print(",");
//  //Accel Linear em X
//  Serial.print(mpu.getAccX(), 2);
//  Serial.print(",");
//  
//}
