#define Echo_inPin 12 // Echo input pin
#define Trigger_outPin 11 // Trigger output pin
 
// Required variables are defined
int maximumRange = 300;
int minimumRange = 2;
float distance;
float duration;
 
void setup () {
 pinMode (Trigger_outPin, OUTPUT);
 pinMode (Echo_inPin, INPUT);
 Serial.begin (115200);
}
 
void loop () {
 
 // Distance measurement is started by means of the 10us long trigger signal
 digitalWrite (Trigger_outPin, HIGH);
 delayMicroseconds (10);
 digitalWrite (Trigger_outPin, LOW);
  
 // Now we wait at the echo input until the signal has been activated
 // and then the time measured how long it remains activated
 duration = pulseIn (Echo_inPin, HIGH);
  
 // Now the distance is calculated using the recorded time
 distance = duration / 58.2;
  
 // Check whether the measured value is within the permissible distance
 if (distance >= maximumRange || distance <= minimumRange)
 {
    // If not, an error message is output.
      Serial.println (0);
 }
  
 else
 {
    // The calculated distance is output in the serial output
      Serial.println (distance);
 }
  // Pause between the individual measurements
 delay (250);
}
