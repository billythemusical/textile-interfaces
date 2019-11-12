/*
Adapted from:
   DIGF 6037 Creation & Computation
   Kate Hartman & Nick Puckett
   Experiment 3 - Lab 2
   Arduino to Processing - sending value for (3) analog values
   Circuit: 
   (3) analog sensors connected to pins A0, A1, A2   
*/


int sensor1val;


void setup() {
  //start serial connection
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {

  // read the sensor value
  sensor1val = analogRead(2);
  // print out the sensor value and add a line break
  Serial.println(sensor1val);

  delay(100); // the delay is necessary for the serial communication

  if(sensor1val > 1000) 
  { 
    digitalWrite(13, HIGH); 
  }
  else 
  { 
    digitalWrite(13, LOW); 
  } 


}
