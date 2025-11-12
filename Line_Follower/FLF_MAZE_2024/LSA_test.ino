const int Kp = 0;   // Kp value that you have to change
const int Kd = 0;   // Kd value that you have to change
const int setPoint = 35;    // Middle point of sensor array
const int baseSpeed = 100;    // Base speed for your motors
const int maxSpeed = 200;   // Maximum speed for your motors

const byte rx = 0;    // Defining pin 0 as Rx
const byte tx = 1;    // Defining pin 1 as Tx
const byte serialEn = 17;    // Connect UART output enable of LSA08 to pin 2
const byte junctionPulse = 4;   // Connect JPULSE of LSA08 to pin 4
const byte dir1 = 13;   // Connect DIR1 of motor driver to pin 13
const byte dir2 = 12;   // Connect DIR2 of motor driver to pin 12
const byte pwm1 = 11;   // Connect PWM1 of motor driver to pin 11
const byte pwm2 = 10;   // Connect PWM2 of motor driver to pin 10

void setup() {
  pinMode(serialEn,OUTPUT);   // Setting serialEn as digital output pin
  pinMode(junctionPulse,INPUT);   // Setting junctionPulse as digital input pin

  // Setting pin 10 - 13 as digital output pin
  for(byte i=10;i<=13;i++) {
    pinMode(i,OUTPUT);
  }

  // Setting initial condition of serialEn pin to HIGH
  digitalWrite(serialEn,HIGH);

  // Setting the initial condition of motors
  // make sure both PWM pins are LOW
  digitalWrite(pwm1,LOW);
  digitalWrite(pwm2,LOW);

  // State of DIR pins are depending on your physical connection
  // if your robot behaves strangely, try changing thses two values
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,LOW);

  // Begin serial communication with baudrate 9600
  Serial.begin(9600);

}

int lastError = 0;    // Declare a variable to store previous error
int positionVal;
int Jpulse ;
void loop() {
  digitalWrite(serialEn,LOW);   // Set serialEN to LOW to request UART data
  while(Serial.available() <= 0);   // Wait for data to be available
  positionVal = Serial.read();    // Read incoming data and store in variable positionVal
  Jpulse = digitalRead(junctionPulse);
  digitalWrite(serialEn,HIGH);    // Stop requesting for UART data

  Serial.print("Position Value: ");
  Serial.print(positionVal);
  Serial.print(", Jpulse: ");
  Serial.println(Jpulse);

  
}
