CODE 1


#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// I2C LCD object
LiquidCrystal_I2C lcd(0x27, 16, 2);

// IR Sensor
const int IR1 = 6;
const int IR2 = 7;

// Servo motor pin
const int ServoMoto = 5;

// Ultrasonic
const int Trig = 3;
const int Echo = 4;
const float SpeedSoundAt20C = 343.0; // Speed of sound in air at 20°C in meters per second
const float TemperatureCo = 0.6;       // Temperature coefficient for the speed of sound in air in m/s/°C

// Vibration sensor
const int VibrationSensor = 8;

// Servo motor object
Servo myServo;

// Flag variable to indicate if an accident has occurred
bool accidentDetected = false;

// Previous vibration sensor status
int previousVibrationStatus = LOW;

void setup() {
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(VibrationSensor, INPUT);
  
  lcd.init();
  lcd.backlight();
  myServo.attach(ServoMoto);

  Serial.begin(9600); // Start serial communication
  
  while (!Serial);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20);
}

void loop() {
  // Read vibration sensor value
  int vibrationValue = digitalRead(VibrationSensor);

  // If vibration status changed, send the new status through LoRa
  if (vibrationValue != previousVibrationStatus) {
    if (vibrationValue == HIGH) {
      Serial.println("Sending Vibration Status: High");
      LoRa.beginPacket();
      LoRa.print("HIGH"); // Send "HIGH" when vibration is detected
      LoRa.endPacket();
    } else {
      Serial.println("Sending Vibration Status: Low");
      LoRa.beginPacket();
      LoRa.print("LOW"); // Send "LOW" when no vibration is detected
      LoRa.endPacket();
    }
    previousVibrationStatus = vibrationValue;
  }

  // Clear the trigger pin
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  // Set the trigger pin high for 10 microseconds
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  // Measure the duration of the pulse from the echo pin
  long duration = pulseIn(Echo, HIGH);
  // Calculate distance in meters
  double distance = duration * calculateSpeedOfSound() / 2000; // Divide by 2000 for round-trip time and convert to meters
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" meters");

  int IR1s = digitalRead(IR1);
  int IR2s = digitalRead(IR2);

  if (IR1s == LOW && IR2s == LOW)
  {
    moveServo(90);
    delay(2000);
  }
  else if (IR2s == HIGH && IR1s == LOW)
  {
    for (int pos = 90; pos >= 0; pos -= 1)
    {
      // goes from 90 degrees to 0 degrees
      myServo.write(pos);
      delay(15);
    }
    delay(2000);
  }
  else if (IR2s == LOW && IR1s == HIGH)
  {
    for (int pos = 90; pos <= 180; pos += 1)
    {
      // goes from 90 degrees to 180 degrees
      myServo.write(pos);
      delay(15);
    }
    delay(2000);
  }
  else
  {
    moveServo(90);
    delay(2000);
  }

  // Update LCD
  updateLCD(distance, vibrationValue);

  // Delay before the next iteration
  delay(1000); // Adjust delay as needed
}

// Function to move the servo to a specified angle
void moveServo(int angle)
{
  myServo.write(angle); // Set servo angle
  delay(1000);          // Wait for servo to reach the
}

// Function to update the LCD with distance and vibration status
void updateLCD(double distance, int vibrationValue)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("V.Dis.M: ");
  lcd.print(distance);
  lcd.setCursor(0, 1);
  lcd.print("Accient: ");
  lcd.print(vibrationValue == HIGH ? "Yes!S.N" : "No!");
}

// Function to calculate the speed of sound
float calculateSpeedOfSound()
{
  // Read temperature in Celsius (Note: You need to add a library or function to read temperature)
  float TemperatureCo = 20.0; // Example temperature, replace this with actual temperature reading
  // Calculate speed of sound based on temperature
  float SpeedSound = SpeedSoundAt20C + (TemperatureCo - 20) * TemperatureCo;
  return SpeedSound;
}

CODE 2

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <LiquidCrystal_I2C.h>

// I2C LCD object
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// Pin to control D7 LED
const int LEDPin = 7;

void setup() {
  pinMode(LEDPin, OUTPUT);
  lcd.init();
  lcd.backlight();

  Serial.begin(9600); // Start serial communication
  
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20);
}

void loop() {
  // Receive message
  String receivedMessage = receiveMessage();
  
  // If message is received, and it's "accident", display on LCD and turn on LED
  if (receivedMessage == "accident") {
    digitalWrite(LEDPin, HIGH);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Accident Detected!");
  } else {
    digitalWrite(LEDPin, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No Accident");
  }
  
  // Wait for a while before receiving again
  delay(1000);
}

// Function to receive message
String receiveMessage() {
  String message = "";
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet '");
    delay(1000);
    Serial.println(packetSize);
    // read packet
    while (LoRa.available()) {
      message += (char)LoRa.read();
    }
    Serial.print("Message: ");
    Serial.println(message);
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  } else {
    // No packet received
    Serial.println("Nothing received");
  }
  return message;
}
