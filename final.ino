#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <DHT.h>

// Define pins for DHT11 sensor
#define DHTPIN 2
#define DHTTYPE DHT11

// Define pins for ultrasonic sensor
#define TRIGGER_PIN 8
#define ECHO_PIN 9

// Define servo pin
#define SERVO_PIN 5

// Constants for ultrasonic sensor and servo motor
const int DISTANCE_THRESHOLD = 100; // cm
const int SERVO_DELAY = 150;         // ms between servo movements
const int SERVO_START = 15;         // Starting angle
const int SERVO_END = 165;          // Ending angle (150-degree sweep)

// Initialize I2C LCD (address: 0x27, size: 16x2)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Create Servo and DHT objects
Servo myservo;
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Initialize LCD and display a welcome message
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Initializing");

  delay(1000); // Wait for initialization

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");

  // Set up ultrasonic sensor pins and initialize DHT sensor
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  dht.begin();

  // Attach the servo motor and set it to the starting position
  myservo.attach(SERVO_PIN);
  myservo.write(SERVO_START);
}

void loop() {
  bool intruderDetected = false;
  int intruderDistance = -1;
  int intruderAngle = -1;

  // Perform a forward sweep with the servo motor to detect intruders
  for (int angle = SERVO_START; angle <= SERVO_END; angle += 3) {
    myservo.write(angle);
    delay(SERVO_DELAY);
    int distance = measureDistance();
    if (distance < DISTANCE_THRESHOLD) {
      intruderDetected = true;
      intruderDistance = distance;
      intruderAngle = angle;
      displayIntruderAlert(intruderAngle, intruderDistance);
      break;
    }
  }

  // If no intruder is detected in the forward sweep, perform a backward sweep
  if (!intruderDetected) {
    for (int angle = SERVO_END; angle >= SERVO_START; angle -= 3) {
      myservo.write(angle);
      delay(SERVO_DELAY);
      int distance = measureDistance();
      if (distance < DISTANCE_THRESHOLD) {
        intruderDetected = true;
        intruderDistance = distance;
        intruderAngle = angle;
        displayIntruderAlert(intruderAngle, intruderDistance);
        break;
      }
    }
  }

  // If no intruder is detected, display temperature and humidity on LCD
  if (!intruderDetected) {
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (!isnan(temperature) && !isnan(humidity)) {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print("°C, Humidity: ");
      Serial.print(humidity);
      Serial.println("%");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperature);
      lcd.print(" C");

      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print(" %");
    } else {
      Serial.println("Failed to read from DHT sensor!");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor Error!");
    }

    delay(2000); // Update every two seconds
  }
}

// Function to measure distance using the ultrasonic sensor
int measureDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(20);

  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH); // Measure time of echo pulse

  return duration * 0.034 / 2; // Convert to distance in cm
}

// Function to display an intruder alert on the LCD and Serial Monitor
void displayIntruderAlert(int angle, int distance) {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("INTRUDER ALERT!");

  lcd.setCursor(0, 1);
  
lcd.print("deg:");  
lcd.print(angle);  
lcd.print(" Dis:");  
lcd.print(distance);  
lcd.print("cm");  

Serial.print("Intruder detected at angle ");  
Serial.print(angle);  
Serial.print(", Distance: ");  
Serial.println(distance);  

delay(2000);  
}  
