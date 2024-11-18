#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050_tockn.h>
#include <EEPROM.h>


#define SEALEVELPRESSURE_HPA (1013.25)
#define DEBGU_MSG 1

#if defined __AVR_ATmega32U4__ 
float T2 = 25.9;  // Temperature data point 1
float R2 = 165;   // Reading data point 1
float T1 = 30.7;     // Temperature data point 2
float R1 = 160;   // Reading data point 2
#endif

//#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
#if defined(STM32F0) || defined(STM32F1) || defined(STM32F3) || defined(STM32F4) || defined(STM32L4)
float T2 = 25;    // Temperature data point 1
float R2 = 661;   // Reading data point 1
float T1 = 15.5;  // Temperature data point 2
float R1 = 695;   // Reading data point 2
#endif


void eeprom_word_write(int addr, int val);
short eeprom_word_read(int addr);
void blink_setup();
void blink(int length);
void led_set(int ledPin, bool state);
int read_analog();


Adafruit_BME280 bme;
MPU6050 mpu6050(Wire);

//HardwareSerial Serial1(PA10, PA9);  // Uart to raspberry pi zero.
//HardwareSerial Serial2(PA3, PA2); // External uart pin

long timer = 0;
int bmePresent;
int RXLED = 17;  // The RX LED has a defined Arduino pin
int greenLED = 9;
int blueLED = 8;
int Sensor1 = 0;
float Sensor2 = 0;
int first_time = true;
int first_read = true;

float Temp;
float rest;

void eeprom_word_write(int addr, int val);
short eeprom_word_read(int addr);

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);

  #if defined(DEBUG_MSG)
    Serial.println("STM Payload Starting");
  #endif

  blink_setup();
  blink(500);
  delay(250);
  blink(500);
  delay(250);
  led_set(greenLED, HIGH);
  delay(250);
  led_set(greenLED, LOW);
  led_set(blueLED, HIGH);
  delay(250);
  led_set(blueLED, LOW);

  // Check the existance of BME280 sensor
  if (bme.begin(0x76)) {
    bmePresent = 1;
  } else {
    bmePresent = 0;
    #if defined(DEBUG_MSG)
      Serial.println("Cound Not Find BME280 Sensor");
    #endif
  }

  // MPU6050 Initialize and calibrate if needed
  mpu6050.begin();
  if(eeprom_word_read(0) == 0xA07){
    #if defined(DEBUG_MSG)
      Serial.println("Reading gyro offsets from EEPROM\n");
    #endif
    
    float xOffset = ((float)eeprom_word_read(1)) / 100.0;
    float yOffset = ((float)eeprom_word_read(2)) / 100.0;
    float zOffset = ((float)eeprom_word_read(3)) / 100.0;
    
    #if defined(DEBUG_MSG)
      Serial.println(xOffset, DEC);
      Serial.println(yOffset, DEC);
      Serial.println(zOffset, DEC);
    #endif

    mpu6050.setGyroOffsets(xOffset, yOffset, zOffset);
  }else{
    #if defined(DEBUG_MSG)
      Serial.println("Calculating gyro offsets and storing in EEPROM\n");
    #endif

    mpu6050.calcGyroOffsets(true);
    eeprom_word_write(0, 0xA07);
    eeprom_word_write(1, (int)(mpu6050.getGyroXoffset() * 100.0) + 0.5);
    eeprom_word_write(2, (int)(mpu6050.getGyroYoffset() * 100.0) + 0.5);
    eeprom_word_write(3, (int)(mpu6050.getGyroZoffset() * 100.0) + 0.5);

    #if defined(DEBUG_MSG)
      Serial.println(eeprom_word_read(0), HEX);
      Serial.println(((float)eeprom_word_read(1)) / 100.0, DEC);
      Serial.println(((float)eeprom_word_read(2)) / 100.0, DEC);
      Serial.println(((float)eeprom_word_read(3)) / 100.0, DEC);
    #endif
  }
  analogReadResolution(12);

  #if defined(DEBUG_MSG)
    Serial.println("STM Payload Done Setup");
  #endif
}

void loop() {

  // Message handling of raspberry pi zero
  if (Serial1.available() > 0) {
    blink(50);
    char msg = Serial1.read();

    if(bmePresent){
      Serial1.print("OK BME280 ");
      Serial1.print(bme.readTemperature());
      Serial1.print(" ");
      Serial1.print(bme.readPressure() / 100.0F);
      Serial1.print(" ");
      Serial1.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      Serial1.print(" ");
      Serial1.print(bme.readHumidity());
    }else{
      Serial1.print("OK BME280 0.0 0.0 0.0 0.0");
    }
    mpu6050.update();

    Serial1.print(" MPU6050 ");
    Serial1.print(mpu6050.getGyroX());
    Serial1.print(" ");
    Serial1.print(mpu6050.getGyroY());
    Serial1.print(" ");
    Serial1.print(mpu6050.getGyroZ());

    Serial1.print(" ");
    Serial1.print(mpu6050.getAccX());
    Serial1.print(" ");
    Serial1.print(mpu6050.getAccY());
    Serial1.print(" ");
    Serial1.print(mpu6050.getAccZ());

    Temp = T1 + (read_analog() - R1) * ((T2 - T1) / (R2 - R1));
    Sensor2 = analogRead(PA4) * 3.3 / 4095;

    Serial1.print(" XS ");
    Serial1.print(Temp);
    Serial1.print(" ");
    Serial1.println(Sensor2);

    float rotation = sqrt(mpu6050.getGyroX() * mpu6050.getGyroX() + mpu6050.getGyroY() * mpu6050.getGyroY() + mpu6050.getGyroZ() * mpu6050.getGyroZ());
    float acceleration = sqrt(mpu6050.getAccX() * mpu6050.getAccX() + mpu6050.getAccY() * mpu6050.getAccY() + mpu6050.getAccZ() * mpu6050.getAccZ());

    if (first_read == true) {
      first_read = false;
      rest = acceleration;
    }

    if (acceleration > 1.2 * rest)
      led_set(greenLED, HIGH);
    else
      led_set(greenLED, LOW);

    if (rotation > 5)
      led_set(blueLED, HIGH);
    else
      led_set(blueLED, LOW);
  }

  // Message handling of uart though USB-C
  if(Serial.available() > 0){
    blink(50);
    char msg = Serial.read();

    if(msg == 'R'){
      #if defined(DEBUG_MSG)
        Serial.println("System Restart");
      #endif

      delay(500);
      first_read = true;
      setup();

    }else if(msg == 'C'){
      #if defined(DEBUG_MSG)
        Serial.println("Clear gyro offsets");
      #endif

      delay(500);
      first_time = true;
      eeprom_word_write(0, 0x00);
      setup();
    }

    if(msg == '?'){
      if(bmePresent){
        Serial.print("OK BME280 ");
        Serial.print(bme.readTemperature());
        Serial.print(" ");
        Serial.print(bme.readPressure() / 100.0F);
        Serial.print(" ");
        Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.print(" ");
        Serial.print(bme.readHumidity());
      }else{
        Serial.print("OK BME280 0.0 0.0 0.0 0.0");
      }

      mpu6050.update();
      Serial.print(" MPU6050 ");
      Serial.print(mpu6050.getGyroX());
      Serial.print(" ");
      Serial.print(mpu6050.getGyroY());
      Serial.print(" ");
      Serial.print(mpu6050.getGyroZ());

      Serial.print(" ");
      Serial.print(mpu6050.getAccX());
      Serial.print(" ");
      Serial.print(mpu6050.getAccY());
      Serial.print(" ");
      Serial.print(mpu6050.getAccZ());

      Temp = T1 + (read_analog() - R1) * ((T2 - T1) / (R2 - R1));
      //Temp = (analogRead(PA7));
      Sensor2 = analogRead(PA4) * 3.3 / 4095;

      Serial.print(" XS ");
      Serial.print(Temp);
      Serial.print(" ");
      Serial.println(Sensor2);
    }
  }
  delay(100);
}

void eeprom_word_write(int addr, int val){
  EEPROM.write(addr * 2, lowByte(val));
  EEPROM.write(addr * 2 + 1, highByte(val));
}

short eeprom_word_read(int addr){
  return ((EEPROM.read(addr * 2 + 1) << 8) | EEPROM.read(addr * 2));
}

void blink_setup(){
//#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
#if defined(STM32F0) || defined(STM32F1) || defined(STM32F3) || defined(STM32F4) || defined(STM32L4)
  // initialize digital pin PB1 as an output.
  pinMode(PC13, OUTPUT);
  pinMode(PB9, OUTPUT);
  pinMode(PB8, OUTPUT);
#endif

#if defined __AVR_ATmega32U4__
  pinMode(RXLED, OUTPUT);  // Set RX LED as an output
  // TX LED is set as an output behind the scenes
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
#endif
}

void blink(int length){
//#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
#if defined(STM32F0) || defined(STM32F1) || defined(STM32F3) || defined(STM32F4) || defined(STM32L4)
  digitalWrite(PC13, LOW);  // turn the LED on (HIGH is the voltage level)
#endif

#if defined __AVR_ATmega32U4__
  digitalWrite(RXLED, LOW);  // set the RX LED ON
  TXLED0;                    //TX LED is not tied to a normally controlled pin so a macro is needed, turn LED OFF
#endif

  delay(length);  // wait for a lenth of time

//#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
#if defined(STM32F0) || defined(STM32F1) || defined(STM32F3) || defined(STM32F4) || defined(STM32L4)
  digitalWrite(PC13, HIGH);  // turn the LED off by making the voltage LOW
#endif

#if defined __AVR_ATmega32U4__
  digitalWrite(RXLED, HIGH);  // set the RX LED OFF
  TXLED0;                     //TX LED macro to turn LED ON
#endif
}

void led_set(int ledPin, bool state){
//#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
#if defined(STM32F0) || defined(STM32F1) || defined(STM32F3) || defined(STM32F4) || defined(STM32L4)
  if (ledPin == greenLED)
    digitalWrite(PB9, state);
  else if (ledPin == blueLED)
    digitalWrite(PB8, state);
#endif

#if defined __AVR_ATmega32U4__
  digitalWrite(ledPin, state);
#endif
}

int read_analog(){
  int sensorValue;
#if defined __AVR_ATmega32U4__
  sensorValue = analogRead(A3);
#endif
//#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
#if defined(STM32F0) || defined(STM32F1) || defined(STM32F3) || defined(STM32F4) || defined(STM32L4)
  sensorValue = analogRead(PA7);
#endif
  return (sensorValue);
}
