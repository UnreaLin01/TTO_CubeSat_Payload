#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050_tockn.h>
#include <EEPROM.h>

//#define DEBUG_MSG

#define RX_LED 17
#define BLUE_LED 8
#define GREEN_LED 9
#define SEA_LEVEL_PRESSURE 1013.25

#define T2  25.9  // Temperature data point 1
#define R2  661.0 // Reading data point 1
#define T1  15.5  // Temperature data point 2
#define R1  695.0 // Reading data point 2

Adafruit_BME280 bme;
MPU6050 mpu6050(Wire);

void led_setup();
void led_blink(int length);
void led_set(int led_pin, bool state);
void eeprom_word_write(int addr, int val);
short eeprom_word_read(int addr);
void update_sensors();

static uint8_t bme_present;
static float bme_temperature;
static float bme_pressure;
static float bme_altitude;
static float bme_humidity;

static float mpu_gyro_x;
static float mpu_gyro_y;
static float mpu_gyro_z;
static float mpu_acc_x;
static float mpu_acc_y;
static float mpu_acc_z;

static float diode_temp;
static float voltage;

void setup() {
  Serial.begin(115200); // Uart through USB
  Serial1.begin(115200); // Uart to raspberry pi zero.

  analogReadResolution(12);

  #if defined(DEBUG_MSG)
    Serial.println("STM Payload Starting");
  #endif

  led_setup();
  led_blink(500);
  delay(250);
  led_blink(500);
  delay(250);
  led_set(GREEN_LED, HIGH);
  delay(250);
  led_set(GREEN_LED, LOW);
  led_set(BLUE_LED, HIGH);
  delay(250);
  led_set(BLUE_LED, LOW);

  // Check the existance of BME280 sensor
  if (bme.begin(0x76)) {
    bme_present = 1;
  } else {
    bme_present = 0;
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

  #if defined(DEBUG_MSG)
    Serial.println("STM Payload Done Setup");
  #endif
}

void loop() {
  update_sensors();

  // Message handling of raspberry pi zero
  if (Serial1.available() > 0){
    
    char tmp = Serial1.read();
    led_blink(50);

    Serial1.print("OK BME280 ");
    Serial1.print(bme_temperature);
    Serial1.print(" ");
    Serial1.print(bme_pressure);
    Serial1.print(" ");
    Serial1.print(bme_altitude);
    Serial1.print(" ");
    Serial1.print(bme_humidity);

    Serial1.print(" MPU6050 ");
    Serial1.print(mpu_gyro_x);
    Serial1.print(" ");
    Serial1.print(mpu_gyro_y);
    Serial1.print(" ");
    Serial1.print(mpu_gyro_z);
    Serial1.print(" ");
    Serial1.print(mpu_acc_x);
    Serial1.print(" ");
    Serial1.print(mpu_acc_y);
    Serial1.print(" ");
    Serial1.print(mpu_acc_z);

    Serial1.print(" XS ");
    Serial1.print(diode_temp);
    Serial1.print(" ");
    Serial1.println(voltage);
  }

  // Message handling of uart though USB-C
  if(Serial.available() > 0){
    
    char msg = Serial.read();
    led_blink(50);

    if(msg == 'R'){
      #if defined(DEBUG_MSG)
        Serial.println("System Restart");
      #endif
      delay(500);
      setup();

    }else if(msg == 'C'){
      #if defined(DEBUG_MSG)
        Serial.println("Clear gyro offsets");
      #endif
      delay(500);
      eeprom_word_write(0, 0x00);
      setup();
      
    }else if(msg == '?'){
      #if defined(DEBUG_MSG)
        Serial.print("OK BME280 ");
        Serial.print(bme_temperature);
        Serial.print(" ");
        Serial.print(bme_pressure);
        Serial.print(" ");
        Serial.print(bme_altitude);
        Serial.print(" ");
        Serial.print(bme_humidity);

        Serial.print(" MPU6050 ");
        Serial.print(mpu_gyro_x);
        Serial.print(" ");
        Serial.print(mpu_gyro_y);
        Serial.print(" ");
        Serial.print(mpu_gyro_z);
        Serial.print(" ");
        Serial.print(mpu_acc_x);
        Serial.print(" ");
        Serial.print(mpu_acc_y);
        Serial.print(" ");
        Serial.print(mpu_acc_z);

        Serial.print(" XS ");
        Serial.print(diode_temp);
        Serial.print(" ");
        Serial.println(voltage);
      #else
        Serial.write(0xFF);
        Serial.write((byte *) &bme_temperature, 4);
        Serial.write((byte *) &bme_pressure, 4);
        Serial.write((byte *) &bme_altitude, 4);
        Serial.write((byte *) &bme_humidity, 4);
        Serial.write((byte *) &mpu_gyro_x, 4);
        Serial.write((byte *) &mpu_gyro_y, 4);
        Serial.write((byte *) &mpu_gyro_z, 4);
        Serial.write((byte *) &mpu_acc_x, 4);
        Serial.write((byte *) &mpu_acc_y, 4);
        Serial.write((byte *) &mpu_acc_z, 4);
        Serial.write((byte *) &diode_temp, 4);
        Serial.write((byte *) &voltage, 4);
      #endif
    }
  }

  delay(50);
}

void update_sensors(){

  bme_temperature = bme_present ? bme.readTemperature() : 0.0;
  bme_pressure = bme_present ? bme.readPressure() / 100.0 : 0.0;
  bme_altitude = bme_present ? bme.readAltitude(SEA_LEVEL_PRESSURE) : 0.0;
  bme_humidity = bme_present ? bme.readHumidity() : 0.0;

  static float diode_temp_buf[10] = {0.0};
  static uint8_t diode_temp_buf_index = 0;
  static float diode_temp_sum = 0.0;

  diode_temp_sum -= diode_temp_buf[diode_temp_buf_index];
  diode_temp_buf[diode_temp_buf_index] = T1 + (analogRead(PA7) - R1) * ((T2 - T1) / (R2 - R1));
  diode_temp_sum += diode_temp_buf[diode_temp_buf_index];
  diode_temp_buf_index = (diode_temp_buf_index + 1) % 10;
  diode_temp = diode_temp_sum / 10.0;
  
  voltage = analogRead(PA4) * 3.3 / 4096;

  mpu6050.update();
  mpu_gyro_x = mpu6050.getGyroX();
  mpu_gyro_y = mpu6050.getGyroY();
  mpu_gyro_z = mpu6050.getGyroZ();
  mpu_acc_x = mpu6050.getAccX();
  mpu_acc_y = mpu6050.getAccY();
  mpu_acc_z = mpu6050.getAccZ();

}

void led_setup(){
  pinMode(PC13, OUTPUT);
  pinMode(PB9, OUTPUT);
  pinMode(PB8, OUTPUT);
}

void led_blink(int length){
  digitalWrite(PC13, LOW);
  delay(length);
  digitalWrite(PC13, HIGH);
}

void led_set(int led_pin, bool state){
  if(led_pin == GREEN_LED){
    digitalWrite(PB9, state);
  }else if(led_pin == BLUE_LED){
    digitalWrite(PB8, state);
  }
}

void eeprom_word_write(int addr, int val){
  EEPROM.write(addr * 2, lowByte(val));
  EEPROM.write(addr * 2 + 1, highByte(val));
}

short eeprom_word_read(int addr){
  return ((EEPROM.read(addr * 2 + 1) << 8) | EEPROM.read(addr * 2));
}