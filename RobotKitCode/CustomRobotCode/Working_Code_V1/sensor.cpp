//=================================================================================================
#include "sensor.hpp"

// address we will assign if triple sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX1 A1
#define SHT_LOX2 A2
#define SHT_LOX3 A3

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// stores the measurements
VL53L0X_RangingMeasurementData_t left_dist;
VL53L0X_RangingMeasurementData_t right_dist;
VL53L0X_RangingMeasurementData_t front_dist;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  delay(50);
  digitalWrite(SHT_LOX2, HIGH);
  delay(50);
  digitalWrite(SHT_LOX3, HIGH);
  delay(50);

  // activating LOX1 and resetting both LOX2 and LOX3
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

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
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }
}

int readLeft(){
  static int distanceLeft = 8190; // Initialize to a safe "open" distance!
  
  if (lox3.isRangeComplete()) {
    uint16_t reading = lox3.readRange();
    lox3.clearInterruptMask(false);
    
    // Filter Adafruit's out-of-range infinity error codes
    if (reading > 8000) {
      distanceLeft = 1000;
    } else {
      distanceLeft = reading;
    }
  }
  return distanceLeft;
}

int readRight(){
  static int distanceRight = 8190; // Initialize to a safe "open" distance!
  
  if (lox1.isRangeComplete()) {
    uint16_t reading = lox1.readRange();
    lox1.clearInterruptMask(false);
    
    // Filter Adafruit's out-of-range infinity error codes
    if (reading > 8000) {
      distanceRight = 1000;
    } else {
      distanceRight = reading;
    }
  }
  return distanceRight;
}

int readFront(){
  static int distanceFront = 8190; // Initialize to a safe "open" distance!
  
  if (lox2.isRangeComplete()) {
    uint16_t reading = lox2.readRange();
    lox2.clearInterruptMask(false);
    
    // Filter Adafruit's out-of-range infinity error codes
    if (reading > 8000) {
      distanceFront = 1000;
    } else {
      distanceFront = reading;
    }
  }
  return distanceFront;
}

void readAll() {
  
  lox1.rangingTest(&right_dist, false);  
  lox2.rangingTest(&front_dist, false);  
  lox3.rangingTest(&left_dist, false);

  /*
  // print sensor one reading
  Serial.print(F("1: "));
  if(left_dist.RangeStatus != 4) {     // if not out of range
    Serial.print(left_dist.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(right_dist.RangeStatus != 4) {
    Serial.print(right_dist.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));

  // print sensor three reading
  Serial.print(F("3: "));
  if(front_dist.RangeStatus != 4) {
    Serial.print(front_dist.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
  */
}

void ToF_setup() {
  // wait until serial port opens for native USB devices
  //while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  Serial.println(F("All three in reset mode...(pins are low)"));
  
  Serial.println(F("Starting..."));
  setID();

  lox1.startRangeContinuous();
  lox2.startRangeContinuous();
  lox3.startRangeContinuous();
 
}

void ToF_test() {
  const int LED_LEFT_PIN = A0;   
  const int LED_FRONT_PIN = A6;
  const int LED_RIGHT_PIN = A7;  

  digitalWrite(LED_LEFT_PIN, LOW);
  digitalWrite(LED_FRONT_PIN, LOW);
  digitalWrite(LED_RIGHT_PIN, LOW);

  const int THRESHOLD_MM = 100;

  Serial.println(F("--- STARTING TOF DIAGNOSTIC ---"));

  // 1. Test LEFT Sensor (lox3)
  Serial.println(F("Waiting for wall on LEFT sensor..."));
  while(true) {
    if (lox3.isRangeComplete()) {
      left_dist.RangeMilliMeter = lox3.readRange();
      lox3.clearInterruptMask(false);
      
      if (left_dist.RangeMilliMeter < THRESHOLD_MM) {
        digitalWrite(LED_LEFT_PIN, HIGH);
        Serial.println(F("Left Sensor: OK!"));
        delay(1000); 
        break;
      }
    }
  }

  // 2. Test FRONT Sensor (lox2)
  Serial.println(F("Waiting for wall on FRONT sensor..."));
  while(true) {  
    if (lox2.isRangeComplete()) {
      front_dist.RangeMilliMeter = lox2.readRange();
      lox2.clearInterruptMask(false);
      
      if (front_dist.RangeMilliMeter < THRESHOLD_MM) {
        digitalWrite(LED_FRONT_PIN, HIGH);
        Serial.println(F("Front Sensor: OK!"));
        delay(1000);
        break;
      }
    }
  }

  // 3. Test RIGHT Sensor (lox1)
  Serial.println(F("Waiting for wall on RIGHT sensor..."));
  while(true) {
    if (lox1.isRangeComplete()) {
      right_dist.RangeMilliMeter = lox1.readRange();
      lox1.clearInterruptMask(false);
      
      if (right_dist.RangeMilliMeter < THRESHOLD_MM) {
        digitalWrite(LED_RIGHT_PIN, HIGH);
        Serial.println(F("Right Sensor: OK!"));
        delay(1000);
        break;
      }
    }
  }

  Serial.println(F("--- TOF DIAGNOSTIC COMPLETE ---"));
  
  for(int i = 0; i < 2; i++) {
    digitalWrite(LED_LEFT_PIN, LOW);
    digitalWrite(LED_FRONT_PIN, LOW);
    digitalWrite(LED_RIGHT_PIN, LOW);
    delay(200);
    digitalWrite(LED_LEFT_PIN, HIGH);
    digitalWrite(LED_FRONT_PIN, HIGH);
    digitalWrite(LED_RIGHT_PIN, HIGH);
    delay(200);
  }

  digitalWrite(LED_LEFT_PIN, LOW);
  digitalWrite(LED_FRONT_PIN, LOW);
  digitalWrite(LED_RIGHT_PIN, LOW);
}

