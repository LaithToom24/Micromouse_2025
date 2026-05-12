// --- PIN DEFINITIONS ---
// Standby Pin
const int STBY = 8;

// Left Motor (Motor A)
const int PWMA = 5;
const int AIN1 = 6;
const int AIN2 = 7;

// Right Motor (Motor B)
const int PWMB = 11;
const int BIN1 = 9;
const int BIN2 = 10;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Raw Motor Hardware Test...");

  // Set all motor control pins as outputs
  pinMode(STBY, OUTPUT);
  
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Wake up the motor driver
  digitalWrite(STBY, HIGH); 
}

void loop() {
  // --------------------------------------------------
  // TEST 1: BOTH MOTORS FORWARD (2 Seconds)
  // --------------------------------------------------
  Serial.println("Moving Forward...");
  
  // Left Motor Forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 150); // ~60% power

  // Right Motor Forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 150); // ~60% power
  
  delay(2000);

  // --------------------------------------------------
  // TEST 2: COAST / STOP (1 Second)
  // --------------------------------------------------
  Serial.println("Stopping...");
  
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  
  delay(1000);

  // --------------------------------------------------
  // TEST 3: BOTH MOTORS REVERSE (2 Seconds)
  // --------------------------------------------------
  Serial.println("Moving Reverse...");
  
  // Left Motor Reverse
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 150); 

  // Right Motor Reverse
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 150); 
  
  delay(2000);

  // --------------------------------------------------
  // TEST 4: COAST / STOP (1 Second)
  // --------------------------------------------------
  Serial.println("Stopping...");
  
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  
  delay(1000);
}