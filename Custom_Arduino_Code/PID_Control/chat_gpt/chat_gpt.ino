// --- Pin Definitions ---
const int ENCODER_LEFT_A  = 2;
const int ENCODER_RIGHT_A = 3;
const int ENCODER_LEFT_B  = 4;
const int ENCODER_RIGHT_B = 5;

const int MOTOR_LEFT_DIR  = 7;
const int MOTOR_RIGHT_DIR = 8;
const int MOTOR_LEFT_PWM  = 9;
const int MOTOR_RIGHT_PWM = 10;

// --- Globals ---
volatile long left_encoderCount  = 0;
volatile long right_encoderCount = 0;

// --- Robot Parameters ---
float wheel_diameter_m      = 0.03;                   // 3 cm
float wheel_circumference_m = 3.1416f * wheel_diameter_m;
int encoder_ticks_per_rev   = 12;                    // adjust per encoder
float gear_ratio            = 10;                    // adjust if motor geared

unsigned long last_time_ms  = 0;
unsigned long sample_period_ms = 500;                 // sampling interval

float left_velocity_mps  = 0.0;
float right_velocity_mps = 0.0;

// --- Previous States for Quadrature Decoding ---
volatile int lastLeftA  = 0;
volatile int lastLeftB  = 0;
volatile int lastRightA = 0;
volatile int lastRightB = 0;

// --- Setup ---
void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), readRightEncoder, CHANGE);

  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);

  analogWrite(MOTOR_LEFT_PWM, 100);   // moderate speed
  analogWrite(MOTOR_RIGHT_PWM, 100);

  last_time_ms = millis();

  Serial.println("System Initialized");
}

// --- Loop ---
void loop() {
  unsigned long now = millis();
  if (now - last_time_ms >= sample_period_ms) {
    noInterrupts();
    long left_count  = left_encoderCount;
    long right_count = right_encoderCount;
    left_encoderCount  = 0;
    right_encoderCount = 0;
    interrupts();

    float dt_s = (now - last_time_ms) / 1000.0f;
    last_time_ms = now;

    float left_revs  = (left_count  / (float)encoder_ticks_per_rev) / gear_ratio;
    float right_revs = (right_count / (float)encoder_ticks_per_rev) / gear_ratio;

    left_velocity_mps  = left_revs  * wheel_circumference_m / dt_s;
    right_velocity_mps = right_revs * wheel_circumference_m / dt_s;

    Serial.print("Left velocity (m/s): ");
    Serial.println(left_velocity_mps, 4);
    Serial.print("Right velocity (m/s): ");
    Serial.println(right_velocity_mps, 4);
  }
}

// --- Quadrature Decoding Functions ---
void readLeftEncoder() {
  int A = digitalRead(ENCODER_LEFT_A);
  int B = digitalRead(ENCODER_LEFT_B);
  int encoded = (A << 1) | B;
  int sum = (lastLeftA << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    left_encoderCount++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    left_encoderCount--;
  lastLeftA = A;
  lastLeftB = B;
}

void readRightEncoder() {
  int A = digitalRead(ENCODER_RIGHT_A);
  int B = digitalRead(ENCODER_RIGHT_B);
  int encoded = (A << 1) | B;
  int sum = (lastRightA << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    right_encoderCount++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    right_encoderCount--;
  lastRightA = A;
  lastRightB = B;
}
