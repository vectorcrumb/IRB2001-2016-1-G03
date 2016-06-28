#define ENCODER_L 2
#define ENCODER_R 3
#define MOTORL_A 4
#define MOTORL_B 5
#define MOTORR_A 6
#define MOTORR_B 7
#define ENABLE_L 8
#define ENABLE_R 9
#define LED 13
#define LOWER_LIMIT -127
#define UPPER_LIMIT 127
#define DEBUG 500

volatile long counter_l = 0, counter_r = 0;
long p_counter_l = 0, p_counter_r = 0;
long prev_time;
long debug_time;
long speed_l = 0, speed_r = 0;
int motor_l = 0, motor_r = 0;
boolean dir_l = true, dir_r = false;
boolean initialize = false;
int p_motor_l = 0, p_motor_r = 0;

void dirMotors(boolean left_motor_dir, boolean right_motor_dir) {
  digitalWrite(MOTORL_A, left_motor_dir);
  digitalWrite(MOTORL_B, !left_motor_dir);
  digitalWrite(MOTORR_A, right_motor_dir);
  digitalWrite(MOTORR_B, !right_motor_dir);
}

int clamp(int value, int l_limit, int u_limit) {
  if (value <= l_limit) {
    return l_limit;
  } else if (value >= u_limit) {
    return u_limit;
  } return value;
}

int clamp_motor(int value) {
  return clamp(value, LOWER_LIMIT, UPPER_LIMIT);
}

void left_encoder() {
  // ISR for left encoder. Increment by 1 if direction is positive
  // or decrement by 1 if direction is negative.
  counter_l += dir_l ? 1 : -1;
}

void right_encoder() {
  // ISR for right encoder. Increment by 1 if direction is positive
  // or decrement by 1 if direction is negative.
  counter_r += dir_r ? 1 : -1;
}

void setup() {
  // Configure direction pins
  pinMode(MOTORL_A, OUTPUT);
  pinMode(MOTORL_B, OUTPUT);
  pinMode(MOTORR_A, OUTPUT);
  pinMode(MOTORR_B, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  // Set motor values to 0 for startup
  motor_l = 0;
  motor_r = 0;
  // Activate ISR for both encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), right_encoder, RISING);
  Serial.begin(9600);
  prev_time = millis();
  debug_time = millis();
  initialize = true;
}

void loop() {
  
  // Read serial data for motors
  if (Serial.available()) {
    if (initialize) {
      motor_l = clamp_motor(Serial.parseInt());
      motor_r = clamp_motor(Serial.parseInt());
      Serial.print(motor_l);
      Serial.print(",");
      Serial.println(motor_r);
    }
    else {
      byte incoming = Serial.read();
      if (incoming == 105){
        digitalWrite(LED, HIGH);
        initialize = true;
        Serial.println("$$$");
      }
    }
  }

  // Control both motor directions
  dirMotors(dir_l, dir_r);

  // Map motors outputs from [-128, 127] to [0, 255]
  if(p_motor_l != motor_l) {
    dir_l = motor_l >= 0 ? false : true;
    motor_l = map(abs(motor_l), 0, UPPER_LIMIT, 0, 255);
  }
  if(p_motor_r != motor_r) {
    dir_r = motor_r >= 0 ? false : true;
    motor_r = map(abs(motor_r), 0, UPPER_LIMIT, 0, 255);
  }
  
  // Set motor speed
  analogWrite(ENABLE_L, motor_l);
  analogWrite(ENABLE_R, motor_r);

  // Calculate time step
  int delta_t = millis() - prev_time;
  
  // Update encoder speeds
  speed_l = (int) (counter_l - p_counter_l) / delta_t;
  speed_r = (int) (counter_r - p_counter_r) / delta_t;
  


//  if (millis() - debug_time > DEBUG) {
//    Serial.print("New: ");
//    Serial.print(dir_l ? "-" : "");
//    Serial.print(motor_l);
//    Serial.print(", ");
//    Serial.print(dir_r ? "-" : "");
//    Serial.print(motor_r);
//    Serial.print("\t\t");
//    Serial.print(counter_l);
//    Serial.print(", ");
//    Serial.println(counter_r);
//    debug_time = millis();
//  }
  
  // Record previous counter and time
  p_counter_l = counter_l;
  p_counter_r = counter_r;
  p_motor_l = motor_l;
  p_motor_r = motor_r;
  prev_time = millis();
}

