#define ENCODER_L 2
#define ENCODER_R 3
#define MOTORL_A 4
#define MOTORL_B 5
#define MOTORR_A 6
#define MOTORR_B 7
#define ENABLE_L 8
#define ENABLE_R 9
#define LOWER_LIMIT -128
#define UPPER_LIMIT 127

volatile long counter_l = 0, counter_r = 0;
long p_counter_l = 0, p_counter_r = 0;
long prev_time;
long speed_l = 0, speed_r = 0;
byte motor_l = 0, motor_r = 0;
boolean dir_l, dir_r;

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
  pinMode(MOTORL_A, HIGH);
  pinMode(MOTORL_B, HIGH);
  pinMode(MOTORR_A, HIGH);
  pinMode(MOTORR_B, HIGH);
  // Activate ISR for both encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), right_encoder, RISING);
  Serial.begin(9600);
  prev_time = millis();
}

void loop() {
  
  // Read serial data for motors
  if (Serial.available()) {
    motor_l = Serial.parseInt();
    motor_r = Serial.parseInt();
    Serial.flush();
  }

  // Calculate motor direction. Positive if value is larger than 0
  dir_l = motor_l >= 0 ? true : false;
  dir_r = motor_r >= 0 ? true : false;

  // Control both motor directions
  dirMotors(dir_l, dir_r);

  // Map motors outputs from [-128, 127] to [0, 255]
  motor_l = clamp_motor(motor_l) + 128;
  motor_r = clamp_motor(motor_r) + 128;
  
  // Set motor speed
  analogWrite(ENABLE_L, abs(motor_l));
  analogWrite(ENABLE_R, abs(motor_r));

  // Calculate time step
  int delta_t = millis() - prev_time;
  
  // Update encoder speeds
  speed_l = (int) (counter_l - p_counter_l) / delta_t;
  speed_r = (int) (counter_r - p_counter_r) / delta_t;
  
  // Send encoder data
  Serial.print(speed_l);
  Serial.print(',');
  Serial.print(speed_r);
  Serial.print(',');
  Serial.println(delta_t);
  
  // Record previous counter and time
  p_counter_l = counter_l;
  p_counter_r = counter_r;
  prev_time = millis();
}

