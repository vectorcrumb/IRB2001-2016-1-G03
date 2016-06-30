#define IN1 6  //DERECHA
#define IN2 7  //DERECHA
#define IN3 4  //IZQUIERDA
#define IN4 5  //IZQUIERDA
#define ENA 8
#define ENB 9
#define VALOR_ADELANTE 127
#define VALOR_ATRAS -127
#define BUF 5
#define Kp 1.3

byte incoming[BUF];
long dir_left = 1, dir_right = 1;

void mover(int izq, int der, int dir_izq, int dir_der) {
  int error = 0;
  if (dir_izq != dir_der) {
    // En un giro, se va a 0 y el ajuste se anula.
    error = speed_l + speed_r;
  } else {
    error = speed_l - speed_r;
  }
  int adjust = error * Kp
  int mapizq = map(abs(izq), 0,100, 0,255);
  int mapder = map(abs(der), 0,100, 0,255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  if (dir_der == 1){
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
  }
  if (dir_der == 2){
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
  }
  if (dir_izq == 1){
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
  }
  if (dir_izq == 2){
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
  }
  // Invertir signos en caso que se acentue el error
  analogWrite(ENA - adjust, mapder);
  analogWrite(ENB + adjust, mapizq);  
}

const byte encoderL = 2;
const byte encoderR = 3;
volatile unsigned long stepsL;
volatile unsigned long stepsR;

void stepL()
{
  stepsL += dir_left == 1 ? 1 : -1;
}
void stepR()
{
  stepsR += dir_right == 1 ? 1 : -1;
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(encoderL),stepL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderR),stepR,CHANGE);
  Serial1.begin(9600);
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop(){
  if (Serial1.available()){
    Serial1.readBytes(incoming, BUF);
    long motor_left = incoming[0];
    long motor_right = incoming[1];
    dir_left = incoming[2];
    dir_right = incoming[3];

    //+, - gira a la derecha
    mover(motor_left, motor_right, dir_left, dir_right);
    Serial.print(motor_left);
    Serial.print(',');
    Serial.print(motor_right);
    Serial.print(";");
    Serial.print(dir_left);
    Serial.print(",");
    Serial.println(dir_right);
    Serial1.flush();
    Serial.flush();
  }
  int delta_t = millis() - prev_time;
  // Update encoder speeds
  speed_l = (int) counter_l / delta_t;
  speed_r = (int) counter_r / delta_t;
  // Record values as previous values
  counter_l = 0;
  counter_r = 0;
  prev_time = millis();
}
