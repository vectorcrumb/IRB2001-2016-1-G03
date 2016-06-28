#define IN1 6  //DERECHA
#define IN2 7  //DERECHA
#define IN3 4  //IZQUIERDA
#define IN4 5  //IZQUIERDA
#define ENA 8
#define ENB 9
#define VALOR_ADELANTE 127
#define VALOR_ATRAS -127
#define BUF 4

byte incoming[BUF];

void mover(int izq, int der, int dir_izq, int dir_der) {
  int mapizq = map(abs(izq), 0,100, 0,255);
  int mapder = map(abs(der), 0,100, 0,255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW); // parte todo en cero.
  
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
  
  analogWrite(ENA, mapder);
  analogWrite(ENB, mapizq);  
}
void avanzar() {
  mover(100, 100, 1,1);
}

void retroceder() {
  mover(100, 100, 1, 1);
}

void reloj() {
  mover(100, 100, 1, 2);
}

void contrareloj() {
  mover(100, 100, 2, 1);
}

void detenerse() {
  mover(0, 0, 0, 0);
}

void setup() {
  // put your setup code here,   to run once:
  Serial1.begin(9600);
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop(){
  // put your main code here, to run repeatedly:
  if (Serial1.available()){
    Serial1.readBytes(incoming, BUF);
    long motor_left = incoming[0];
    long motor_right = incoming[1];
    long dir_left = incoming[2];
    long dir_right = incoming[3];
    if (motor_left>100 or motor_left<-100){
      motor_left = 0;
    }
    if (motor_right>100 or motor_right<-100){
      motor_right = 0;
    }

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
}
