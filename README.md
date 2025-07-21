# tpi-auto-UTN

```
// Pines motores
int in1 = 12, in2 = 13, enA = 6;
int in3 = 7,  in4 = 11, enB = 5;

// Pines sensores
int sensori = 8, sensorm = 4, sensord = 2;

// PID
const int velConst = 150;
const float kp = 25.0, ki = 0.3, kd = 15.0;
float Iacum = 0, p_old = 0;
unsigned long lastTime;

void setup() {
  Serial.begin(9600);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(enA, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(sensori, INPUT_PULLUP);
  pinMode(sensorm, INPUT_PULLUP);
  pinMode(sensord, INPUT_PULLUP);
  lastTime = millis();
}

void loop() {
  // 1) leer e invertir lógica
  bool onBlackI = (digitalRead(sensori) == LOW);
  bool onBlackM = (digitalRead(sensorm) == LOW);
  bool onBlackD = (digitalRead(sensord) == LOW);

  // 2) dt
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // 3) calcular p
  float p;
  if      (onBlackI && !onBlackM && !onBlackD)    p = -2;
  else if (!onBlackI && onBlackM && !onBlackD)    p =  0;
  else if (!onBlackI && !onBlackM && onBlackD)    p =  2;
  else if (onBlackI && onBlackM && !onBlackD)     p = -1;
  else if (!onBlackI && onBlackM && onBlackD)     p =  1;
  else {  // 3 sensores o ninguno
    p = (p_old > 0) ? 3 : -3;
  }

  // 4) PID
  Iacum += p * dt;
  Iacum = constrain(Iacum, -100, 100);     // anti‑windup
  float d = (p - p_old) / dt;
  p_old = p;

  float correction = kp*p + ki*Iacum + kd*d;

  // 5) velocidades
  int vI = constrain(velConst - (int)correction, 0, 255);
  int vD = constrain(velConst + (int)correction, 0, 255);

  // 6) aplicar a motores
  moverMotor(in1,in2,enA, vI);
  moverMotor(in3,in4,enB, vD);

  // debug
  Serial.print(p); Serial.print("  corr="); Serial.print(correction);
  Serial.print("  vI="); Serial.print(vI); Serial.print(" vD="); Serial.println(vD);
}

void moverMotor(int pinF, int pinR, int pinE, int vel) {
  digitalWrite(pinF, vel>0 ? HIGH : LOW);
  digitalWrite(pinR, vel>0 ? LOW  : HIGH);
  analogWrite(pinE, abs(vel));
}
  
```
