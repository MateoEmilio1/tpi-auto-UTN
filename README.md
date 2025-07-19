# tpi-auto-UTN

```
// Sensores
const int sensorI = 8;
const int sensorM = 4;
const int sensorD = 2;


// Motor izquierdo
const int mPin1 = 12;
const int mPin2 = 13;
const int enablePin1y2 = 6;

// Motor derecho
const int mPin3 = 7;
const int mPin4 = 11;
const int enablePin3y4 = 5;

// Parámetros del controlador PID
const float Kp = 10, Ki = 0.01, Kd = 10;
int P, I, D;
int ultimoError = 0;
int velocidad = 255;
void setup() {
  pinMode(sensorI, INPUT);
  pinMode(sensorM, INPUT);
  pinMode(sensorD, INPUT);
  pinMode(mPin1, OUTPUT);
  pinMode(mPin2, OUTPUT);
  pinMode(enablePin1y2, OUTPUT);
  pinMode(mPin3, OUTPUT);
  pinMode(mPin4, OUTPUT);
  pinMode(enablePin3y4, OUTPUT);

  digitalWrite(mPin1, LOW);
  digitalWrite(mPin2, LOW);
  digitalWrite(enablePin1y2, LOW);
  digitalWrite(mPin3, LOW);
  digitalWrite(mPin4, LOW);
  digitalWrite(enablePin3y4, LOW);
}
void loop() {
  int inputMedio = digitalRead(sensorM);
  int inputDerecha = digitalRead(sensorD);
  int inputIzquierda = digitalRead(sensorI);
  int pos = posicion(inputIzquierda, inputMedio, inputDerecha);
  int error = pos - 1000;
  P = error;
  I = I + error;
  D = error - ultimoError;
  ultimoError = error;
  int PID = P * Kp + I * Ki + D * Kd;
  int motorD = velocidad + PID;
  int motorI = velocidad - PID;
  if (motorD > 200) motorD = 200;
  if (motorI > 200) motorI = 200;
  if (motorD < -100) motorD = -100;
  if (motorI < -100) motorI = -100;

  cambiar_velocidad(motorD, motorI);
}
void cambiar_velocidad(int velD, int velI) {
  if (velD < 0) {
	digitalWrite(mPin1, LOW);
	digitalWrite(mPin2, HIGH);
	velD = -velD;
  } else {
	digitalWrite(mPin1, HIGH);
	digitalWrite(mPin2, LOW);
  }
  if (velI < 0) {
	digitalWrite(mPin3, LOW);
	digitalWrite(mPin4, HIGH);
	velI = -velI;
  } else {
	digitalWrite(mPin3, HIGH);
	digitalWrite(mPin4, LOW);
  }
  analogWrite(enablePin1y2, velD);  
  analogWrite(enablePin3y4, velI);
}
int posicion(int i, int c, int d) {
  if (i == LOW && c == HIGH && d == LOW) return 1000;  // Detecta el sensor del medio (mantener recto)
  if (i == LOW && c == LOW && d == HIGH) return 2000;  // Detecta el sensor derecho
  if (i == HIGH && c == LOW && d == LOW) return 0; 	// Detecta el sensor de la izquierda
  if (i == LOW && c == HIGH && d == HIGH) return 1500; // Detectan los sensores derecho y del medio
  if (i == HIGH && c == HIGH && d == LOW) return 500;  // Detectan los sensores izquierdo y del medio
  if (i == HIGH && c == LOW && d == HIGH) return 1000; // Detectan los sensores izquierdo y derecho (mantener recto)
  if (i == LOW && c == LOW && d == LOW) return 1000;   // Ningún sensor detecta (mantener recto)
  return 1000;                                      	// Todos los sensores detectan (mantener recto)
}

```
