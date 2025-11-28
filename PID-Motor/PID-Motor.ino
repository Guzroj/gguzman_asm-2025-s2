// === Control PID de posición para puerta que SOLO se mueve hacia el sensor ===

// Pines
const int TRIG_PIN   = 13;
const int ECHO_PIN   = 12;
const int MOTOR_PIN  = 9;
const int BUTTON_PIN = 2;

// PID
float Kp = 5.0;
float Ki = 0.8;
float Kd = 0.3;

float setpoint = 25.0;      // distancia deseada en cm
float posicion_actual = 0;  // lectura del sensor

float error = 0;
float last_error = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// Límites
const float INTEGRAL_MAX = 200.0;
const int   PWM_MAX = 255;

// Tiempo de muestreo
const unsigned long SAMPLE_TIME = 50; // 50 ms
unsigned long lastSampleTime = 0;

// Estado
bool pid_enabled = false;

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  analogWrite(MOTOR_PIN, 0);

  // Leer posición inicial
  delay(200);
  posicion_actual = readDistanceCM();
  Serial.println("=== PID 1 sentido (siempre hacia el sensor) ===");
  Serial.print("Posición inicial: ");
  Serial.print(posicion_actual);
  Serial.println(" cm");
}

void loop() {
  // Botón: toggle PID
  static bool last_button = HIGH;
  bool b = digitalRead(BUTTON_PIN);
  if (b == LOW && last_button == HIGH) {
    delay(50); // debounce simple
    pid_enabled = !pid_enabled;
    if (pid_enabled) {
      Serial.println("\n>>> PID ACTIVADO <<<");
    } else {
      Serial.println("\n>>> PID DESACTIVADO <<<");
      analogWrite(MOTOR_PIN, 0);
      integral = 0;
      last_error = 0;
    }
  }
  last_button = b;

  // Leer sensor siempre
  posicion_actual = readDistanceCM();

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_TIME) {
    lastSampleTime = now;

    // error definido PARA 1 SOLO SENTIDO:
    // positivo = estoy más lejos que el setpoint -> hay que mover
    error = posicion_actual - setpoint;

    if (pid_enabled && error > 0) {
      // --- PID clásico ---
      float dt = SAMPLE_TIME / 1000.0;

      float P = Kp * error;

      integral += error * dt;
      if (integral > INTEGRAL_MAX)  integral = INTEGRAL_MAX;
      if (integral < 0)             integral = 0;  // no acumular si ya está muy cerca o pasándose

      float I = Ki * integral;

      derivative = (error - last_error) / dt;
      float D = Kd * derivative;

      output = P + I + D;

      // sólo valores positivos y dentro de 0..255
      if (output < 0)      output = 0;
      if (output > PWM_MAX) output = PWM_MAX;

      last_error = error;

      // Aplicar PWM: SIEMPRE hacia el sensor
      analogWrite(MOTOR_PIN, (int)output);
    } else {
      // Ya llegó o PID apagado: motor off
      output = 0;
      analogWrite(MOTOR_PIN, 0);
      if (!pid_enabled) {
        integral = 0;
        last_error = 0;
      }
    }

    // Monitoreo
    printStatus();
  }
}

// === Lectura de distancia en cm (HC-SR04) ===
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration <= 0) return posicion_actual; // conservar valor anterior si falla

  float dist = duration * 0.0343 / 2.0;

  // Limitar por seguridad (ajusta si tu carril es diferente)
  if (dist < 0) dist = 0;
  if (dist > 100) dist = 100;

  return dist;
}

// === Debug por Serial ===
void printStatus() {
  Serial.print("PID:");
  Serial.print(pid_enabled ? "ON " : "OFF");
  Serial.print(" | SP:");
  Serial.print(setpoint, 1);
  Serial.print("cm | Pos:");
  Serial.print(posicion_actual, 1);
  Serial.print("cm | Err:");
  Serial.print(error, 1);
  Serial.print(" | PWM:");
  Serial.println((int)output);
}
