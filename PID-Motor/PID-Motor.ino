// === Control PID de posición para puerta que SOLO se mueve hacia el sensor ===

// Pines
const int TRIG_PIN   = 13;
const int ECHO_PIN   = 12;
const int MOTOR_PIN  = 9;

const int BUTTON_PID   = 2;  // Activa/desactiva PID
const int BUTTON_START = 3;  // Inicia/detiene motor

// PID
float Kp = 5.0;
float Ki = 0.8;
float Kd = 0.3;

float setpoint = 6.0;      
float posicion_actual = 0;  

float error = 0;
float last_error = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// Límites
const float INTEGRAL_MAX = 200.0;
const int PWM_MAX = 255;

// Tiempos
const unsigned long SAMPLE_TIME = 50;
unsigned long lastSampleTime = 0;

// Debounce (sin delay)
unsigned long lastDebouncePID   = 0;
unsigned long lastDebounceStart = 0;
const unsigned long DEBOUNCE_MS = 50;

// Estados
bool pid_enabled   = false;
bool motor_running = false;

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  pinMode(BUTTON_PID,   INPUT_PULLUP);
  pinMode(BUTTON_START, INPUT_PULLUP);

  analogWrite(MOTOR_PIN, 0);

  posicion_actual = readDistanceCM();

  Serial.println("=== PID 1 sentido — Sistema listo ===");
  Serial.print("Posición inicial: ");
  Serial.println(posicion_actual);
}

void loop() {

  //---------------------------------------------------------------------------
  // BOTÓN START (sin delay)
  //---------------------------------------------------------------------------
  static bool last_start = HIGH;
  bool start_btn = digitalRead(BUTTON_START);

  if (start_btn == LOW && last_start == HIGH && millis() - lastDebounceStart > DEBOUNCE_MS) {

    lastDebounceStart = millis();

    motor_running = !motor_running;
    pid_enabled = false;
    integral = 0;
    last_error = 0;

    if (motor_running) {
      Serial.println("\n>>> MOTOR INICIADO (SIN PID) <<<");
    } else {
      Serial.println("\n>>> MOTOR DETENIDO <<<");
      analogWrite(MOTOR_PIN, 0);
    }
  }
  last_start = start_btn;



  //---------------------------------------------------------------------------
  // BOTÓN PID (sin delay)
  //---------------------------------------------------------------------------
  static bool last_button = HIGH;
  bool b = digitalRead(BUTTON_PID);

  if (b == LOW && last_button == HIGH && millis() - lastDebouncePID > DEBOUNCE_MS) {

    lastDebouncePID = millis();

    if (motor_running) {
      pid_enabled = !pid_enabled;

      if (pid_enabled) {
        Serial.println("\n>>> PID ACTIVADO <<<");
        integral = 0;
        last_error = error;   // sincroniza el PID para no generar picos
      } else {
        Serial.println("\n>>> PID DESACTIVADO <<<");
        integral = 0;
        last_error = 0;
      }
    } else {
      Serial.println("\n[WARN] Inicie el motor antes de activar PID.");
    }
  }
  last_button = b;



  //---------------------------------------------------------------------------
  // LECTURA DEL SENSOR
  //---------------------------------------------------------------------------
  posicion_actual = readDistanceCM();

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_TIME) {

    lastSampleTime = now;

    error = posicion_actual - setpoint;

    // PARO AUTOMÁTICO AL LLEGAR
    if (motor_running && error <= 0) {
      motor_running = false;
      pid_enabled = false;
      analogWrite(MOTOR_PIN, 0);
      Serial.println("\n>>> LLEGÓ AL SETPOINT — MOTOR DETENIDO <<<");
      printStatus();
      return;
    }

    if (!motor_running) {
      analogWrite(MOTOR_PIN, 0);
      printStatus();
      return;
    }


    //---------------------------------------------------------------------------
    // MODO PID
    //---------------------------------------------------------------------------
    if (pid_enabled && error > 0) {

      float dt = SAMPLE_TIME / 1000.0;

      float P = Kp * error;

      integral += error * dt;
      if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
      if (integral < 0)            integral = 0;

      float I = Ki * integral;

      derivative = (error - last_error) / dt;
      float D = Kd * derivative;

      output = P + I + D;

      if (output < 0) output = 0;
      if (output > PWM_MAX) output = PWM_MAX;

      last_error = error;

      analogWrite(MOTOR_PIN, (int)output);
    }


    //---------------------------------------------------------------------------
    // MODO SIN PID — PWM máximo
    //---------------------------------------------------------------------------
    else if (!pid_enabled && motor_running) {
      output = 255;
      analogWrite(MOTOR_PIN, 255);
    }

    printStatus();
  }
}



// ============================================================================
// SENSOR ULTRASONIDO
// ============================================================================
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration <= 0) return posicion_actual;

  float dist = duration * 0.0343 / 2.0;

  if (dist < 0) dist = 0;
  if (dist > 100) dist = 100;

  return dist;
}



// ============================================================================
// DEBUG
// ============================================================================
void printStatus() {
  Serial.print("RUN:");
  Serial.print(motor_running ? "YES" : "NO");
  Serial.print(" | PID:");
  Serial.print(pid_enabled ? "ON" : "OFF");
  Serial.print(" | SP:");
  Serial.print(setpoint);
  Serial.print(" cm | Pos:");
  Serial.print(posicion_actual, 1);
  Serial.print(" cm | Err:");
  Serial.print(error, 1);
  Serial.print(" | PWM:");
  Serial.println(output);
}
