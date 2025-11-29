// === Control PID bidireccional para puerta con L293D ===

// Pines sensor
const int TRIG_PIN   = 13;
const int ECHO_PIN   = 12;

// Pines L293D (Puente H)
const int MOTOR_IN1  = 9;   // Control direcciÃ³n 1
const int MOTOR_IN2  = 10;  // Control direcciÃ³n 2
const int MOTOR_EN   = 11;  // Enable (PWM)

// Botones
const int BUTTON_PID      = 2;  // Activa/desactiva PID
const int BUTTON_FORWARD  = 3;  // Motor adelante (alejar del sensor)
const int BUTTON_BACKWARD = 4;  // Motor atrÃ¡s (acercar al sensor)

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

// LÃ­mites
const float INTEGRAL_MAX = 200.0;
const int PWM_MAX = 255;

// Tiempos
const unsigned long SAMPLE_TIME = 50;
unsigned long lastSampleTime = 0;

// Debounce
unsigned long lastDebouncePID = 0;
unsigned long lastDebounceForward = 0;
unsigned long lastDebounceBackward = 0;
const unsigned long DEBOUNCE_MS = 50;

// Estados
bool pid_enabled   = false;
bool motor_running = false;
int motor_direction = 0;  // 0=parado, 1=adelante, -1=atrÃ¡s

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);

  pinMode(BUTTON_PID,      INPUT_PULLUP);
  pinMode(BUTTON_FORWARD,  INPUT_PULLUP);
  pinMode(BUTTON_BACKWARD, INPUT_PULLUP);

  stopMotor();

  posicion_actual = readDistanceCM();

  Serial.println("=== PID Bidireccional â€” Sistema listo ===");
  Serial.print("PosiciÃ³n inicial: ");
  Serial.print(posicion_actual);
  Serial.println(" cm");
}

void loop() {

  //---------------------------------------------------------------------------
  // BOTÃ“N ADELANTE (alejar del sensor)
  //---------------------------------------------------------------------------
  static bool last_forward = HIGH;
  bool forward_btn = digitalRead(BUTTON_FORWARD);

  if (forward_btn == LOW && last_forward == HIGH && 
      millis() - lastDebounceForward > DEBOUNCE_MS) {
    
    lastDebounceForward = millis();

    motor_running = !motor_running;
    pid_enabled = false;
    integral = 0;
    last_error = 0;

    if (motor_running) {
      motor_direction = 1;
      Serial.println("\n>>> MOTOR ADELANTE (SIN PID) <<<");
      setMotorForward(255);
    } else {
      motor_direction = 0;
      Serial.println("\n>>> MOTOR DETENIDO <<<");
      stopMotor();
    }
  }
  last_forward = forward_btn;


  //---------------------------------------------------------------------------
  // BOTÃ“N ATRÃS (acercar al sensor)
  //---------------------------------------------------------------------------
  static bool last_backward = HIGH;
  bool backward_btn = digitalRead(BUTTON_BACKWARD);

  if (backward_btn == LOW && last_backward == HIGH && 
      millis() - lastDebounceBackward > DEBOUNCE_MS) {
    
    lastDebounceBackward = millis();

    motor_running = !motor_running;
    pid_enabled = false;
    integral = 0;
    last_error = 0;

    if (motor_running) {
      motor_direction = -1;
      Serial.println("\n>>> MOTOR ATRÃS (SIN PID) <<<");
      setMotorBackward(255);
    } else {
      motor_direction = 0;
      Serial.println("\n>>> MOTOR DETENIDO <<<");
      stopMotor();
    }
  }
  last_backward = backward_btn;


  //---------------------------------------------------------------------------
  // BOTÃ“N PID
  //---------------------------------------------------------------------------
  static bool last_pid = HIGH;
  bool pid_btn = digitalRead(BUTTON_PID);

  if (pid_btn == LOW && last_pid == HIGH && 
      millis() - lastDebouncePID > DEBOUNCE_MS) {
    
    lastDebouncePID = millis();

    if (motor_running) {
      pid_enabled = !pid_enabled;

      if (pid_enabled) {
        Serial.println("\n>>> PID ACTIVADO <<<");
        integral = 0;
        last_error = error;
      } else {
        Serial.println("\n>>> PID DESACTIVADO <<<");
        integral = 0;
        last_error = 0;
      }
    } else {
      Serial.println("\n[WARN] Inicie el motor antes de activar PID.");
    }
  }
  last_pid = pid_btn;


  //---------------------------------------------------------------------------
  // LECTURA Y CONTROL
  //---------------------------------------------------------------------------
  posicion_actual = readDistanceCM();

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_TIME) {
    
    lastSampleTime = now;

    error = posicion_actual - setpoint;

    // PARO AUTOMÃTICO AL LLEGAR (solo en modo atrÃ¡s hacia el sensor)
    if (motor_running && motor_direction == -1 && error <= 0) {
      motor_running = false;
      pid_enabled = false;
      motor_direction = 0;
      stopMotor();
      Serial.println("\n>>> LLEGÃ“ AL SETPOINT â€” MOTOR DETENIDO <<<");
      printStatus();
      return;
    }

    if (!motor_running) {
      stopMotor();
      printStatus();
      return;
    }


    //---------------------------------------------------------------------------
    // MODO PID (solo funciona yendo hacia el sensor, motor_direction = -1)
    //---------------------------------------------------------------------------
    if (pid_enabled && motor_direction == -1 && error > 0) {

      float dt = SAMPLE_TIME / 1000.0;

      float P = Kp * error;

      integral += error * dt;
      if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
      if (integral < 0) integral = 0;

      float I = Ki * integral;

      derivative = (error - last_error) / dt;
      float D = Kd * derivative;

      output = P + I + D;

      if (output < 0) output = 0;
      if (output > PWM_MAX) output = PWM_MAX;

      last_error = error;

      setMotorBackward((int)output);
    }


    //---------------------------------------------------------------------------
    // MODO SIN PID â€” PWM mÃ¡ximo en la direcciÃ³n establecida
    //---------------------------------------------------------------------------
    else if (!pid_enabled && motor_running) {
      output = 255;
      if (motor_direction == 1) {
        setMotorForward(255);
      } else if (motor_direction == -1) {
        setMotorBackward(255);
      }
    }

    printStatus();
  }
}


// ============================================================================
// FUNCIONES DE CONTROL DEL MOTOR
// ============================================================================

void setMotorForward(int speed) {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN, speed);
}

void setMotorBackward(int speed) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_EN, speed);
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN, 0);
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
  Serial.print(" | DIR:");
  if (motor_direction == 1) Serial.print("FWD");
  else if (motor_direction == -1) Serial.print("BCK");
  else Serial.print("---");
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