// === Control PID bidireccional para puerta con puente H L293D ===

// Pines sensor
const int TRIG_PIN   = 13;
const int ECHO_PIN   = 12;

// Pines L293D (Puente H)
const int MOTOR_IN1  = 9;   // Control dirección 1
const int MOTOR_IN2  = 10;  // Control dirección 2
const int MOTOR_EN   = 11;  // Enable (PWM)

// Botones
const int BUTTON_PID      = 2;  // Activa/desactiva PID
const int BUTTON_FORWARD  = 3;  // Motor adelante (acercar al sensor)
const int BUTTON_BACKWARD = 4;  // Motor atrás (alejar del sensor)

// PID Original
float Kp = 5.0;
float Ki = 0.8;
float Kd = 0.3;

//float Kp = 12.0;
//float Ki = 0.3; //Baja más para máxima estabilidad
//float Kd = 4.0; //Frenado super brusco

float setpoint_forward = 4.0;   // Setpoint para ir adelante (cerca del sensor)
float setpoint_backward = 54.0;   // Setpoint para ir atrás (lejos del sensor)
float setpoint = 6.0;            // Setpoint activo actual

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

// Debounce
unsigned long lastDebouncePID = 0;
unsigned long lastDebounceForward = 0;
unsigned long lastDebounceBackward = 0;
const unsigned long DEBOUNCE_MS = 50;

// Estados
bool pid_enabled   = false;
bool motor_running = false;
int motor_direction = 0;  // 0=parado, 1=adelante, -1=atrás

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

  Serial.println("=== PID Bidireccional — Sistema listo ===");
  Serial.print("Posición inicial: ");
  Serial.print(posicion_actual);
  Serial.println(" cm");
  Serial.print("Setpoint ADELANTE: ");
  Serial.print(setpoint_forward);
  Serial.println(" cm");
  Serial.print("Setpoint ATRÁS: ");
  Serial.print(setpoint_backward);
  Serial.println(" cm");
}

void loop() {

  //---------------------------------------------------------------------------
  // Botón adelante
  //---------------------------------------------------------------------------
  static bool last_forward = HIGH;
  bool forward_btn = digitalRead(BUTTON_FORWARD);

  if (forward_btn == LOW && last_forward == HIGH && 
      millis() - lastDebounceForward > DEBOUNCE_MS) {
    
    lastDebounceForward = millis();

    // Si está detenido O yendo en dirección contraria, cambia a adelante
    if (!motor_running || motor_direction == -1) {
      motor_running = true;
      motor_direction = 1;
      setpoint = setpoint_forward;
      pid_enabled = false;  // Empieza sin PID
      integral = 0;
      last_error = 0;
      
      Serial.println("\n>>> MOTOR ADELANTE (SIN PID) <<<");
      Serial.print("Objetivo: ");
      Serial.print(setpoint);
      Serial.println(" cm");
      setMotorForward(255);
    } 
    // Si ya está yendo adelante, lo detiene
    else if (motor_direction == 1) {
      motor_running = false;
      motor_direction = 0;
      pid_enabled = false;
      Serial.println("\n>>> MOTOR DETENIDO <<<");
      stopMotor();
    }
  }
  last_forward = forward_btn;


  //---------------------------------------------------------------------------
  // Botón atrás
  //---------------------------------------------------------------------------
  static bool last_backward = HIGH;
  bool backward_btn = digitalRead(BUTTON_BACKWARD);

  if (backward_btn == LOW && last_backward == HIGH && 
      millis() - lastDebounceBackward > DEBOUNCE_MS) {
    
    lastDebounceBackward = millis();

    // Si está detenido O yendo en dirección contraria, cambia a atrás
    if (!motor_running || motor_direction == 1) {
      motor_running = true;
      motor_direction = -1;
      setpoint = setpoint_backward;
      pid_enabled = false;  // Empieza sin PID
      integral = 0;
      last_error = 0;
      
      Serial.println("\n>>> MOTOR ATRÁS (SIN PID) <<<");
      Serial.print("Objetivo: ");
      Serial.print(setpoint);
      Serial.println(" cm");
      setMotorBackward(255);
    }
    // Si ya está yendo atrás, lo detiene
    else if (motor_direction == -1) {
      motor_running = false;
      motor_direction = 0;
      pid_enabled = false;
      Serial.println("\n>>> MOTOR DETENIDO <<<");
      stopMotor();
    }
  }
  last_backward = backward_btn;


  //---------------------------------------------------------------------------
  // Botón PID
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
  // Lectura del sensor y control
  //---------------------------------------------------------------------------
  posicion_actual = readDistanceCM();

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_TIME) {
    
    lastSampleTime = now;

    // Cálculo de error según dirección
    if (motor_direction == -1) {
      // Yendo atrás: error = setpoint - posicion_actual
      error = setpoint - posicion_actual;
    } else if (motor_direction == 1) {
      // Yendo adelante: error = posicion_actual - setpoint
      error = posicion_actual - setpoint;
    } else {
      error = 0;
    }

    // -----------------------------------------------
    // Paro automático seguro al llegar al setpoint
    // -----------------------------------------------
    const float TOLERANCIA = 2.0;  // cm

    if (motor_running) {
        if (abs(error) <= TOLERANCIA) {
            detenerAutomatico();
            return;
        }
    }

    if (!motor_running) {
      stopMotor();
      printStatus();
      return;
    }


    //---------------------------------------------------------------------------
    // Modo PID 
    //---------------------------------------------------------------------------
    if (pid_enabled) {

      float dt = SAMPLE_TIME / 1000.0;

      // Se toma solo la magnitud del error
      float e = error;
      if (e < 0) e = -e;

      // --- PID ---
      float P = Kp * e;

      integral += e * dt;
      if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
      if (integral < 0)            integral = 0;

      float I = Ki * integral;

      derivative = (e - last_error) / dt;
      float D = Kd * derivative;

      output = P + I + D;

      if (output < 0)       output = 0;
      if (output > PWM_MAX) output = PWM_MAX;

      // Magnitud del error para la derivada
      last_error = e;

      // --- Aplicar PID según dirección ---
      if (motor_direction == -1) {
        // Atrás (alejar del sensor)
        setMotorBackward((int)output);
      } 
      else if (motor_direction == 1) {
        // Adelante (acercar al sensor)
        setMotorForward((int)output);
      }
    }


    //---------------------------------------------------------------------------
    // Modo sin PID — PWM máximo
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
// Funciones de control del motor
// ============================================================================

void setMotorForward(int speed) {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN, speed);
}

void detenerAutomatico() {
  motor_running = false;
  pid_enabled = false;
  motor_direction = 0;
  stopMotor();
  Serial.println("\n>>> PARO AUTOMÁTICO — SETPOINT ALCANZADO <<<");
  printStatus();
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
// Sensor ultrasonido
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
// Debug
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
