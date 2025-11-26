//Controlador PID para puerta corrediza - Arduino

// ============= DEFINICIÓN DE PINES =============
const int MOTOR_PIN = 9;    // Pin PWM al transistor

// Botón de control
const int BUTTON_PIN = 2;   // Pin digital con pull-up interno

// ============= PARÁMETROS DEL PID =============
float Kp = 2.0;    // Constante Proporcional
float Ki = 0.5;    // Constante Integral
float Kd = 0.1;    // Constante Derivativa

// Variables del PID
float error = 0;              // Error recibido del comparador
float last_error = 0;         // Error anterior
float integral = 0;           // Suma acumulada de errores
float derivative = 0;         // Tasa de cambio del error
float output = 0;             // Salida del PID (0-255 para PWM)

// Límites del controlador
const float INTEGRAL_MAX = 100.0;  // Anti-windup para integral
const float OUTPUT_MIN = 0.0;      // PWM mínimo
const float OUTPUT_MAX = 255.0;    // PWM máximo

// ============= CONTROL DE TIEMPO =============
unsigned long previousMillis = 0;     // Tiempo anterior
const unsigned long SAMPLE_TIME = 50; // Periodo de muestreo: 50ms (20Hz)

// ============= CONTROL DEL SISTEMA =============
bool pid_enabled = false;         // Estado del controlador
bool last_button_state = HIGH;    // Estado anterior del botón
unsigned long last_debounce = 0;  // Tiempo para debounce
const unsigned long DEBOUNCE_TIME = 50; // 50ms de debounce

// ============= SELECTOR DE VELOCIDAD =============
// Tres velocidades diferentes (3 valores de error esperados)
// En la práctica, el comparador enviará estos valores
const float SPEED_ERRORS[3] = {-10.0, 0.0, 10.0}; // Errores de ejemplo
int speed_selector = 1; // Velocidad inicial (media)

// Simulación de entrada del comparador (para pruebas)
float simulated_error = 0.0;

void setup() {
  // Inicializar comunicación serial
  Serial.begin(9600);
  Serial.println("Controlador PID - Puerta Corrediza");
  Serial.println("===================================");
  
  // Configurar pin del motor como salida PWM
  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0); // Motor inicialmente apagado
  
  // Configurar botón con pull-up interno
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  Serial.println("Controlador inicializado");
  Serial.println("Presione el boton para activar/desactivar el PID");
  Serial.println("Envie valores de error por serial (ej: 'e15.5')");
  Serial.println("Comandos: '1', '2', '3' para cambiar perfil de velocidad");
}

void loop() {
  // Obtener tiempo actual
  unsigned long currentMillis = millis();
  
  // ============= MANEJO DEL BOTÓN (sin delay) =============
  handleButton();
  
  // ============= SELECTOR DE VELOCIDAD Y ENTRADA DE ERROR =============
  handleSerialInput();
  
  // ============= CONTROL PID A INTERVALO FIJO =============
  // Ejecutar el PID solo cuando haya transcurrido el periodo de muestreo
  if (currentMillis - previousMillis >= SAMPLE_TIME) {
    previousMillis = currentMillis; // Actualizar tiempo anterior
    
    // 1. OBTENER ERROR DEL COMPARADOR
    // En un sistema real, este valor vendría del bloque comparador
    // Para pruebas, se puede simular o recibir por serial
    error = getErrorFromComparator();
    
    // 2. ALGORITMO PID (solo si está habilitado)
    if (pid_enabled) {
      // Término Proporcional
      float P = Kp * error;
      
      // Término Integral con anti-windup
      integral += error;
      // Limitar integral para evitar windup
      if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
      if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;
      float I = Ki * integral;
      
      // Término Derivativo
      derivative = error - last_error;
      float D = Kd * derivative;
      
      // Salida total del PID
      output = P + I + D;
      
      // Limitar salida al rango de PWM
      output = constrain(output, OUTPUT_MIN, OUTPUT_MAX);
      
      // Actualizar error anterior
      last_error = error;
      
    } else {
      // Si el PID está deshabilitado, detener motor
      output = 0;
      integral = 0; // Reset integral cuando está apagado
      last_error = 0;
    }
    
    // 3. APLICAR SEÑAL DE CONTROL AL MOTOR
    analogWrite(MOTOR_PIN, (int)output);
    
    // 4. ENVIAR DATOS POR SERIAL (para graficar/monitorear)
    printStatus();
  }
  
  // Aquí el programa puede hacer otras tareas mientras espera
  // el siguiente ciclo de control (¡No hay delay!)
}

// ============= OBTENER ERROR DEL COMPARADOR =============
float getErrorFromComparator() {
  // En un sistema real, este valor vendría de:
  // 1. El comparador (hardware externo), o
  // 2. Calculado: error = setpoint - sensor_value
  // 
  // Para pruebas, usamos el error simulado que se puede
  // modificar por serial o usar un valor fijo
  
  return simulated_error;
}

// ============= MANEJO DEL BOTÓN SIN DELAY =============
void handleButton() {
  bool current_button_state = digitalRead(BUTTON_PIN);
  
  // Detectar flanco de bajada con debounce
  if (current_button_state == LOW && last_button_state == HIGH) {
    if (millis() - last_debounce > DEBOUNCE_TIME) {
      // Toggle del PID
      pid_enabled = !pid_enabled;
      
      if (pid_enabled) {
        Serial.println("\n>>> PID ACTIVADO <<<");
      } else {
        Serial.println("\n>>> PID DESACTIVADO <<<");
        analogWrite(MOTOR_PIN, 0); // Detener motor
      }
      
      last_debounce = millis();
    }
  }
  
  last_button_state = current_button_state;
}

// ============= SELECTOR DE VELOCIDAD Y ENTRADA DE ERROR =============
void handleSerialInput() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    
    // Cambiar perfil de velocidad
    if (input == '1') {
      speed_selector = 0;
      simulated_error = SPEED_ERRORS[0];
      Serial.print("Velocidad BAJA - Error simulado: ");
      Serial.println(simulated_error);
    } else if (input == '2') {
      speed_selector = 1;
      simulated_error = SPEED_ERRORS[1];
      Serial.print("Velocidad MEDIA - Error simulado: ");
      Serial.println(simulated_error);
    } else if (input == '3') {
      speed_selector = 2;
      simulated_error = SPEED_ERRORS[2];
      Serial.print("Velocidad ALTA - Error simulado: ");
      Serial.println(simulated_error);
    }
    // Recibir valor de error personalizado (formato: e<valor>)
    // Ejemplo: "e15.5" establece error = 15.5
    else if (input == 'e') {
      float custom_error = Serial.parseFloat();
      if (custom_error != 0 || Serial.peek() == '0') {
        simulated_error = custom_error;
        Serial.print("Error personalizado: ");
        Serial.println(simulated_error);
      }
    }
  }
}

// ============= IMPRIMIR ESTADO DEL CONTROLADOR =============
void printStatus() {
  Serial.print("PID:");
  Serial.print(pid_enabled ? "ON " : "OFF");
  Serial.print(" | Err:");
  Serial.print(error, 2);
  Serial.print(" | Out:");
  Serial.print((int)output);
  Serial.print(" | P:");
  Serial.print(Kp * error, 2);
  Serial.print(" I:");
  Serial.print(Ki * integral, 2);
  Serial.print(" D:");
  Serial.println(Kd * derivative, 2);
}