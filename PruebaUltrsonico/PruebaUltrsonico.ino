/*
 * PREGUNTA 7: Caracterización del sistema mediante respuesta al escalón
 * 
 * Hardware:
 * - Arduino/ESP32
 * - Motor DC + transistor/driver
 * - Sensor ultrasónico HC-SR04
 * 
 * Funcionamiento:
 * 1. Motor apagado durante 2 segundos (estado inicial)
 * 2. En t=2s, aplica escalón de PWM (50% = 128)
 * 3. Registra posición cada 50ms durante 10 segundos
 * 4. Imprime datos en formato CSV para MATLAB
 */

// ============= PINES =============
// Sensor Ultrasónico
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

// Motor (ajusta según tu driver/transistor)
const int MOTOR_PIN = 9;  // Pin PWM

// ============= PARÁMETROS DEL EXPERIMENTO =============
const int PWM_ESCALON = 128;              // 50% de PWM (0-255)
const unsigned long TIEMPO_INICIAL = 1000; // 2s antes del escalón
const unsigned long TIEMPO_TOTAL = 7000;  // 12s total (2s + 10s de medición)
const unsigned long SAMPLE_TIME = 50;      // Muestreo cada 50ms

// ============= VARIABLES =============
unsigned long startTime = 0;
unsigned long previousMillis = 0;
bool escalon_aplicado = false;

void setup() {
  Serial.begin(9600);
  
  // Configurar pines
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  
  // Motor apagado inicialmente
  analogWrite(MOTOR_PIN, 0);
  
  // Esperar un momento antes de empezar
  delay(1000);
  
  // Imprimir encabezado CSV
  Serial.println("=== RESPUESTA AL ESCALON ===");
  Serial.println("PWM del escalon: " + String(PWM_ESCALON));
  Serial.println("Tiempo de muestreo: " + String(SAMPLE_TIME) + " ms");
  Serial.println("----------------------------");
  Serial.println("Tiempo_ms,Posicion_cm,PWM");
  
  // Iniciar cronómetro
  startTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long tiempoTranscurrido = currentMillis - startTime;
  
  // Verificar si llegamos al tiempo de aplicar el escalón
  if (!escalon_aplicado && tiempoTranscurrido >= TIEMPO_INICIAL) {
    analogWrite(MOTOR_PIN, PWM_ESCALON);
    escalon_aplicado = true;
  }
  
  // Tomar muestra cada SAMPLE_TIME
  if (currentMillis - previousMillis >= SAMPLE_TIME) {
    previousMillis = currentMillis;
    
    // Leer posición del sensor
    float posicion = readUltrasonic();
    
    // Imprimir datos en formato CSV
    Serial.print(tiempoTranscurrido);
    Serial.print(",");
    Serial.print(posicion, 2);
    Serial.print(",");
    Serial.println(escalon_aplicado ? PWM_ESCALON : 0);
  }
  
  // Detener después del tiempo total
  if (tiempoTranscurrido >= TIEMPO_TOTAL) {
    analogWrite(MOTOR_PIN, 0); // Apagar motor
    Serial.println("=== FIN DEL EXPERIMENTO ===");
    Serial.println("Copie los datos y guárdelos en un archivo CSV");
    while(1); // Detener programa
  }
}

// ============= LEER SENSOR ULTRASÓNICO =============
float readUltrasonic() {
  // Limpiar trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Enviar pulso
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Leer echo
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Calcular distancia en cm
  float distance = duration * 0.0343 / 2.0;
  
  // Validar lectura
  if (duration == 0 || distance > 100) {
    return 0; // Error en lectura
  }
  
  return distance;
}