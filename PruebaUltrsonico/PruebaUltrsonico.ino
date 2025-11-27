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
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;

// Motor (ajusta según tu driver/transistor)
const int MOTOR_PIN = 9;  // Pin PWM

// ============= PARÁMETROS DEL EXPERIMENTO =============
const int PWM_ESCALON = 128;              
const unsigned long TIEMPO_INICIAL = 1000; 
const unsigned long TIEMPO_TOTAL = 14000;  
const unsigned long SAMPLE_TIME = 50;      

// **NUEVO: Offset de calibración**
const float OFFSET_CALIBRACION = 4.10;  // Distancia cuando puerta está en el borde

// ============= VARIABLES =============
unsigned long startTime = 0;
unsigned long previousMillis = 0;
bool escalon_aplicado = false;

volatile unsigned long pulseStartTime = 0;
volatile unsigned long pulseDuration = 0;
volatile bool pulseReady = false;
unsigned long lastTrigTime = 0;
const unsigned long TRIG_COOLDOWN = 100;

void setup() {
  Serial.begin(9600);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  
  analogWrite(MOTOR_PIN, 0);
  
  Serial.println("=== RESPUESTA AL ESCALON ===");
  Serial.println("PWM del escalon: " + String(PWM_ESCALON));
  Serial.println("Tiempo de muestreo: " + String(SAMPLE_TIME) + " ms");
  Serial.println("Offset de calibracion: " + String(OFFSET_CALIBRACION) + " cm");
  Serial.println("----------------------------");
  Serial.println("Tiempo_ms,Posicion_cm,PWM");
  
  startTime = millis();
  previousMillis = startTime;
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long tiempoTranscurrido = currentMillis - startTime;
  
  if (!escalon_aplicado && tiempoTranscurrido >= TIEMPO_INICIAL) {
    analogWrite(MOTOR_PIN, PWM_ESCALON);
    escalon_aplicado = true;
  }

  if (currentMillis - previousMillis >= SAMPLE_TIME) {
    previousMillis = currentMillis;

    // Leer posición del sensor y aplicar offset
    float posicionRaw = readUltrasonic();
    float posicionCalibrada = posicionRaw - OFFSET_CALIBRACION;
    
    // Evitar valores negativos
    if (posicionCalibrada < 0) {
      posicionCalibrada = 0;
    }

    // Imprimir datos en formato CSV
    Serial.print(tiempoTranscurrido);
    Serial.print(",");
    Serial.print(posicionCalibrada, 2);
    Serial.print(",");
    Serial.println(escalon_aplicado ? PWM_ESCALON : 0);
  }

  if (tiempoTranscurrido >= TIEMPO_TOTAL) {
    analogWrite(MOTOR_PIN, 0);
    Serial.println("=== FIN DEL EXPERIMENTO ===");
    Serial.println("Copie los datos y guárdelos en un archivo CSV");
    while(1);
  }
}

// ============= LEER SENSOR ULTRASÓNICO =============
float readUltrasonic() {
  unsigned long currentMicros = micros();
  
  if (currentMicros - lastTrigTime >= TRIG_COOLDOWN * 1000) {
    lastTrigTime = currentMicros;
    
    digitalWrite(TRIG_PIN, LOW);
    digitalWrite(TRIG_PIN, HIGH);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    
    if (duration > 0) {
      pulseDuration = duration;
    }
  }
  
  float distance = pulseDuration * 0.0343 / 2.0;
  
  if (pulseDuration == 0 || distance > 100) {
    return OFFSET_CALIBRACION; // Si hay error, retornar el offset (posición = 0)
  }
  
  return distance;
}