#include <Arduino.h>

// ----- Definiciones de pines (ajusta según tu conexión) -----
// Eliminamos la variable de enable ya que se controla externamente
// const int ENA      = 15;  // Pin de habilitación del driver (eliminado)
const int pwmA     = 5;   // Canal PWM para una dirección
const int pwmB     = 4;   // Canal PWM para la otra dirección
const int encoderA = 12;  // Pin del encoder, fase A
const int encoderB = 13;  // Pin del encoder, fase B

// ----- Variables del encoder -----
volatile long encoderCount = 0;
volatile byte lastStateA = 0;
volatile byte lastStateB = 0;
volatile byte stateA = 0;
volatile byte stateB = 0;

// ----- Constantes y variables de control PID -----
// Se asume que 4532 pulsos equivalen a 360°
const float COUNTS_PER_REVOLUTION = 4532.0;
float Kp = 0.2;
float Ki = 0.8;
float Kd = 1.5;
const float POSITION_TOLERANCE = 6.0; // Tolerancia en grados
const int MIN_PWM = 150;
const int MAX_PWM = 250;

// Variables de control
float integral    = 0;
float lastError   = 0;
float targetPosition = 0; // Ángulo deseado (0 a 180)
float currentPosition = 0;
float currentError    = 0;
float currentPWM      = 0;
unsigned long lastTime = 0;

// Variables para depuración (debug) vía Serial
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 1000; // Cada 1 segundo

// ----- (Opcional) Configuración de PWM con LEDC en ESP32‑S2 -----
// const int pwmFreq = 5000; // Frecuencia en Hz
// const int pwmResolution = 8; // Resolución en bits
// void setupPWM() {
//   ledcSetup(0, pwmFreq, pwmResolution);
//   ledcAttachPin(pwmA, 0);
//   
//   ledcSetup(1, pwmFreq, pwmResolution);
//   ledcAttachPin(pwmB, 1);
// }

// ----- Rutina de interrupción para el encoder -----
// Para ESP32‑S2 usamos IRAM_ATTR
IRAM_ATTR void handleEncoder() {
  noInterrupts();  // Lectura atómica
  
  stateA = digitalRead(encoderA);
  stateB = digitalRead(encoderB);
  
  if (stateA != lastStateA) {
    if (stateA != stateB) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }
  if (stateB != lastStateB) {
    if (stateB == stateA) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }
  
  lastStateA = stateA;
  lastStateB = stateB;
  
  interrupts();
}

// ----- Función para obtener la posición actual en grados -----
float getPos() {
  return (encoderCount * 360.0) / COUNTS_PER_REVOLUTION;
}

// ----- Función para detener el motor -----
// Al no controlar el enable, basta con poner a 0 el PWM
void stopMotor() {
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
  currentPWM = 0;
  integral = 0;
  Serial.println("Motor detenido.");
}

// ----- Función para aplicar PWM al motor -----
void setMotorPWM(int pwm) {
  currentPWM = constrain(pwm, -MAX_PWM, MAX_PWM);
  
  if (currentPWM > 0) {
    analogWrite(pwmA, 0);
    analogWrite(pwmB, abs(currentPWM));
  } else {
    analogWrite(pwmA, abs(currentPWM));
    analogWrite(pwmB, 0);
  }
}

// ----- Función de control PID para mover el motor al ángulo deseado -----
bool moveTo(float setpoint) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt < 0.001) dt = 0.001;  // Evitar dt muy pequeño
  lastTime = now;
  
  currentPosition = getPos();
  currentError = setpoint - currentPosition;
  
  if (abs(currentError) <= POSITION_TOLERANCE) {
    stopMotor();
    return true;
  }
  
  integral += currentError * dt;
  float derivative = (currentError - lastError) / dt;
  lastError = currentError;
  
  float pidOutput = (Kp * currentError) + (Ki * integral) + (Kd * derivative);
  int pwm = constrain(int(-pidOutput), -MAX_PWM, MAX_PWM); // Invierto la salida para corregir polaridad
  
  if (abs(pwm) < MIN_PWM && pwm != 0) {
    pwm = (pwm > 0) ? MIN_PWM : -MIN_PWM;
  }
  
  setMotorPWM(pwm);
  return false;
}

// ----- Configuración (setup) -----
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n--- Iniciando ESP32-S2 Motor Control (sin lógica de Enable) ---");
  
  // (Opcional) Configurar PWM con LEDC
  // setupPWM();
  
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  
  // Se asume que la alimentación del enable se gestiona externamente (conectado a 5V)
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
  
  lastStateA = digitalRead(encoderA);
  lastStateB = digitalRead(encoderB);
  attachInterrupt(digitalPinToInterrupt(encoderA), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), handleEncoder, CHANGE);
  Serial.println("Encoder inicializado.");
  
  lastTime = millis();
}

// ----- Loop principal -----
void loop() {
  // Procesar comandos desde el Monitor Serie
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();
    
    if (inputString.equalsIgnoreCase("s")) {
      stopMotor();
      targetPosition = currentPosition;
      Serial.println("Comando de parada recibido.");
    }
    else if (inputString.equalsIgnoreCase("r")) {
      noInterrupts();
      encoderCount = 0;
      interrupts();
      currentPosition = 0;
      targetPosition = 0;
      Serial.println("Encoder reiniciado a 0.");
    }
    else {
      float newTarget = inputString.toFloat();
      if (newTarget >= 0 && newTarget <= 180) {
        targetPosition = newTarget;
        Serial.print("Nuevo ángulo objetivo: ");
        Serial.println(targetPosition);
      } else {
        Serial.println("Entrada inválida. Use un valor entre 0 y 180, 's' para detener o 'r' para reiniciar.");
      }
    }
  }
  
  moveTo(targetPosition);
  
  if (millis() - lastDebugTime >= DEBUG_INTERVAL) {
    lastDebugTime = millis();
    Serial.println("\n--- Estado Actual ---");
    Serial.print("Posición: ");
    Serial.println(currentPosition);
    Serial.print("Objetivo: ");
    Serial.println(targetPosition);
    Serial.print("Error: ");
    Serial.println(currentError);
    Serial.print("PWM: ");
    Serial.println(currentPWM);
    Serial.print("Encoder Count: ");
    Serial.println(encoderCount);
    Serial.print("PID (Kp, Ki, Kd): ");
    Serial.print(Kp); Serial.print(", ");
    Serial.print(Ki); Serial.print(", ");
    Serial.println(Kd);
    Serial.println("-------------------");
  }
  
  // Pequeña pausa para permitir ejecución en background
  delay(1);
}
