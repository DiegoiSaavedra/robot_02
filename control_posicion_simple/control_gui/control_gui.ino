#include <Arduino.h>

// ---------- Definiciones de pines ----------
// Motor 1
const int pwmA1     = 5;    // Canal PWM para una dirección
const int pwmB1     = 4;    // Canal PWM para la otra dirección
const int encoderA1 = 12;   // Pin del encoder, fase A
const int encoderB1 = 13;   // Pin del encoder, fase B

// Motor 2
const int pwmA2     = 2;   // Canal PWM para una dirección
const int pwmB2     = 3;   // Canal PWM para la otra dirección
const int encoderA2 = 8;   // Pin del encoder, fase A
const int encoderB2 = 9;   // Pin del encoder, fase B

// ---------- Variables del encoder ----------
// Motor 1
volatile long encoderCount1 = 0;
volatile byte lastStateA1 = 0;
volatile byte lastStateB1 = 0;
volatile byte stateA1 = 0;
volatile byte stateB1 = 0;

// Motor 2
volatile long encoderCount2 = 0;
volatile byte lastStateA2 = 0;
volatile byte lastStateB2 = 0;
volatile byte stateA2 = 0;
volatile byte stateB2 = 0;

// ---------- Constantes y variables de control PID ----------
const float COUNTS_PER_REVOLUTION = 4532.0;  // Pulsos por revolución (360°)
float Kp = 2;
float Ki = 2;
float Kd = 1.5;
const float POSITION_TOLERANCE = 1.0;        // Tolerancia en grados
int MIN_PWM = 98;                            // Ahora variable para poder cambiarlo
const int MAX_PWM = 250;

// Variables PID y control para Motor 1
float integral1      = 0;
float lastError1     = 0;
float targetPosition1 = 0;  // Ángulo deseado (0 a 180)
float currentPosition1 = 0;
float currentPWM1      = 0;
unsigned long lastTime1 = 0;

// Variables PID y control para Motor 2
float integral2      = 0;
float lastError2     = 0;
float targetPosition2 = 0;  // Ángulo deseado (0 a 180)
float currentPosition2 = 0;
float currentPWM2      = 0;
unsigned long lastTime2 = 0;

// Intervalo para debug (impresión por Serial)
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 1000; // cada 1 segundo

// ---------- Funciones de los encoders ----------
// ISR para Motor 1
IRAM_ATTR void handleEncoder1() {
  noInterrupts();
  stateA1 = digitalRead(encoderA1);
  stateB1 = digitalRead(encoderB1);
  if (stateA1 != lastStateA1) {
    if (stateA1 != stateB1) {
      encoderCount1++;
    } else {
      encoderCount1--;
    }
  }
  if (stateB1 != lastStateB1) {
    if (stateB1 == stateA1) {
      encoderCount1++;
    } else {
      encoderCount1--;
    }
  }
  lastStateA1 = stateA1;
  lastStateB1 = stateB1;
  interrupts();
}

// ISR para Motor 2
IRAM_ATTR void handleEncoder2() {
  noInterrupts();
  stateA2 = digitalRead(encoderA2);
  stateB2 = digitalRead(encoderB2);
  if (stateA2 != lastStateA2) {
    if (stateA2 != stateB2) {
      encoderCount2++;
    } else {
      encoderCount2--;
    }
  }
  if (stateB2 != lastStateB2) {
    if (stateB2 == stateA2) {
      encoderCount2++;
    } else {
      encoderCount2--;
    }
  }
  lastStateA2 = stateA2;
  lastStateB2 = stateB2;
  interrupts();
}

// ---------- Funciones auxiliares ----------
float getPos1() {
  return (encoderCount1 * 360.0) / COUNTS_PER_REVOLUTION;
}

float getPos2() {
  return (encoderCount2 * 360.0) / COUNTS_PER_REVOLUTION;
}

void stopMotor1() {
  analogWrite(pwmA1, 0);
  analogWrite(pwmB1, 0);
  currentPWM1 = 0;
  integral1 = 0;
  Serial.println("Motor 1 detenido.");
}

void stopMotor2() {
  analogWrite(pwmA2, 0);
  analogWrite(pwmB2, 0);
  currentPWM2 = 0;
  integral2 = 0;
  Serial.println("Motor 2 detenido.");
}

void setMotorPWM1(int pwm) {
  currentPWM1 = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (currentPWM1 > 0) {
    analogWrite(pwmA1, 0);
    analogWrite(pwmB1, abs(currentPWM1));
  } else {
    analogWrite(pwmA1, abs(currentPWM1));
    analogWrite(pwmB1, 0);
  }
}

void setMotorPWM2(int pwm) {
  currentPWM2 = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (currentPWM2 > 0) {
    analogWrite(pwmA2, 0);
    analogWrite(pwmB2, abs(currentPWM2));
  } else {
    analogWrite(pwmA2, abs(currentPWM2));
    analogWrite(pwmB2, 0);
  }
}

// ---------- Funciones de control PID ----------
bool moveToMotor1(float setpoint) {
  unsigned long now = millis();
  float dt = (now - lastTime1) / 1000.0;
  if (dt < 0.001) dt = 0.001;
  lastTime1 = now;
  
  currentPosition1 = getPos1();
  float error = setpoint - currentPosition1;
  
  if (abs(error) <= POSITION_TOLERANCE) {
    stopMotor1();
    return true;
  }
  
  integral1 += error * dt;
  float derivative = (error - lastError1) / dt;
  lastError1 = error;
  
  float pidOutput = (Kp * error) + (Ki * integral1) + (Kd * derivative);
  int pwm = constrain(int(-pidOutput), -MAX_PWM, MAX_PWM);
  
  if (abs(pwm) < MIN_PWM && pwm != 0) {
    pwm = (pwm > 0) ? MIN_PWM : -MIN_PWM;
  }
  
  setMotorPWM1(pwm);
  return false;
}

bool moveToMotor2(float setpoint) {
  unsigned long now = millis();
  float dt = (now - lastTime2) / 1000.0;
  if (dt < 0.001) dt = 0.001;
  lastTime2 = now;
  
  currentPosition2 = getPos2();
  float error = setpoint - currentPosition2;
  
  if (abs(error) <= POSITION_TOLERANCE) {
    stopMotor2();
    return true;
  }
  
  integral2 += error * dt;
  float derivative = (error - lastError2) / dt;
  lastError2 = error;
  
  float pidOutput = (Kp * error) + (Ki * integral2) + (Kd * derivative);
  int pwm = constrain(int(-pidOutput), -MAX_PWM, MAX_PWM);
  
  if (abs(pwm) < MIN_PWM && pwm != 0) {
    pwm = (pwm > 0) ? MIN_PWM : -MIN_PWM;
  }
  
  setMotorPWM2(pwm);
  return false;
}

// ---------- Configuración (setup) ----------
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n--- Iniciando Control de Motores DC ---");
  
  // Configurar pines para Motor 1
  pinMode(pwmA1, OUTPUT);
  pinMode(pwmB1, OUTPUT);
  pinMode(encoderA1, INPUT_PULLUP);
  pinMode(encoderB1, INPUT_PULLUP);
  
  // Configurar pines para Motor 2
  pinMode(pwmA2, OUTPUT);
  pinMode(pwmB2, OUTPUT);
  pinMode(encoderA2, INPUT_PULLUP);
  pinMode(encoderB2, INPUT_PULLUP);
  
  // Inicializar salidas PWM a 0
  analogWrite(pwmA1, 0);
  analogWrite(pwmB1, 0);
  analogWrite(pwmA2, 0);
  analogWrite(pwmB2, 0);
  
  // Inicializar estados de los encoders
  lastStateA1 = digitalRead(encoderA1);
  lastStateB1 = digitalRead(encoderB1);
  lastStateA2 = digitalRead(encoderA2);
  lastStateB2 = digitalRead(encoderB2);
  
  // Adjuntar interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB1), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA2), handleEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB2), handleEncoder2, CHANGE);
  
  Serial.println("Encoders inicializados.");
  
  lastTime1 = millis();
  lastTime2 = millis();
}

// ---------- Loop principal ----------
void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();
    
    // Comandos especiales:
    if (inputString.equalsIgnoreCase("s")) {
      stopMotor1();
      stopMotor2();
      targetPosition1 = currentPosition1;
      targetPosition2 = currentPosition2;
      Serial.println("Comando de parada recibido para ambos motores.");
    }
    else if (inputString.equalsIgnoreCase("r")) {
      noInterrupts();
      encoderCount1 = 0;
      encoderCount2 = 0;
      interrupts();
      currentPosition1 = 0;
      currentPosition2 = 0;
      targetPosition1 = 0;
      targetPosition2 = 0;
      Serial.println("Encoders reiniciados a 0 para ambos motores.");
    }
    else if (inputString.startsWith("pid:")) {
      // Formato esperado: pid:Kp,Ki,Kd
      String pidValues = inputString.substring(4);
      pidValues.trim();
      int index1 = pidValues.indexOf(',');
      int index2 = pidValues.indexOf(',', index1 + 1);
      if (index1 > 0 && index2 > index1) {
        String kpStr = pidValues.substring(0, index1);
        String kiStr = pidValues.substring(index1 + 1, index2);
        String kdStr = pidValues.substring(index2 + 1);
        float newKp = kpStr.toFloat();
        float newKi = kiStr.toFloat();
        float newKd = kdStr.toFloat();
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
        Serial.print("Nuevos valores PID: Kp=");
        Serial.print(Kp);
        Serial.print(" Ki=");
        Serial.print(Ki);
        Serial.print(" Kd=");
        Serial.println(Kd);
      } else {
        Serial.println("Formato de PID inválido. Use: pid:Kp,Ki,Kd");
      }
    }
    else if (inputString.startsWith("min:")) {
      // Formato esperado: min:valor
      String minStr = inputString.substring(4);
      minStr.trim();
      int newMin = minStr.toInt();
      if (newMin >= 0 && newMin < MAX_PWM) {
        MIN_PWM = newMin;
        Serial.print("Nuevo valor MIN_PWM: ");
        Serial.println(MIN_PWM);
      } else {
        Serial.println("Valor MIN_PWM inválido. Debe ser menor que MAX_PWM y positivo.");
      }
    }
    else {
      // Se esperan comandos en el formato "1:ángulo 2:ángulo"
      int index1 = inputString.indexOf("1:");
      if (index1 >= 0) {
        int end1 = inputString.indexOf(' ', index1);
        String motor1Cmd = (end1 == -1) ? inputString.substring(index1) : inputString.substring(index1, end1);
        String angle1Str = motor1Cmd.substring(2);
        float newTarget1 = angle1Str.toFloat();
        if (newTarget1 >= 0 && newTarget1 <= 180) {
          targetPosition1 = newTarget1;
          Serial.print("Nuevo ángulo objetivo Motor 1: ");
          Serial.println(targetPosition1);
        } else {
          Serial.println("Entrada inválida para Motor 1. Use un valor entre 0 y 180.");
        }
      }
      
      int index2 = inputString.indexOf("2:");
      if (index2 >= 0) {
        int end2 = inputString.indexOf(' ', index2);
        String motor2Cmd = (end2 == -1) ? inputString.substring(index2) : inputString.substring(index2, end2);
        String angle2Str = motor2Cmd.substring(2);
        float newTarget2 = angle2Str.toFloat();
        if (newTarget2 >= 0 && newTarget2 <= 180) {
          targetPosition2 = newTarget2;
          Serial.print("Nuevo ángulo objetivo Motor 2: ");
          Serial.println(targetPosition2);
        } else {
          Serial.println("Entrada inválida para Motor 2. Use un valor entre 0 y 180.");
        }
      }
    }
  }
  
  // Actualizar el control PID de cada motor
  moveToMotor1(targetPosition1);
  moveToMotor2(targetPosition2);
  
  // Depuración: imprimir estado cada DEBUG_INTERVAL
  if (millis() - lastDebugTime >= DEBUG_INTERVAL) {
    lastDebugTime = millis();
    Serial.println("\n--- Estado Actual ---");
    Serial.print("Motor 1 - Posición: ");
    Serial.print(currentPosition1);
    Serial.print("° | Objetivo: ");
    Serial.print(targetPosition1);
    Serial.print("° | PWM: ");
    Serial.println(currentPWM1);
    Serial.print("Encoder Count: ");
    Serial.println(encoderCount1);
    
    Serial.print("Motor 2 - Posición: ");
    Serial.print(currentPosition2);
    Serial.print("° | Objetivo: ");
    Serial.print(targetPosition2);
    Serial.print("° | PWM: ");
    Serial.println(currentPWM2);
    Serial.print("Encoder Count: ");
    Serial.println(encoderCount2);
    Serial.println("-------------------");
  }
  
  delay(1);
}
