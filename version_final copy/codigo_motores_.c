/*
 * ROBOT ESP32-CAM - MOVIMIENTO HACIA ADELANTE
 * Configuración física REAL según imagen:
 * 
 * POSICIÓN FÍSICA DE LOS MOTORES:
 * Motor A: FRONTAL IZQUIERDO  (rueda delantera izquierda)
 * Motor B: TRASERO CENTRAL    (rueda trasera, detrás del ESP32-CAM) 
 * Motor C: FRONTAL DERECHO    (rueda delantera derecha)
 * 
 * PARA IR HACIA ADELANTE:
 * - Motor A (frontal izq) + Motor C (frontal der) = ADELANTE
 * - Motor B (trasero) se mantiene parado
 */

// ===== CONFIGURACIÓN DE PINES SEGÚN CÓDIGO QUE FUNCIONA =====
// Motor A - FRONTAL IZQUIERDO: IN3→GPIO2, IN4→GPIO4
#define MOTOR_A_IN3 2
#define MOTOR_A_IN4 4

// Motor B - TRASERO CENTRAL: IN1→GPIO12, IN2→GPIO13  
#define MOTOR_B_IN1 12
#define MOTOR_B_IN2 13

// Motor C - FRONTAL DERECHO: IN3→GPIO14, IN4→GPIO15
#define MOTOR_C_IN3 14
#define MOTOR_C_IN4 15

// Configuración de movimiento
const int forwardSpeed = 200;  // Velocidad para ir adelante (0-255)
const int testDuration = 4000; // 4 segundos de movimiento

void setup() {
  Serial.begin(115200);
  Serial.println("\n==============================================");
  Serial.println("    ROBOT ESP32-CAM - MOVIMIENTO ADELANTE");
  Serial.println("==============================================");
  Serial.println("CONFIGURACION FISICA REAL:");
  Serial.println("Motor A (GPIO 2,4)  = FRONTAL IZQUIERDO");
  Serial.println("Motor B (GPIO 12,13)= TRASERO CENTRAL");
  Serial.println("Motor C (GPIO 14,15)= FRONTAL DERECHO");
  Serial.println("");
  Serial.println("MOVIMIENTO HACIA ADELANTE:");
  Serial.println("- Motor A (frontal izq) = ACTIVO");
  Serial.println("- Motor B (trasero)     = PARADO");
  Serial.println("- Motor C (frontal der) = ACTIVO");
  Serial.println("==============================================\n");
  
  // Configurar todos los pines como salida
  pinMode(MOTOR_A_IN3, OUTPUT);
  pinMode(MOTOR_A_IN4, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_C_IN3, OUTPUT);
  pinMode(MOTOR_C_IN4, OUTPUT);
  
  // Asegurar que todos estén parados al inicio
  stopAllMotors();
  
  Serial.println("Todos los motores detenidos");
  Serial.println("Iniciando movimiento hacia ADELANTE en 3 segundos...\n");
  delay(3000);
}

void loop() {
  Serial.println("**************************************************");
  Serial.println("        MOVIMIENTO HACIA ADELANTE");
  Serial.println("**************************************************");
  Serial.println("Activando motores A (frontal izq) + C (frontal der)");
  Serial.println("Motor B (trasero) permanece PARADO");
  Serial.println("");
  
  // MOVER HACIA ADELANTE: Solo motores frontales A y C
  Serial.println("ROBOT -> ADELANTE (4 segundos)");
  moveForward();
  delay(testDuration);
  
  Serial.println("ROBOT -> DETENIDO");
  stopAllMotors();
  delay(2000);
  
  Serial.println("**************************************************");
  Serial.println("        MOVIMIENTO HACIA ATRAS");
  Serial.println("**************************************************");
  Serial.println("Invirtiendo motores A (frontal izq) + C (frontal der)");
  Serial.println("Motor B (trasero) permanece PARADO");
  Serial.println("");
  
  // MOVER HACIA ATRÁS: Invertir motores frontales A y C
  Serial.println("ROBOT -> ATRAS (4 segundos)");
  moveBackward();
  delay(testDuration);
  
  Serial.println("ROBOT -> DETENIDO");
  stopAllMotors();
  delay(3000);
  
  Serial.println("==============================================");
  Serial.println("OBSERVA EL MOVIMIENTO:");
  Serial.println("- ¿El robot va hacia ADELANTE correctamente?");
  Serial.println("- ¿El robot va hacia ATRAS correctamente?");
  Serial.println("- ¿Se mantiene recto o gira?");
  Serial.println("==============================================\n");
  
  delay(5000);  // Pausa antes de repetir
}

// ===== FUNCIONES DE MOVIMIENTO =====

// Función para mover hacia ADELANTE
void moveForward() {
  // Motor A (frontal izquierdo) - ADELANTE
  setMotor(MOTOR_A_IN3, MOTOR_A_IN4, forwardSpeed, true);
  
  // Motor B (trasero central) - PARADO
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
  
  // Motor C (frontal derecho) - ADELANTE  
  setMotor(MOTOR_C_IN3, MOTOR_C_IN4, forwardSpeed, true);
  
  Serial.printf("Motor A (frontal izq): ADELANTE velocidad %d\n", forwardSpeed);
  Serial.printf("Motor B (trasero):     PARADO\n");
  Serial.printf("Motor C (frontal der): ADELANTE velocidad %d\n", forwardSpeed);
}

// Función para mover hacia ATRÁS
void moveBackward() {
  // Motor A (frontal izquierdo) - ATRÁS
  setMotor(MOTOR_A_IN3, MOTOR_A_IN4, forwardSpeed, false);
  
  // Motor B (trasero central) - PARADO
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
  
  // Motor C (frontal derecho) - ATRÁS
  setMotor(MOTOR_C_IN3, MOTOR_C_IN4, forwardSpeed, false);
  
  Serial.printf("Motor A (frontal izq): ATRAS velocidad %d\n", forwardSpeed);
  Serial.printf("Motor B (trasero):     PARADO\n");
  Serial.printf("Motor C (frontal der): ATRAS velocidad %d\n", forwardSpeed);
}

// Función para controlar un motor individual
void setMotor(int in1, int in2, int speed, bool forward) {
    if (speed == 0) {
        // Motor parado
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    } else if (forward) {
        // Motor hacia adelante
        analogWrite(in1, speed);
        digitalWrite(in2, LOW);
    } else {
        // Motor hacia atrás
        digitalWrite(in1, LOW);
        analogWrite(in2, speed);
    }
}

// Función para detener todos los motores
void stopAllMotors() {
    // Detener Motor A (frontal izquierdo)
    setMotor(MOTOR_A_IN3, MOTOR_A_IN4, 0, true);
    
    // Detener Motor B (trasero central)
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    
    // Detener Motor C (frontal derecho)
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, 0, true);
    
    Serial.println(">>> TODOS LOS MOTORES DETENIDOS <<<");
}