// TEST SOLO MOTORES - ESP32-CAM
// Sin WiFi, sin cámara, sin servidor web
// Solo control de 3 motores con DRV8833

// MOTORES - Controladores DRV8833
// Controlador #1 - Motores 1 y 2
#define MOTOR1_IN1 12
#define MOTOR1_IN2 13
#define MOTOR2_IN1 14
#define MOTOR2_IN2 15

// Controlador #2 - Motor 3
#define MOTOR3_IN1 2
#define MOTOR3_IN2 4

// LED para indicar funcionamiento
#define LED_PIN 33

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("🎮 TEST SOLO MOTORES ESP32-CAM");
  Serial.println("==============================");
  Serial.println("Sin WiFi - Sin cámara - Solo motores");
  
  // Configurar LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Encender LED para indicar inicio
  
  // Configurar pines de motores como salidas
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);
  
  // Inicializar todos los motores parados
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  digitalWrite(MOTOR3_IN1, LOW);
  digitalWrite(MOTOR3_IN2, LOW);
  
  Serial.println("✅ Motores configurados");
  Serial.println("💡 LED encendido");
  Serial.println("⏳ Iniciando en 3 segundos...");
  
  delay(3000);
  Serial.println("🚀 COMENZANDO PRUEBA DE MOTORES");
}

void loop() {
  Serial.println("\n--- CICLO DE PRUEBA MOTORES ---");
  
  // MOTOR 1 - Adelante
  Serial.println("🎮 MOTOR 1 - ADELANTE (3 seg)");
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  delay(3000);
  
  // MOTOR 1 - Parar
  Serial.println("🛑 MOTOR 1 - PARAR");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  delay(1000);
  
  // MOTOR 1 - Atrás
  Serial.println("🎮 MOTOR 1 - ATRÁS (2 seg)");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  delay(2000);
  
  // MOTOR 1 - Parar
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  delay(1000);
  
  // MOTOR 2 - Adelante
  Serial.println("🎮 MOTOR 2 - ADELANTE (3 seg)");
  digitalWrite(MOTOR2_IN1, HIGH);
  digitalWrite(MOTOR2_IN2, LOW);
  delay(3000);
  
  // MOTOR 2 - Parar
  Serial.println("🛑 MOTOR 2 - PARAR");
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  delay(1000);
  
  // MOTOR 2 - Atrás
  Serial.println("🎮 MOTOR 2 - ATRÁS (2 seg)");
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, HIGH);
  delay(2000);
  
  // MOTOR 2 - Parar
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  delay(1000);
  
  // MOTOR 3 - Adelante
  Serial.println("🎮 MOTOR 3 - ADELANTE (3 seg)");
  digitalWrite(MOTOR3_IN1, HIGH);
  digitalWrite(MOTOR3_IN2, LOW);
  delay(3000);
  
  // MOTOR 3 - Parar
  Serial.println("🛑 MOTOR 3 - PARAR");
  digitalWrite(MOTOR3_IN1, LOW);
  digitalWrite(MOTOR3_IN2, LOW);
  delay(1000);
  
  // MOTOR 3 - Atrás
  Serial.println("🎮 MOTOR 3 - ATRÁS (2 seg)");
  digitalWrite(MOTOR3_IN1, LOW);
  digitalWrite(MOTOR3_IN2, HIGH);
  delay(2000);
  
  // MOTOR 3 - Parar
  digitalWrite(MOTOR3_IN1, LOW);
  digitalWrite(MOTOR3_IN2, LOW);
  delay(1000);
  
  // TODOS LOS MOTORES ADELANTE
  Serial.println("🎮 TODOS LOS MOTORES - ADELANTE (4 seg)");
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, HIGH);
  digitalWrite(MOTOR2_IN2, LOW);
  digitalWrite(MOTOR3_IN1, HIGH);
  digitalWrite(MOTOR3_IN2, LOW);
  delay(4000);
  
  // TODOS LOS MOTORES ATRÁS
  Serial.println("🎮 TODOS LOS MOTORES - ATRÁS (3 seg)");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, HIGH);
  digitalWrite(MOTOR3_IN1, LOW);
  digitalWrite(MOTOR3_IN2, HIGH);
  delay(3000);
  
  // PARAR TODOS
  Serial.println("🛑 PARAR TODOS LOS MOTORES");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  digitalWrite(MOTOR3_IN1, LOW);
  digitalWrite(MOTOR3_IN2, LOW);
  
  // Parpadear LED para indicar fin de ciclo
  Serial.println("💡 Fin de ciclo - LED parpadeando");
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(300);
    digitalWrite(LED_PIN, HIGH);
    delay(300);
  }
  
  Serial.println("📊 Estado del sistema:");
  Serial.printf("   RAM libre: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("   Uptime: %lu segundos\n", millis()/1000);
  
  Serial.println("⏳ Esperando 5 segundos para repetir...");
  delay(5000);
}