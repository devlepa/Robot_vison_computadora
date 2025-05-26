// CÓDIGO CORRECTO SEGÚN TU CONFIGURACIÓN REAL
// Motor A: GPIO2, GPIO4 (DRV#1 - funciona)
// Motor B: GPIO12, GPIO13 (DRV#2 - no funciona)  
// Motor C: GPIO14, GPIO15 (DRV#2 - no funciona)

#define LED_PIN 33

// MOTOR A - DRV8833 #1 (EL QUE FUNCIONA)
#define MOTOR_A_IN1 2   // GPIO2 → DRV#1 IN3
#define MOTOR_A_IN2 4   // GPIO4 → DRV#1 IN4

// MOTOR B - DRV8833 #2 (EL QUE NO FUNCIONA)
#define MOTOR_B_IN1 12  // GPIO12 → DRV#2 IN1
#define MOTOR_B_IN2 13  // GPIO13 → DRV#2 IN2

// MOTOR C - DRV8833 #2 (EL QUE NO FUNCIONA)
#define MOTOR_C_IN1 14  // GPIO14 → DRV#2 IN3
#define MOTOR_C_IN2 15  // GPIO15 → DRV#2 IN4

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("🎮 TEST MOTORES CONFIGURACIÓN REAL");
  Serial.println("==================================");
  Serial.println("Motor A: GPIO2,4  (DRV#1 - debe funcionar)");
  Serial.println("Motor B: GPIO12,13 (DRV#2 - puede no funcionar)");
  Serial.println("Motor C: GPIO14,15 (DRV#2 - puede no funcionar)");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Configurar todos los pines
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_C_IN1, OUTPUT);
  pinMode(MOTOR_C_IN2, OUTPUT);
  
  // Apagar todos los motores inicialmente
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  digitalWrite(MOTOR_C_IN1, LOW);
  digitalWrite(MOTOR_C_IN2, LOW);
  
  Serial.println("✅ Motores configurados según tu conexión real");
  Serial.println("⏳ Iniciando en 3 segundos...");
  delay(3000);
  Serial.println("🚀 COMENZANDO PRUEBA\n");
}

void loop() {
  Serial.println("=== CICLO DE PRUEBA MOTORES REALES ===\n");
  
  // MOTOR A - DRV#1 (DEBE FUNCIONAR)
  Serial.println("🎮 MOTOR A (DRV#1) - ADELANTE (3 seg)");
  Serial.println("   Pines: GPIO2=HIGH, GPIO4=LOW");
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  delay(3000);
  
  Serial.println("🛑 MOTOR A - PARAR");
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  delay(1000);
  
  Serial.println("🎮 MOTOR A (DRV#1) - ATRÁS (2 seg)");
  Serial.println("   Pines: GPIO2=LOW, GPIO4=HIGH");
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  delay(2000);
  
  Serial.println("🛑 MOTOR A - PARAR");
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  delay(2000);
  
  // MOTOR B - DRV#2 (PUEDE NO FUNCIONAR)
  Serial.println("🎮 MOTOR B (DRV#2) - ADELANTE (3 seg)");
  Serial.println("   Pines: GPIO12=HIGH, GPIO13=LOW");
  Serial.println("   ⚠️ Si no se mueve = DRV#2 sin alimentación");
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  delay(3000);
  
  Serial.println("🛑 MOTOR B - PARAR");
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  delay(1000);
  
  Serial.println("🎮 MOTOR B (DRV#2) - ATRÁS (2 seg)");
  Serial.println("   Pines: GPIO12=LOW, GPIO13=HIGH");
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  delay(2000);
  
  Serial.println("🛑 MOTOR B - PARAR");
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  delay(2000);
  
  // MOTOR C - DRV#2 (PUEDE NO FUNCIONAR)
  Serial.println("🎮 MOTOR C (DRV#2) - ADELANTE (3 seg)");
  Serial.println("   Pines: GPIO14=HIGH, GPIO15=LOW");
  Serial.println("   ⚠️ Si no se mueve = DRV#2 sin alimentación");
  digitalWrite(MOTOR_C_IN1, HIGH);
  digitalWrite(MOTOR_C_IN2, LOW);
  delay(3000);
  
  Serial.println("🛑 MOTOR C - PARAR");
  digitalWrite(MOTOR_C_IN1, LOW);
  digitalWrite(MOTOR_C_IN2, LOW);
  delay(1000);
  
  Serial.println("🎮 MOTOR C (DRV#2) - ATRÁS (2 seg)");
  Serial.println("   Pines: GPIO14=LOW, GPIO15=HIGH");
  digitalWrite(MOTOR_C_IN1, LOW);
  digitalWrite(MOTOR_C_IN2, HIGH);
  delay(2000);
  
  Serial.println("🛑 MOTOR C - PARAR");
  digitalWrite(MOTOR_C_IN1, LOW);
  digitalWrite(MOTOR_C_IN2, LOW);
  delay(2000);
  
  // TODOS LOS MOTORES JUNTOS
  Serial.println("🎮 TODOS LOS MOTORES - ADELANTE (4 seg)");
  Serial.println("   Motor A: GPIO2=HIGH, GPIO4=LOW");
  Serial.println("   Motor B: GPIO12=HIGH, GPIO13=LOW");
  Serial.println("   Motor C: GPIO14=HIGH, GPIO15=LOW");
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  digitalWrite(MOTOR_C_IN1, HIGH);
  digitalWrite(MOTOR_C_IN2, LOW);
  delay(4000);
  
  Serial.println("🎮 TODOS LOS MOTORES - ATRÁS (3 seg)");
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  digitalWrite(MOTOR_C_IN1, LOW);
  digitalWrite(MOTOR_C_IN2, HIGH);
  delay(3000);
  
  Serial.println("🛑 PARAR TODOS LOS MOTORES");
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  digitalWrite(MOTOR_C_IN1, LOW);
  digitalWrite(MOTOR_C_IN2, LOW);
  
  // Diagnóstico
  Serial.println("\n📊 DIAGNÓSTICO:");
  Serial.println("✅ Motor A (DRV#1): ¿Se movió correctamente?");
  Serial.println("❓ Motor B (DRV#2): ¿Se movió o no?");
  Serial.println("❓ Motor C (DRV#2): ¿Se movió o no?");
  Serial.println("");
  Serial.println("Si solo Motor A funciona:");
  Serial.println("🔧 DRV#2 necesita verificación de alimentación");
  Serial.println("🔌 Verificar VCC y GND en DRV#2");
  Serial.println("⚡ Verificar GND común ESP32 ↔ DRV#2");
  
  Serial.printf("\n💾 RAM libre: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("⏱️ Uptime: %lu segundos\n", millis()/1000);
  
  // Parpadear LED para indicar fin de ciclo
  Serial.println("💡 Fin de ciclo - LED parpadeando");
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(300);
    digitalWrite(LED_PIN, HIGH);
    delay(300);
  }
  
  Serial.println("⏳ Esperando 5 segundos para repetir...\n");
  delay(5000);
}