#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// Definir modelo de cámara
#define CAMERA_MODEL_AI_THINKER

// Pines para AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// WiFi
const char* ssid = "Ruby";
const char* password = "3108564848";

// Servidor web
WebServer server(80);

// ===== CONTROL DE MOTORES OMNIDIRECCIONAL =====
// Controlador DRV8833 #1 - Motores 1 y 2
#define MOTOR1_IN1 12
#define MOTOR1_IN2 13
#define MOTOR2_IN1 14
#define MOTOR2_IN2 15

// Controlador DRV8833 #2 - Motor 3
#define MOTOR3_IN1 2
#define MOTOR3_IN2 4

// PWM para control de velocidad
const int freq = 1000;
const int pwm_resolution = 8;
const int pwmChannel1A = 0;
const int pwmChannel1B = 1;
const int pwmChannel2A = 2;
const int pwmChannel2B = 3;
const int pwmChannel3A = 4;
const int pwmChannel3B = 5;

// Variables de estado
bool camera_ok = false;
bool robot_active = false;
int motor1_speed = 0;  // -255 a 255
int motor2_speed = 0;
int motor3_speed = 0;

// Contadores para estadísticas
unsigned long total_commands = 0;
unsigned long total_captures = 0;
unsigned long last_command_time = 0;

void setup() {
  // Serial
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n🤖 ROBOT OMNIDIRECCIONAL ESP32-CAM");
  Serial.println("=====================================");
  
  // Configurar motores PRIMERO
  setupMotors();
  Serial.println("✅ Motores configurados");
  
  // Configurar cámara
  setupCamera();
  Serial.println("✅ Cámara configurada");
  
  // Conectar WiFi
  setupWiFi();
  Serial.println("✅ WiFi conectado");
  
  // Iniciar servidor web
  setupWebServer();
  Serial.println("✅ Servidor web iniciado");
  
  // Información final
  Serial.println("=====================================");
  Serial.print("🌐 IP del robot: ");
  Serial.println(WiFi.localIP());
  Serial.println("🎯 Endpoints disponibles:");
  Serial.println("   /: Página principal");
  Serial.println("   /capture: Captura para IA Python");
  Serial.println("   /control: Control omnidireccional");
  Serial.println("   /stop: Parada de emergencia");
  Serial.println("   /status: Estado del robot");
  Serial.println("🚀 Robot listo para IA!");
  Serial.println("=====================================");
}

void setupMotors() {
  // Configurar pines de motores
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);
  
  // Configurar PWM
  ledcSetup(pwmChannel1A, freq, pwm_resolution);
  ledcSetup(pwmChannel1B, freq, pwm_resolution);
  ledcSetup(pwmChannel2A, freq, pwm_resolution);
  ledcSetup(pwmChannel2B, freq, pwm_resolution);
  ledcSetup(pwmChannel3A, freq, pwm_resolution);
  ledcSetup(pwmChannel3B, freq, pwm_resolution);
  
  // Asociar pines a PWM
  ledcAttachPin(MOTOR1_IN1, pwmChannel1A);
  ledcAttachPin(MOTOR1_IN2, pwmChannel1B);
  ledcAttachPin(MOTOR2_IN1, pwmChannel2A);
  ledcAttachPin(MOTOR2_IN2, pwmChannel2B);
  ledcAttachPin(MOTOR3_IN1, pwmChannel3A);
  ledcAttachPin(MOTOR3_IN2, pwmChannel3B);
  
  // Parar todos los motores
  stopAllMotors();
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;  // 320x240 - Perfecto para IA
  config.jpeg_quality = 12;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Error cámara: 0x%x\n", err);
    camera_ok = false;
  } else {
    camera_ok = true;
  }
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n❌ WiFi falló");
    ESP.restart();
  }
}

void setupWebServer() {
  // Página principal
  server.on("/", []() {
    String html = "<!DOCTYPE html><html><head><title>Robot IA Omnidireccional</title>";
    html += "<meta charset='utf-8'><meta name='viewport' content='width=device-width'>";
    html += "<style>body{font-family:Arial;text-align:center;background:#f0f0f0}";
    html += ".container{max-width:600px;margin:20px auto;background:white;padding:20px;border-radius:10px}";
    html += ".status{background:#e8f5e8;padding:10px;margin:10px 0;border-radius:5px}";
    html += ".motor{background:#f0f8ff;padding:8px;margin:5px 0;border-radius:3px}";
    html += "button{padding:10px 20px;margin:5px;border:none;border-radius:5px;cursor:pointer}";
    html += ".stop{background:#ff4444;color:white}.test{background:#44ff44;color:black}</style></head>";
    html += "<body><div class='container'>";
    html += "<h1>🤖 Robot IA Omnidireccional</h1>";
    html += "<div class='status'><h3>📊 Estado del Sistema</h3>";
    html += "<p>🌐 IP: " + WiFi.localIP().toString() + "</p>";
    html += "<p>📡 WiFi: " + String(ssid) + "</p>";
    html += "<p>📷 Cámara: " + String(camera_ok ? "OK" : "Error") + "</p>";
    html += "<p>🤖 Robot: " + String(robot_active ? "Activo" : "Inactivo") + "</p>";
    html += "<p>💾 RAM: " + String(ESP.getFreeHeap()) + " bytes</p></div>";
    html += "<div class='status'><h3>⚙️ Estado Motores</h3>";
    html += "<div class='motor'>Motor 1: " + String(motor1_speed) + "</div>";
    html += "<div class='motor'>Motor 2: " + String(motor2_speed) + "</div>";
    html += "<div class='motor'>Motor 3: " + String(motor3_speed) + "</div></div>";
    html += "<div class='status'><h3>📈 Estadísticas</h3>";
    html += "<p>📸 Capturas: " + String(total_captures) + "</p>";
    html += "<p>🎮 Comandos: " + String(total_commands) + "</p></div>";
    html += "<button class='stop' onclick=\"fetch('/stop')\">🛑 PARAR</button>";
    html += "<button class='test' onclick=\"fetch('/test_motors')\">🧪 TEST MOTORES</button>";
    html += "</div></body></html>";
    server.send(200, "text/html", html);
  });
  
  // Captura para IA (FUNCIONA PERFECTAMENTE)
  server.on("/capture", []() {
    Serial.println("📸 IA solicita captura");
    total_captures++;
    
    if (!camera_ok) {
      server.send(500, "text/plain", "Camera error");
      return;
    }
    
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb || fb->len == 0) {
      server.send(500, "text/plain", "Capture failed");
      if (fb) esp_camera_fb_return(fb);
      return;
    }
    
    server.sendHeader("Content-Type", "image/jpeg");
    server.sendHeader("Content-Length", String(fb->len));
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
    
    esp_camera_fb_return(fb);
    Serial.printf("✅ Imagen enviada a IA: %d bytes\n", fb->len);
  });
  
  // Control omnidireccional desde IA Python
  server.on("/control", HTTP_POST, []() {
    Serial.println("🎮 IA envía comando");
    total_commands++;
    last_command_time = millis();
    
    if (server.hasArg("plain")) {
      String json = server.arg("plain");
      Serial.println("📦 JSON: " + json);
      
      // Parse JSON simple
      float x = 0, y = 0, rotation = 0;
      bool valid = false;
      
      int xPos = json.indexOf("\"x\":");
      int yPos = json.indexOf("\"y\":");
      int rotPos = json.indexOf("\"rotation\":");
      
      if (xPos >= 0 && yPos >= 0 && rotPos >= 0) {
        x = extractFloat(json, xPos + 4);
        y = extractFloat(json, yPos + 4);
        rotation = extractFloat(json, rotPos + 11);
        valid = true;
      }
      
      if (valid) {
        Serial.printf("🎯 Comando: X=%.2f Y=%.2f R=%.2f\n", x, y, rotation);
        moveOmnidirectional(x, y, rotation);
        robot_active = (x != 0 || y != 0 || rotation != 0);
        
        server.send(200, "application/json", 
                   "{\"status\":\"OK\",\"x\":" + String(x) + 
                   ",\"y\":" + String(y) + ",\"rotation\":" + String(rotation) + "}");
      } else {
        server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      }
    } else {
      server.send(400, "application/json", "{\"error\":\"No data\"}");
    }
  });
  
  // Parada de emergencia
  server.on("/stop", []() {
    Serial.println("🛑 Parada de emergencia");
    stopAllMotors();
    robot_active = false;
    server.send(200, "application/json", "{\"status\":\"Stopped\"}");
  });
  
  // Test de motores
  server.on("/test_motors", []() {
    Serial.println("🧪 Test de motores iniciado");
    
    // Test secuencial de cada motor
    controlMotor(1, 100);  delay(1000);
    controlMotor(1, 0);    delay(500);
    controlMotor(2, 100);  delay(1000);
    controlMotor(2, 0);    delay(500);
    controlMotor(3, 100);  delay(1000);
    controlMotor(3, 0);
    
    Serial.println("✅ Test completado");
    server.send(200, "text/plain", "Test de motores completado");
  });
  
  // Status del robot
  server.on("/status", []() {
    String json = "{";
    json += "\"status\":\"" + String(robot_active ? "active" : "idle") + "\",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
    json += "\"camera\":" + String(camera_ok ? "true" : "false") + ",";
    json += "\"motor1\":" + String(motor1_speed) + ",";
    json += "\"motor2\":" + String(motor2_speed) + ",";
    json += "\"motor3\":" + String(motor3_speed) + ",";
    json += "\"captures\":" + String(total_captures) + ",";
    json += "\"commands\":" + String(total_commands);
    json += "}";
    server.send(200, "application/json", json);
  });
  
  server.begin();
}

float extractFloat(String json, int startPos) {
  int endPos = json.indexOf(",", startPos);
  if (endPos == -1) endPos = json.indexOf("}", startPos);
  if (endPos > startPos) {
    return json.substring(startPos, endPos).toFloat();
  }
  return 0.0;
}

void controlMotor(int motor, int speed) {
  speed = constrain(speed, -255, 255);
  
  switch (motor) {
    case 1:
      motor1_speed = speed;
      if (speed > 0) {
        ledcWrite(pwmChannel1A, speed);
        ledcWrite(pwmChannel1B, 0);
      } else if (speed < 0) {
        ledcWrite(pwmChannel1A, 0);
        ledcWrite(pwmChannel1B, -speed);
      } else {
        ledcWrite(pwmChannel1A, 0);
        ledcWrite(pwmChannel1B, 0);
      }
      break;
      
    case 2:
      motor2_speed = speed;
      if (speed > 0) {
        ledcWrite(pwmChannel2A, speed);
        ledcWrite(pwmChannel2B, 0);
      } else if (speed < 0) {
        ledcWrite(pwmChannel2A, 0);
        ledcWrite(pwmChannel2B, -speed);
      } else {
        ledcWrite(pwmChannel2A, 0);
        ledcWrite(pwmChannel2B, 0);
      }
      break;
      
    case 3:
      motor3_speed = speed;
      if (speed > 0) {
        ledcWrite(pwmChannel3A, speed);
        ledcWrite(pwmChannel3B, 0);
      } else if (speed < 0) {
        ledcWrite(pwmChannel3A, 0);
        ledcWrite(pwmChannel3B, -speed);
      } else {
        ledcWrite(pwmChannel3A, 0);
        ledcWrite(pwmChannel3B, 0);
      }
      break;
  }
}

void moveOmnidirectional(float x, float y, float rotation) {
  // Cálculo omnidireccional para 3 motores a 120°
  // Motor 1: 0°, Motor 2: 120°, Motor 3: 240°
  
  float motor1 = y + rotation;
  float motor2 = -0.5 * y + 0.866 * x + rotation;
  float motor3 = -0.5 * y - 0.866 * x + rotation;
  
  // Escalar a rango de motores [-255, 255]
  int m1 = constrain((int)(motor1 * 255), -255, 255);
  int m2 = constrain((int)(motor2 * 255), -255, 255);
  int m3 = constrain((int)(motor3 * 255), -255, 255);
  
  // Aplicar a motores
  controlMotor(1, m1);
  controlMotor(2, m2);
  controlMotor(3, m3);
  
  Serial.printf("🎯 Omnidireccional: M1=%d M2=%d M3=%d\n", m1, m2, m3);
}

void stopAllMotors() {
  controlMotor(1, 0);
  controlMotor(2, 0);
  controlMotor(3, 0);
  robot_active = false;
}

void loop() {
  // Procesar servidor web
  server.handleClient();
  
  // Verificar conexión WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi perdido, reconectando...");
    WiFi.begin(ssid, password);
  }
  
  // Parada de seguridad si no hay comandos por 10 segundos
  if (robot_active && millis() - last_command_time > 10000) {
    Serial.println("⚠️ Sin comandos por 10s - parada de seguridad");
    stopAllMotors();
  }
  
  // Heartbeat cada 30 segundos
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 30000) {
    Serial.printf("💓 Alive | WiFi:%s | Heap:%d | Capturas:%lu | Comandos:%lu\n",
                  WiFi.status() == WL_CONNECTED ? "OK" : "FAIL",
                  ESP.getFreeHeap(), total_captures, total_commands);
    lastHeartbeat = millis();
  }
  
  delay(10);
}