#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

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

// WiFi - Tus credenciales
const char* ssid = "Ruby";
const char* password = "3108564848";

// Servidor web
WebServer server(80);

// LED indicador
#define LED_PIN 33

// CONFIGURACIÓN REAL DE MOTORES (según tu setup)
// Motor A - DRV8833 #1 (funciona)
#define MOTOR_A_IN1 2   // GPIO2 → DRV#1 IN3
#define MOTOR_A_IN2 4   // GPIO4 → DRV#1 IN4

// Motor B - DRV8833 #2 
#define MOTOR_B_IN1 12  // GPIO12 → DRV#2 IN1
#define MOTOR_B_IN2 13  // GPIO13 → DRV#2 IN2

// Motor C - DRV8833 #2
#define MOTOR_C_IN1 14  // GPIO14 → DRV#2 IN3
#define MOTOR_C_IN2 15  // GPIO15 → DRV#2 IN4

// Variables de estado
int motorA_speed = 0;
int motorB_speed = 0;
int motorC_speed = 0;
bool robot_active = false;
unsigned long last_command = 0;
unsigned long command_count = 0;

// Inicialización paso a paso para estabilidad
void setup() {
  Serial.begin(115200);
  delay(2000);  // Delay inicial crucial
  
  Serial.println("\n🤖 ROBOT VISION COMPUTADORA - SERVIDOR ESTABLE");
  Serial.println("===============================================");
  
  // PASO 1: Configurar LED primero
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.println("✅ LED configurado");
  delay(500);
  
  // PASO 2: Configurar motores
  setupMotors();
  Serial.println("✅ Motores configurados");
  delay(1000);
  
  // PASO 3: Configurar cámara CON manejo de errores
  if (!setupCamera()) {
    Serial.println("❌ Error cámara - continuando sin cámara");
  } else {
    Serial.println("✅ Cámara configurada");
  }
  delay(1000);
  
  // PASO 4: Conectar WiFi con timeout
  if (!setupWiFi()) {
    Serial.println("❌ WiFi falló - reiniciando en 5s");
    delay(5000);
    ESP.restart();
  }
  Serial.println("✅ WiFi conectado");
  delay(500);
  
  // PASO 5: Configurar servidor web
  setupWebServer();
  Serial.println("✅ Servidor web configurado");
  delay(500);
  
  // INFORMACIÓN FINAL
  Serial.println("===============================================");
  Serial.print("🌐 IP del Robot: ");
  Serial.println(WiFi.localIP());
  Serial.println("📡 Endpoints disponibles:");
  Serial.println("   /: Página principal");
  Serial.println("   /capture: Captura para Python");
  Serial.println("   /control: Control motores");
  Serial.println("   /stop: Parada emergencia");
  Serial.println("   /status: Estado del sistema");
  Serial.println("🚀 ROBOT LISTO PARA VISIÓN ARTIFICIAL!");
  Serial.println("===============================================");
  
  // LED parpadea para confirmar sistema listo
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
  }
}

void setupMotors() {
  // Configurar pines como salida
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_C_IN1, OUTPUT);
  pinMode(MOTOR_C_IN2, OUTPUT);
  
  // Asegurar que todos estén apagados
  stopAllMotors();
  delay(100);
}

bool setupCamera() {
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
  
  // Configuración optimizada para estabilidad
  config.frame_size = FRAMESIZE_QVGA;  // 320x240 - óptimo para líneas
  config.jpeg_quality = 12;            // Buena calidad
  config.fb_count = 1;                 // Un buffer para estabilidad
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Error cámara: 0x%x\n", err);
    return false;
  }
  
  // Configurar sensor para detección de líneas
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 0);     // Brillo normal
    s->set_contrast(s, 1);       // Contraste alto para línea negra
    s->set_saturation(s, -1);    // Menos saturación
    s->set_special_effect(s, 0); // Sin efectos
    s->set_whitebal(s, 1);       // Balance blanco automático
    s->set_awb_gain(s, 1);       // Ganancia balance blanco
    s->set_wb_mode(s, 0);        // Modo balance blanco auto
  }
  
  return true;
}

bool setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);  // Importante para estabilidad
  
  Serial.print("🌐 Conectando WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" ✅");
    return true;
  } else {
    Serial.println(" ❌");
    return false;
  }
}

void setupWebServer() {
  // Página principal optimizada
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><title>Robot Vision IA</title>";
    html += "<meta charset='utf-8'><meta name='viewport' content='width=device-width'>";
    html += "<style>body{font-family:Arial;text-align:center;margin:20px;background:#f5f5f5}";
    html += ".container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:10px;box-shadow:0 4px 8px rgba(0,0,0,0.1)}";
    html += ".status{background:#e8f5e8;padding:15px;margin:10px 0;border-radius:5px;border-left:5px solid #28a745}";
    html += ".motor{display:inline-block;background:#f0f8ff;padding:10px;margin:5px;border-radius:5px;min-width:100px}";
    html += ".btn{padding:12px 24px;margin:8px;border:none;border-radius:5px;cursor:pointer;font-size:16px}";
    html += ".btn-danger{background:#dc3545;color:white}.btn-success{background:#28a745;color:white}";
    html += ".stream{border:3px solid #007bff;border-radius:10px;max-width:100%}</style></head>";
    html += "<body><div class='container'>";
    html += "<h1>🤖 Robot Visión Artificial</h1>";
    html += "<div class='status'><h3>📊 Estado del Sistema</h3>";
    html += "<p><strong>🌐 IP:</strong> " + WiFi.localIP().toString() + "</p>";
    html += "<p><strong>📡 WiFi:</strong> " + String(ssid) + " (" + String(WiFi.RSSI()) + " dBm)</p>";
    html += "<p><strong>🎮 Estado:</strong> " + String(robot_active ? "ACTIVO" : "INACTIVO") + "</p>";
    html += "<p><strong>📤 Comandos:</strong> " + String(command_count) + "</p>";
    html += "<p><strong>💾 RAM Libre:</strong> " + String(ESP.getFreeHeap()) + " bytes</p></div>";
    html += "<div class='status'><h3>⚙️ Estado Motores</h3>";
    html += "<div class='motor'>Motor A<br><strong>" + String(motorA_speed) + "</strong></div>";
    html += "<div class='motor'>Motor B<br><strong>" + String(motorB_speed) + "</strong></div>";
    html += "<div class='motor'>Motor C<br><strong>" + String(motorC_speed) + "</strong></div></div>";
    html += "<div class='status'><h3>📹 Cámara en Vivo</h3>";
    html += "<img class='stream' src='/stream' alt='Video Stream'></div>";
    html += "<div style='margin:20px 0'>";
    html += "<button class='btn btn-danger' onclick=\"fetch('/stop')\">🛑 PARAR ROBOT</button>";
    html += "<button class='btn btn-success' onclick=\"location.reload()\">🔄 ACTUALIZAR</button>";
    html += "</div></div></body></html>";
    server.send(200, "text/html", html);
  });
  
  // Stream de video optimizado para Python
  server.on("/stream", HTTP_GET, []() {
    WiFiClient client = server.client();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println("Access-Control-Allow-Origin: *");
    client.println();
    
    while (client.connected()) {
      camera_fb_t * fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("⚠️ Error captura stream");
        delay(100);
        continue;
      }
      
      client.println("--frame");
      client.println("Content-Type: image/jpeg");
      client.printf("Content-Length: %u\r\n\r\n", fb->len);
      
      if (client.write(fb->buf, fb->len) != fb->len) {
        esp_camera_fb_return(fb);
        break;
      }
      client.println();
      esp_camera_fb_return(fb);
      
      delay(50); // ~20 FPS
    }
  });
  
  // Captura individual para Python IA
  server.on("/capture", HTTP_GET, []() {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      server.send(500, "text/plain", "Camera error");
      return;
    }
    
    server.sendHeader("Content-Type", "image/jpeg");
    server.sendHeader("Content-Length", String(fb->len));
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache");
    
    server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  });
  
  // Control omnidireccional desde Python
  server.on("/control", HTTP_POST, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    
    if (server.hasArg("plain")) {
      String json = server.arg("plain");
      DynamicJsonDocument doc(1024);
      
      if (deserializeJson(doc, json) == DeserializationError::Ok) {
        if (doc.containsKey("x") && doc.containsKey("y") && doc.containsKey("rotation")) {
          float x = doc["x"];
          float y = doc["y"];
          float rotation = doc["rotation"];
          
          // Aplicar movimiento omnidireccional
          moveOmnidirectional(x, y, rotation);
          command_count++;
          last_command = millis();
          robot_active = (x != 0 || y != 0 || rotation != 0);
          
          server.send(200, "application/json", 
                     "{\"status\":\"OK\",\"x\":" + String(x) + 
                     ",\"y\":" + String(y) + ",\"rotation\":" + String(rotation) + "}");
        } else {
          server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
        }
      } else {
        server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      }
    } else {
      server.send(400, "application/json", "{\"error\":\"No data\"}");
    }
  });
  
  // Parada de emergencia
  server.on("/stop", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    stopAllMotors();
    robot_active = false;
    Serial.println("🛑 Parada de emergencia activada");
    server.send(200, "application/json", "{\"status\":\"Emergency stop\"}");
  });
  
  // Estado del sistema
  server.on("/status", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    String json = "{";
    json += "\"motor_a\":" + String(motorA_speed) + ",";
    json += "\"motor_b\":" + String(motorB_speed) + ",";
    json += "\"motor_c\":" + String(motorC_speed) + ",";
    json += "\"active\":" + String(robot_active ? "true" : "false") + ",";
    json += "\"commands\":" + String(command_count) + ",";
    json += "\"uptime\":" + String(millis()/1000) + ",";
    json += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
    json += "\"wifi_rssi\":" + String(WiFi.RSSI());
    json += "}";
    server.send(200, "application/json", json);
  });
  
  server.begin();
}

void moveOmnidirectional(float x, float y, float rotation) {
  // Cálculo omnidireccional para 3 motores a 120°
  // Adaptado para tu configuración específica
  
  float motorA = y + rotation;                    // Motor frontal
  float motorB = -0.5 * y + 0.866 * x + rotation; // Motor izquierdo
  float motorC = -0.5 * y - 0.866 * x + rotation; // Motor derecho
  
  // Convertir a velocidades de motor (-255 a 255)
  int speedA = constrain((int)(motorA * 255), -255, 255);
  int speedB = constrain((int)(motorB * 255), -255, 255);
  int speedC = constrain((int)(motorC * 255), -255, 255);
  
  // Aplicar velocidades
  controlMotorA(speedA);
  controlMotorB(speedB);
  controlMotorC(speedC);
  
  Serial.printf("🎮 Omnidireccional: A=%d B=%d C=%d (X=%.2f Y=%.2f R=%.2f)\n", 
                speedA, speedB, speedC, x, y, rotation);
}

void controlMotorA(int speed) {
  motorA_speed = constrain(speed, -255, 255);
  
  if (motorA_speed > 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else if (motorA_speed < 0) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  }
}

void controlMotorB(int speed) {
  motorB_speed = constrain(speed, -255, 255);
  
  if (motorB_speed > 0) {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else if (motorB_speed < 0) {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
  }
}

void controlMotorC(int speed) {
  motorC_speed = constrain(speed, -255, 255);
  
  if (motorC_speed > 0) {
    digitalWrite(MOTOR_C_IN1, HIGH);
    digitalWrite(MOTOR_C_IN2, LOW);
  } else if (motorC_speed < 0) {
    digitalWrite(MOTOR_C_IN1, LOW);
    digitalWrite(MOTOR_C_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_C_IN1, LOW);
    digitalWrite(MOTOR_C_IN2, LOW);
  }
}

void stopAllMotors() {
  controlMotorA(0);
  controlMotorB(0);
  controlMotorC(0);
  motorA_speed = 0;
  motorB_speed = 0;
  motorC_speed = 0;
  robot_active = false;
}

void loop() {
  // Manejar peticiones web
  server.handleClient();
  
  // Watchdog WiFi
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 30000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠️ WiFi desconectado - reconectando");
      WiFi.begin(ssid, password);
    }
    lastWiFiCheck = millis();
  }
  
  // Parada de seguridad por timeout
  if (robot_active && millis() - last_command > 5000) {
    Serial.println("⚠️ Timeout comando - parada seguridad");
    stopAllMotors();
  }
  
  // Heartbeat cada 30 segundos
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 30000) {
    Serial.printf("💓 Sistema: RAM=%d, Comandos=%lu, Estado=%s\n", 
                  ESP.getFreeHeap(), command_count, robot_active ? "ACTIVO" : "INACTIVO");
    lastHeartbeat = millis();
  }
  
  delay(10);
}