/*
 * Robot Omnidireccional con ESP32-CAM y Visión por Computadora
 * Autor: Asistente IA
 * Descripción: Control de 3 motores omnidireccionales con servidor web y cámara
 * 
 * Configuración de Motores:
 * Motor A (DRV#1): OUT3-4, IN4→GPIO4, IN3→GPIO2
 * Motor B (DRV#2): OUT1-2, IN2→GPIO13, IN1→GPIO12
 * Motor C (DRV#2): OUT3-4, IN3→GPIO14, IN4→GPIO15
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// Configuración WiFi
const char* ssid = "Esteban";
const char* password = "chanchan";

// Servidor web
WebServer server(80);

// Definición de pines para motores
// Motor A (DRV#1)
#define MOTOR_A_IN3 2
#define MOTOR_A_IN4 4

// Motor B (DRV#2) 
#define MOTOR_B_IN1 12
#define MOTOR_B_IN2 13

// Motor C (DRV#2)
#define MOTOR_C_IN3 14
#define MOTOR_C_IN4 15

// Variables de control
int motorSpeed = 200; // Velocidad PWM (0-255)
bool autoMode = false; // Modo automático con IA

// Configuración de cámara para ESP32-CAM
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

void setup() {
  Serial.begin(115200);
  
  // Configurar pines de motores como salida
  pinMode(MOTOR_A_IN3, OUTPUT);
  pinMode(MOTOR_A_IN4, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_C_IN3, OUTPUT);
  pinMode(MOTOR_C_IN4, OUTPUT);
  
  // Detener todos los motores inicialmente
  stopAllMotors();
  
  // Configurar cámara
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
  
  // Configuración de resolución
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Inicializar cámara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error inicializando cámara: 0x%x", err);
    return;
  }
  
  // Conectar a WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("WiFi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  
  // Configurar rutas del servidor web
  server.on("/", handleRoot);
  server.on("/move", HTTP_POST, handleMove);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/auto", HTTP_POST, handleAutoMode);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/stream", HTTP_GET, handleStream);
  server.enableCORS(true);
  
  server.begin();
  Serial.println("Servidor web iniciado");
}

void loop() {
  server.handleClient();
  delay(10);
}

// Página web principal
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Robot Omnidireccional</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial; text-align: center; margin: 20px; }";
  html += ".controls { margin: 20px; }";
  html += "button { padding: 15px 30px; margin: 10px; font-size: 16px; }";
  html += ".speed-control { margin: 20px; }";
  html += "#video { max-width: 100%; height: auto; }";
  html += ".status { background: #f0f0f0; padding: 10px; margin: 10px; }";
  html += "</style></head><body>";
  
  html += "<h1>Robot Omnidireccional con IA</h1>";
  html += "<div class='status'>";
  html += "<h3>Estado: <span id='status'>Manual</span></h3>";
  html += "<p>IP: " + WiFi.localIP().toString() + "</p>";
  html += "</div>";
  
  html += "<div><img id='video' src='/stream' alt='Camara del Robot'></div>";
  
  html += "<div class='controls'>";
  html += "<h3>Control Manual</h3>";
  html += "<button onclick='move(\"forward\")'>ADELANTE</button><br>";
  html += "<button onclick='move(\"left\")'>IZQUIERDA</button>";
  html += "<button onclick='move(\"stop\")'>STOP</button>";
  html += "<button onclick='move(\"right\")'>DERECHA</button><br>";
  html += "<button onclick='move(\"backward\")'>ATRAS</button><br>";
  html += "<button onclick='move(\"rotate_left\")'>ROTAR IZQ</button>";
  html += "<button onclick='move(\"rotate_right\")'>ROTAR DER</button>";
  html += "</div>";
  
  html += "<div class='speed-control'>";
  html += "<label>Velocidad: </label>";
  html += "<input type='range' id='speed' min='100' max='255' value='200' onchange='updateSpeed()'>";
  html += "<span id='speedValue'>200</span>";
  html += "</div>";
  
  html += "<div class='controls'>";
  html += "<button onclick='toggleAuto()' id='autoBtn' style='background-color: #4CAF50;'>ACTIVAR IA</button>";
  html += "</div>";
  
  // JavaScript
  html += "<script>";
  html += "var autoMode = false;";
  
  html += "function move(direction) {";
  html += "  if (autoMode) return;";
  html += "  var xhr = new XMLHttpRequest();";
  html += "  xhr.open('POST', '/move', true);";
  html += "  xhr.setRequestHeader('Content-Type', 'application/json');";
  html += "  xhr.send(JSON.stringify({direction: direction, speed: document.getElementById('speed').value}));";
  html += "}";
  
  html += "function updateSpeed() {";
  html += "  var speed = document.getElementById('speed').value;";
  html += "  document.getElementById('speedValue').textContent = speed;";
  html += "}";
  
  html += "function toggleAuto() {";
  html += "  autoMode = !autoMode;";
  html += "  var btn = document.getElementById('autoBtn');";
  html += "  var status = document.getElementById('status');";
  html += "  if (autoMode) {";
  html += "    btn.textContent = 'DESACTIVAR IA';";
  html += "    btn.style.backgroundColor = '#f44336';";
  html += "    status.textContent = 'Automatico (IA)';";
  html += "  } else {";
  html += "    btn.textContent = 'ACTIVAR IA';";
  html += "    btn.style.backgroundColor = '#4CAF50';";
  html += "    status.textContent = 'Manual';";
  html += "  }";
  html += "  var xhr = new XMLHttpRequest();";
  html += "  xhr.open('POST', '/auto', true);";
  html += "  xhr.setRequestHeader('Content-Type', 'application/json');";
  html += "  xhr.send(JSON.stringify({auto: autoMode}));";
  html += "}";
  
  html += "function updateVideo() {";
  html += "  document.getElementById('video').src = '/stream?' + new Date().getTime();";
  html += "}";
  
  html += "setInterval(updateVideo, 100);";
  html += "</script>";
  
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

// Manejar comandos de movimiento
void handleMove() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    
    String direction = doc["direction"];
    int speed = doc["speed"] | 200;
    motorSpeed = speed;
    
    Serial.println("Movimiento: " + direction + ", Velocidad: " + String(speed));
    
    if (direction == "forward") {
      moveForward();
    } else if (direction == "backward") {
      moveBackward();
    } else if (direction == "left") {
      moveLeft();
    } else if (direction == "right") {
      moveRight();
    } else if (direction == "rotate_left") {
      rotateLeft();
    } else if (direction == "rotate_right") {
      rotateRight();
    } else if (direction == "stop") {
      stopAllMotors();
    }
  }
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// Manejar comando de parada
void handleStop() {
  stopAllMotors();
  server.send(200, "application/json", "{\"status\":\"stopped\"}");
}

// Manejar modo automático
void handleAutoMode() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    autoMode = doc["auto"];
    Serial.println("Modo automático: " + String(autoMode ? "ON" : "OFF"));
  }
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// Capturar imagen
void handleCapture() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Error capturando imagen");
    return;
  }
  
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// Stream de video
void handleStream() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Error capturando imagen");
    return;
  }
  
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// Funciones de movimiento omnidireccional
void moveForward() {
  // Los 3 motores van hacia adelante
  setMotor(MOTOR_A_IN3, MOTOR_A_IN4, motorSpeed, true);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, motorSpeed, true);
  setMotor(MOTOR_C_IN3, MOTOR_C_IN4, motorSpeed, true);
}

void moveBackward() {
  // Los 3 motores van hacia atrás
  setMotor(MOTOR_A_IN3, MOTOR_A_IN4, motorSpeed, false);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, motorSpeed, false);
  setMotor(MOTOR_C_IN3, MOTOR_C_IN4, motorSpeed, false);
}

void moveLeft() {
  // Movimiento lateral izquierdo
  setMotor(MOTOR_A_IN3, MOTOR_A_IN4, motorSpeed, false);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, motorSpeed, true);
  setMotor(MOTOR_C_IN3, MOTOR_C_IN4, 0, true); // Motor C se detiene o gira lento
}

void moveRight() {
  // Movimiento lateral derecho
  setMotor(MOTOR_A_IN3, MOTOR_A_IN4, motorSpeed, true);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, motorSpeed, false);
  setMotor(MOTOR_C_IN3, MOTOR_C_IN4, 0, true); // Motor C se detiene o gira lento
}

void rotateLeft() {
  // Rotación hacia la izquierda
  setMotor(MOTOR_A_IN3, MOTOR_A_IN4, motorSpeed, false);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, motorSpeed, false);
  setMotor(MOTOR_C_IN3, MOTOR_C_IN4, motorSpeed, false);
}

void rotateRight() {
  // Rotación hacia la derecha
  setMotor(MOTOR_A_IN3, MOTOR_A_IN4, motorSpeed, true);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, motorSpeed, true);
  setMotor(MOTOR_C_IN3, MOTOR_C_IN4, motorSpeed, true);
}

void stopAllMotors() {
  setMotor(MOTOR_A_IN3, MOTOR_A_IN4, 0, true);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
  setMotor(MOTOR_C_IN3, MOTOR_C_IN4, 0, true);
}

// Función auxiliar para controlar un motor
void setMotor(int in1, int in2, int speed, bool forward) {
  if (speed == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else if (forward) {
    analogWrite(in1, speed);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    analogWrite(in2, speed);
  }
}

// Función para movimiento desde IA (llamada por Python)
void moveFromAI(float vx, float vy, float omega) {
  if (!autoMode) return;
  
  // Cinemática inversa para robot omnidireccional de 3 ruedas
  // Ángulos de las ruedas: 0°, 120°, 240°
  float v1 = vx;
  float v2 = -0.5 * vx + 0.866 * vy;
  float v3 = -0.5 * vx - 0.866 * vy;
  
  // Agregar componente rotacional
  float radius = 0.1; // Radio del robot en metros
  v1 += omega * radius;
  v2 += omega * radius;
  v3 += omega * radius;
  
  // Normalizar y aplicar velocidades
  float maxV = max(abs(v1), max(abs(v2), abs(v3)));
  if (maxV > 1.0) {
    v1 /= maxV;
    v2 /= maxV;
    v3 /= maxV;
  }
  
  // Aplicar velocidades a motores
  setMotor(MOTOR_A_IN3, MOTOR_A_IN4, abs(v1) * 255, v1 > 0);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, abs(v2) * 255, v2 > 0);
  setMotor(MOTOR_C_IN3, MOTOR_C_IN4, abs(v3) * 255, v3 > 0);
}