/*
 * Robot Seguidor de Línea con ESP32-CAM - Servidor Web y Visión
 * Configuración física REAL según tu robot:
 * 
 * Motor A: FRONTAL IZQUIERDO  (GPIO 2,4)
 * Motor B: TRASERO CENTRAL    (GPIO 12,13) - SE MANTIENE PARADO
 * Motor C: FRONTAL DERECHO    (GPIO 14,15)
 * 
 * MOVIMIENTO:
 * - ADELANTE: Solo motores A y C activos
 * - ATRÁS: Motores A y C en reversa  
 * - GIRO: Un motor más rápido que el otro
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// **********************************************************
//                  CONFIGURACIÓN WIFI
// **********************************************************
const char* ssid = "Esteban";
const char* password = "chanchan";

// **********************************************************
//                  DEFINICIÓN DE PINES
// **********************************************************
// Pines para motores (CONFIGURACIÓN CORRECTA SEGÚN TU ROBOT)
#define MOTOR_A_IN3 2   // Motor A - FRONTAL IZQUIERDO
#define MOTOR_A_IN4 4
#define MOTOR_B_IN1 12  // Motor B - TRASERO CENTRAL (se mantiene parado)
#define MOTOR_B_IN2 13
#define MOTOR_C_IN3 14  // Motor C - FRONTAL DERECHO
#define MOTOR_C_IN4 15

// Pines para cámara ESP32-CAM
#define PWDN_GPIO_NUM       32
#define RESET_GPIO_NUM      -1
#define XCLK_GPIO_NUM        0
#define SIOD_GPIO_NUM       26
#define SIOC_GPIO_NUM       27
#define Y9_GPIO_NUM         35
#define Y8_GPIO_NUM         34
#define Y7_GPIO_NUM         39
#define Y6_GPIO_NUM         36
#define Y5_GPIO_NUM         21
#define Y4_GPIO_NUM         19
#define Y3_GPIO_NUM         18
#define Y2_GPIO_NUM          5
#define VSYNC_GPIO_NUM      25
#define HREF_GPIO_NUM       23
#define PCLK_GPIO_NUM       22

// **********************************************************
//                  OBJETOS GLOBALES
// **********************************************************
WebServer server(80);
QueueHandle_t motorCommandQueue;

// Estructura para comandos de motor simplificada
typedef struct {
    String command;  // "forward", "backward", "left", "right", "stop"
    int speed;       // 0-255
} MotorCommand_t;

// **********************************************************
//                  VARIABLES DE ESTADO
// **********************************************************
bool autoMode = false;
String currentMovement = "stop";
int currentSpeed = 150;
unsigned long lastMotorCommandTime = 0;
const unsigned long MOTOR_UPDATE_INTERVAL_MS = 100;

// Variables para detección de línea
const int THRESHOLD_VAL = 80;
const int ROI_HEIGHT = 60;
float line_position = 0.0;
bool line_detected = false;

// **********************************************************
//                  FUNCIONES DE MOTOR (CONFIGURACIÓN CORRECTA)
// **********************************************************
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

void stopAllMotors() {
    setMotor(MOTOR_A_IN3, MOTOR_A_IN4, 0, true);
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);  // Motor B siempre parado
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, 0, true);
    currentMovement = "stop";
    Serial.println(">>> TODOS LOS MOTORES DETENIDOS <<<");
}

// FUNCIONES DE MOVIMIENTO SEGÚN TU CONFIGURACIÓN
void moveForward(int speed = 200) {
    // Solo motores frontales A y C
    setMotor(MOTOR_A_IN3, MOTOR_A_IN4, speed, true);   // Motor A adelante
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);       // Motor B parado
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, speed, true);   // Motor C adelante
    currentMovement = "forward";
    Serial.printf("ADELANTE - Velocidad: %d\n", speed);
}

void moveBackward(int speed = 200) {
    // Solo motores frontales A y C en reversa
    setMotor(MOTOR_A_IN3, MOTOR_A_IN4, speed, false);  // Motor A atrás
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);       // Motor B parado
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, speed, false);  // Motor C atrás
    currentMovement = "backward";
    Serial.printf("ATRÁS - Velocidad: %d\n", speed);
}

void turnLeft(int speed = 150) {
    // Motor A más lento, Motor C normal (gira a la izquierda)
    setMotor(MOTOR_A_IN3, MOTOR_A_IN4, speed/2, true);  // Motor A lento
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);        // Motor B parado
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, speed, true);    // Motor C normal
    currentMovement = "left";
    Serial.printf("GIRO IZQUIERDA - Velocidad: %d\n", speed);
}

void turnRight(int speed = 150) {
    // Motor A normal, Motor C más lento (gira a la derecha)
    setMotor(MOTOR_A_IN3, MOTOR_A_IN4, speed, true);    // Motor A normal
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);        // Motor B parado
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, speed/2, true);  // Motor C lento
    currentMovement = "right";
    Serial.printf("GIRO DERECHA - Velocidad: %d\n", speed);
}

void rotateLeft(int speed = 150) {
    // Motor A atrás, Motor C adelante (rotación en su lugar)
    setMotor(MOTOR_A_IN3, MOTOR_A_IN4, speed, false);   // Motor A atrás
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);        // Motor B parado
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, speed, true);    // Motor C adelante
    currentMovement = "rotate_left";
    Serial.printf("ROTACIÓN IZQUIERDA - Velocidad: %d\n", speed);
}

void rotateRight(int speed = 150) {
    // Motor A adelante, Motor C atrás (rotación en su lugar)
    setMotor(MOTOR_A_IN3, MOTOR_A_IN4, speed, true);    // Motor A adelante
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);        // Motor B parado
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, speed, false);   // Motor C atrás
    currentMovement = "rotate_right";
    Serial.printf("ROTACIÓN DERECHA - Velocidad: %d\n", speed);
}

// **********************************************************
//                  PROCESAMIENTO DE IMAGEN
// **********************************************************
float calculateLinePosition(uint8_t *binary_roi, int width, int height) {
    if (height <= 0 || width <= 0) return 0.0;

    int line_pixels_x_sum = 0;
    int line_pixels_count = 0;
    
    // Analizar las últimas filas (más cercanas al robot)
    int rows_to_analyze = min(15, height);
    for (int y = height - rows_to_analyze; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (binary_roi[y * width + x] > 0) {
                line_pixels_x_sum += x;
                line_pixels_count++;
            }
        }
    }

    if (line_pixels_count > (width * rows_to_analyze * 0.02)) {
        float line_center_x = (float)line_pixels_x_sum / line_pixels_count;
        // Normalizar: -1 (izquierda) a 1 (derecha), 0 es centro
        return (line_center_x - width / 2.0) / (width / 2.0);
    }
    return 0.0; // No se detectó línea
}

void processImageForLineFollowing(camera_fb_t *fb) {
    if (!autoMode) return;

    uint8_t *gray_image = fb->buf;
    int img_width = fb->width;
    int img_height = fb->height;
    
    // Crear buffer para imagen binarizada
    static uint8_t *binary_buffer = NULL;
    static size_t buffer_size = 0;
    
    size_t needed_size = img_width * img_height;
    if (binary_buffer == NULL || buffer_size != needed_size) {
        if (binary_buffer != NULL) free(binary_buffer);
        binary_buffer = (uint8_t*)malloc(needed_size);
        buffer_size = needed_size;
    }

    // Binarización
    for (int i = 0; i < needed_size; i++) {
        binary_buffer[i] = (gray_image[i] < THRESHOLD_VAL) ? 255 : 0;
    }

    // Definir ROI (región de interés)
    int roi_start_y = img_height - ROI_HEIGHT;
    if (roi_start_y < 0) roi_start_y = 0;
    
    uint8_t *roi_ptr = binary_buffer + roi_start_y * img_width;
    
    // Calcular posición de la línea
    line_position = calculateLinePosition(roi_ptr, img_width, ROI_HEIGHT);
    line_detected = (line_position != 0.0);
    
    // Control del robot basado en la posición de la línea
    if (line_detected) {
        int base_speed = 180;
        
        if (abs(line_position) < 0.1) {
            // Línea centrada - avanzar
            moveForward(base_speed);
        } else if (line_position < -0.1) {
            // Línea a la izquierda - girar izquierda
            turnLeft(base_speed);
        } else if (line_position > 0.1) {
            // Línea a la derecha - girar derecha
            turnRight(base_speed);
        }
        
        Serial.printf("Línea detectada en posición: %.3f\n", line_position);
    } else {
        // No se detectó línea - detener
        stopAllMotors();
        Serial.println("Línea no detectada - Deteniendo");
    }
}

// **********************************************************
//                  TAREAS DE FREERTOS
// **********************************************************
void visionTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 FPS
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        if (autoMode) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                processImageForLineFollowing(fb);
                esp_camera_fb_return(fb);
            }
        }
    }
}

void motorTask(void *pvParameters) {
    MotorCommand_t received_cmd;
    
    while (true) {
        if (xQueueReceive(motorCommandQueue, &received_cmd, pdMS_TO_TICKS(50)) == pdPASS) {
            if (!autoMode) { // Solo ejecutar comandos manuales si no está en modo automático
                executeMotorCommand(received_cmd.command, received_cmd.speed);
                lastMotorCommandTime = millis();
            }
        }
        
        // Si no hay comandos recientes y no está en automático, detener
        if (!autoMode && (millis() - lastMotorCommandTime > 2000)) {
            stopAllMotors();
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void executeMotorCommand(String command, int speed) {
    if (command == "forward") {
        moveForward(speed);
    } else if (command == "backward") {
        moveBackward(speed);
    } else if (command == "left") {
        turnLeft(speed);
    } else if (command == "right") {
        turnRight(speed);
    } else if (command == "rotate_left") {
        rotateLeft(speed);
    } else if (command == "rotate_right") {
        rotateRight(speed);
    } else if (command == "stop") {
        stopAllMotors();
    }
}

// **********************************************************
//                  MANEJADORES WEB
// **********************************************************
void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<title>Robot Seguidor de Línea ESP32-CAM</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>";
    html += "body { font-family: Arial; text-align: center; margin: 20px; background: #f0f0f0; }";
    html += ".container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }";
    html += ".status { background: #e3f2fd; padding: 15px; margin: 15px 0; border-radius: 8px; border-left: 4px solid #2196F3; }";
    html += ".controls { margin: 20px 0; }";
    html += "button { padding: 12px 24px; margin: 8px; font-size: 16px; border: none; border-radius: 6px; cursor: pointer; transition: all 0.3s; }";
    html += ".btn-primary { background: #2196F3; color: white; }";
    html += ".btn-success { background: #4CAF50; color: white; }";
    html += ".btn-danger { background: #f44336; color: white; }";
    html += ".btn-warning { background: #FF9800; color: white; }";
    html += "button:hover { transform: translateY(-2px); box-shadow: 0 4px 8px rgba(0,0,0,0.2); }";
    html += "#video { max-width: 100%; height: auto; border: 2px solid #ddd; border-radius: 8px; margin: 10px 0; }";
    html += ".grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; max-width: 300px; margin: 0 auto; }";
    html += "</style></head><body>";
    
    html += "<div class='container'>";
    html += "<h1>🤖 Robot Seguidor de Línea</h1>";
    
    html += "<div class='status'>";
    html += "<h3>📊 Estado Actual</h3>";
    html += "<p><strong>Modo:</strong> <span id='modeStatus'>" + String(autoMode ? "🤖 Automático" : "🎮 Manual") + "</span></p>";
    html += "<p><strong>Movimiento:</strong> <span id='moveStatus'>" + currentMovement + "</span></p>";
    html += "<p><strong>IP:</strong> " + WiFi.localIP().toString() + "</p>";
    if (autoMode) {
        html += "<p><strong>Línea:</strong> <span id='lineStatus'>" + String(line_detected ? "✅ Detectada" : "❌ No detectada") + "</span></p>";
        html += "<p><strong>Posición:</strong> <span id='posStatus'>" + String(line_position, 3) + "</span></p>";
    }
    html += "</div>";
    
    html += "<div><img id='video' src='/stream' alt='Cámara del Robot'></div>";
    
    html += "<div class='controls'>";
    html += "<h3>🎮 Control de Modo</h3>";
    html += "<button onclick='toggleAuto()' id='autoBtn' class='" + String(autoMode ? "btn-danger" : "btn-success") + "'>";
    html += autoMode ? "🛑 Desactivar IA" : "🤖 Activar IA";
    html += "</button>";
    html += "<button onclick='sendStop()' class='btn-warning'>⏹️ DETENER</button>";
    html += "</div>";
    
    if (!autoMode) {
        html += "<div class='controls'>";
        html += "<h3>🕹️ Control Manual</h3>";
        html += "<div class='grid'>";
        html += "<button onclick='sendMove(\"rotate_left\")' class='btn-primary'>⤺ Rotar Izq</button>";
        html += "<button onclick='sendMove(\"forward\")' class='btn-primary'>⬆️ Adelante</button>";
        html += "<button onclick='sendMove(\"rotate_right\")' class='btn-primary'>⤻ Rotar Der</button>";
        html += "<button onclick='sendMove(\"left\")' class='btn-primary'>⬅️ Izquierda</button>";
        html += "<button onclick='sendMove(\"stop\")' class='btn-danger'>⏸️ STOP</button>";
        html += "<button onclick='sendMove(\"right\")' class='btn-primary'>➡️ Derecha</button>";
        html += "<button></button>";
        html += "<button onclick='sendMove(\"backward\")' class='btn-primary'>⬇️ Atrás</button>";
        html += "<button></button>";
        html += "</div></div>";
    }
    
    html += "</div>";
    
    html += "<script>";
    html += "var autoModeJs = " + String(autoMode ? "true" : "false") + ";";
    
    html += "function toggleAuto() {";
    html += "   autoModeJs = !autoModeJs;";
    html += "   fetch('/auto', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({auto: autoModeJs})})";
    html += "   .then(() => location.reload());";
    html += "}";
    
    html += "function sendStop() {";
    html += "   fetch('/stop', {method: 'POST'}).then(() => location.reload());";
    html += "}";
    
    html += "function sendMove(direction) {";
    html += "   fetch('/move', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({direction: direction, speed: 150})});";
    html += "}";
    
    html += "setInterval(() => {";
    html += "   if (autoModeJs) location.reload();";
    html += "}, 2000);";
    
    html += "</script></body></html>";
    
    server.send(200, "text/html", html);
}

void handleMove() {
    if (server.hasArg("plain")) {
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, server.arg("plain"));
        
        String direction = doc["direction"];
        int speed = doc["speed"] | 150;
        
        if (!autoMode) {
            MotorCommand_t cmd = {direction, speed};
            xQueueSend(motorCommandQueue, &cmd, 0);
            server.send(200, "application/json", "{\"status\":\"ok\", \"direction\":\"" + direction + "\"}");
        } else {
            server.send(403, "application/json", "{\"status\":\"error\", \"message\":\"Robot en modo automatico\"}");
        }
    } else {
        server.send(400, "application/json", "{\"status\":\"error\", \"message\":\"Datos faltantes\"}");
    }
}

void handleStop() {
    autoMode = false;
    stopAllMotors();
    server.send(200, "application/json", "{\"status\":\"stopped\"}");
}

void handleAutoMode() {
    if (server.hasArg("plain")) {
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, server.arg("plain"));
        autoMode = doc["auto"];
        
        stopAllMotors(); // Detener al cambiar modo
        
        Serial.println("Modo automático: " + String(autoMode ? "ON" : "OFF"));
        server.send(200, "application/json", "{\"status\":\"ok\", \"auto\":\"" + String(autoMode ? "true" : "false") + "\"}");
    }
}

void handleStream() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Error capturando imagen");
        return;
    }
    
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache");
    
    if (fb->format == PIXFORMAT_GRAYSCALE) {
        uint8_t *jpeg_buf = NULL;
        size_t jpeg_len = 0;
        
        if (frame2jpg(fb, 80, &jpeg_buf, &jpeg_len)) {
            server.send_P(200, "image/jpeg", (const char *)jpeg_buf, jpeg_len);
            if (jpeg_buf) free(jpeg_buf);
        } else {
            server.send(500, "text/plain", "Error de conversión JPEG");
        }
    } else {
        server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
    }
    
    esp_camera_fb_return(fb);
}

// **********************************************************
//                  SETUP Y LOOP
// **********************************************************
void setup() {
    Serial.begin(115200);
    Serial.println("\n==============================================");
    Serial.println("    ROBOT SEGUIDOR DE LÍNEA ESP32-CAM");
    Serial.println("==============================================");
    
    // Configurar pines de motores
    pinMode(MOTOR_A_IN3, OUTPUT);
    pinMode(MOTOR_A_IN4, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_C_IN3, OUTPUT);
    pinMode(MOTOR_C_IN4, OUTPUT);
    stopAllMotors();
    
    // Crear cola de comandos
    motorCommandQueue = xQueueCreate(10, sizeof(MotorCommand_t));
    
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
    config.pixel_format = PIXFORMAT_GRAYSCALE; // Escala de grises para procesamiento
    
    if(psramFound()){
        config.frame_size = FRAMESIZE_QVGA;     // 320x240
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_QQVGA;    // 160x120
        config.jpeg_quality = 10;
        config.fb_count = 1;
    }
    
    // Inicializar cámara
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error inicializando cámara: 0x%x\n", err);
        ESP.restart();
        return;
    }
    
    // Conectar WiFi
    WiFi.begin(ssid, password);
    Serial.print("Conectando a WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    // Configurar servidor web
    server.on("/", HTTP_GET, handleRoot);
    server.on("/move", HTTP_POST, handleMove);
    server.on("/stop", HTTP_POST, handleStop);
    server.on("/auto", HTTP_POST, handleAutoMode);
    server.on("/stream", HTTP_GET, handleStream);
    server.enableCORS(true);
    
    server.begin();
    Serial.println("Servidor web iniciado");
    
    // Crear tareas
    xTaskCreatePinnedToCore(visionTask, "VisionTask", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, NULL, 4, NULL, 0);
    
    Serial.println("==============================================");
    Serial.println("Robot listo. Accede desde: http://" + WiFi.localIP().toString());
    Serial.println("==============================================");
}

void loop() {
    server.handleClient();
    delay(1);
}