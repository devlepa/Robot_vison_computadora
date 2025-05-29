/*
 * ROBOT SEGUIDOR DE LINEA ESP32-CAM - OPTIMIZADO POR CORES
 * =========================================================
 * 
 * DISTRIBUCION OPTIMIZADA DE TAREAS:
 * - CORE 0: Control de motores y GPIO (tiempo real)
 * - CORE 1: WiFi, HTTP, camara y comunicacion Python
 * 
 * CONFIGURACION FISICA:
 * - Motor A (GPIO 2,4): Frontal Izquierdo
 * - Motor B (GPIO 12,13): Trasero Central (parado)  
 * - Motor C (GPIO 14,15): Frontal Derecho
 * 
 * OPTIMIZACIONES:
 * - Velocidades reducidas para mejor sincronizacion
 * - Comunicacion HTTP simplificada
 * - Procesamiento de imagen optimizado
 * - Mejor gestion de memoria
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ==========================================
// CONFIGURACION WIFI
// ==========================================
const char* ssid = "Esteban";
const char* password = "chanchan";

// ==========================================
// PINES DE HARDWARE
// ==========================================
#define MOTOR_A_IN3 2
#define MOTOR_A_IN4 4
#define MOTOR_B_IN1 12
#define MOTOR_B_IN2 13
#define MOTOR_C_IN3 14
#define MOTOR_C_IN4 15

// Pines de camara ESP32-CAM
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

// ==========================================
// ESTRUCTURAS Y VARIABLES GLOBALES
// ==========================================

WebServer server(80);

// Comando simple para motores
typedef struct {
    String comando;
    int velocidad;
} ComandoMotor_t;

// Parametros de vision simplificados
typedef struct {
    int umbral;
    int altura_roi;
    float zona_muerta;
    int vel_base;
    int vel_giro;
} Parametros_t;

// Colas para comunicacion entre cores
QueueHandle_t cola_comandos_motor;
QueueHandle_t cola_parametros;
SemaphoreHandle_t mutex_estado;

// Variables de estado (protegidas por mutex)
volatile bool modo_automatico = false;
volatile bool sistema_activo = true;
String estado_actual = "parado";

// Parametros optimizados (velocidades reducidas)
Parametros_t params = {
    .umbral = 80,
    .altura_roi = 50,        // ROI mas pequena
    .zona_muerta = 0.12f,    // Zona muerta mas grande
    .vel_base = 180,         // Velocidad reducida
    .vel_giro = 140          // Velocidad de giro reducida
};

// Variables de deteccion
float posicion_linea = 0.0f;
bool linea_detectada = false;
int pixels_detectados = 0;

// Buffer de imagen simplificado
static uint8_t* buffer_imagen = NULL;
static size_t tamano_buffer = 0;

// ==========================================
// FUNCIONES DE CONTROL DE MOTORES (CORE 0)
// ==========================================

void controlarMotor(int pin1, int pin2, int velocidad, bool adelante) {
    if (velocidad == 0) {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
    } else if (adelante) {
        analogWrite(pin1, velocidad);
        digitalWrite(pin2, LOW);
    } else {
        digitalWrite(pin1, LOW);
        analogWrite(pin2, velocidad);
    }
}

void detenerMotores() {
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, 0, true);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, 0, true);
    estado_actual = "parado";
}

void moverAdelante(int vel = 0) {
    if (vel == 0) vel = params.vel_base;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel, true);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel, true);
    estado_actual = "adelante";
}

void moverAtras(int vel = 0) {
    if (vel == 0) vel = params.vel_base;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel, false);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel, false);
    estado_actual = "atras";
}

void girarIzquierda(int vel = 0) {
    if (vel == 0) vel = params.vel_giro;
    int vel_reducida = vel * 0.4f;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel_reducida, true);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel, true);
    estado_actual = "izquierda";
}

void girarDerecha(int vel = 0) {
    if (vel == 0) vel = params.vel_giro;
    int vel_reducida = vel * 0.4f;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel, true);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel_reducida, true);
    estado_actual = "derecha";
}

void rotarIzquierda(int vel = 0) {
    if (vel == 0) vel = params.vel_giro;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel, false);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel, true);
    estado_actual = "rotar_izq";
}

void rotarDerecha(int vel = 0) {
    if (vel == 0) vel = params.vel_giro;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel, true);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel, false);
    estado_actual = "rotar_der";
}

void ejecutarComando(String cmd, int vel) {
    if (cmd == "forward") moverAdelante(vel);
    else if (cmd == "backward") moverAtras(vel);
    else if (cmd == "left") girarIzquierda(vel);
    else if (cmd == "right") girarDerecha(vel);
    else if (cmd == "rotate_left") rotarIzquierda(vel);
    else if (cmd == "rotate_right") rotarDerecha(vel);
    else if (cmd == "stop") detenerMotores();
}

// ==========================================
// PROCESAMIENTO DE VISION SIMPLIFICADO
// ==========================================

float calcularCentroide(uint8_t* roi, int ancho, int alto) {
    int suma_x = 0;
    int contador = 0;
    
    // Analizar solo ultimas 15 filas
    int filas = min(15, alto);
    int inicio_y = alto - filas;
    
    for (int y = inicio_y; y < alto; y++) {
        for (int x = 0; x < ancho; x++) {
            if (roi[y * ancho + x] > 0) {
                suma_x += x;
                contador++;
            }
        }
    }
    
    pixels_detectados = contador;
    int min_pixels = (ancho * filas) * 0.02f; // 2% minimo
    
    if (contador >= min_pixels) {
        float centro_x = (float)suma_x / contador;
        return (centro_x - ancho/2.0f) / (ancho/2.0f);
    }
    return 0.0f;
}

void procesarVision(camera_fb_t* fb) {
    if (!fb || !modo_automatico) return;
    
    uint8_t* img = fb->buf;
    int ancho = fb->width;
    int alto = fb->height;
    size_t tamano = ancho * alto;
    
    // Gestionar buffer dinamicamente
    if (!buffer_imagen || tamano_buffer != tamano) {
        if (buffer_imagen) free(buffer_imagen);
        buffer_imagen = (uint8_t*)malloc(tamano);
        if (!buffer_imagen) return;
        tamano_buffer = tamano;
    }
    
    // Binarizacion simple
    for (int i = 0; i < tamano; i++) {
        buffer_imagen[i] = (img[i] < params.umbral) ? 255 : 0;
    }
    
    // Extraer ROI
    int roi_y = alto - params.altura_roi;
    if (roi_y < 0) roi_y = 0;
    
    uint8_t* roi = buffer_imagen + roi_y * ancho;
    int roi_alto = alto - roi_y;
    
    // Calcular posicion
    posicion_linea = calcularCentroide(roi, ancho, roi_alto);
    linea_detectada = (posicion_linea != 0.0f);
    
    // Control simple
    if (linea_detectada) {
        float abs_pos = fabs(posicion_linea);
        
        if (abs_pos < params.zona_muerta) {
            moverAdelante();
        } else if (posicion_linea < -params.zona_muerta) {
            girarIzquierda();
        } else if (posicion_linea > params.zona_muerta) {
            girarDerecha();
        }
    } else {
        detenerMotores();
    }
}

// ==========================================
// TAREA DE MOTORES (CORE 0)
// ==========================================

void tareaMotores(void* param) {
    ComandoMotor_t cmd;
    Parametros_t nuevos_params;
    
    Serial.println("[CORE0] Tarea de motores iniciada");
    
    while (sistema_activo) {
        // Procesar comandos de motores
        if (xQueueReceive(cola_comandos_motor, &cmd, pdMS_TO_TICKS(50)) == pdPASS) {
            if (!modo_automatico || cmd.comando == "stop") {
                ejecutarComando(cmd.comando, cmd.velocidad);
            }
        }
        
        // Actualizar parametros si hay nuevos
        if (xQueueReceive(cola_parametros, &nuevos_params, 0) == pdPASS) {
            if (xSemaphoreTake(mutex_estado, pdMS_TO_TICKS(100)) == pdTRUE) {
                params = nuevos_params;
                xSemaphoreGive(mutex_estado);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    vTaskDelete(NULL);
}

// ==========================================
// TAREA DE COMUNICACION (CORE 1)
// ==========================================

void tareaComWiFi(void* param) {
    TickType_t ultimo_frame = 0;
    const TickType_t intervalo_frame = pdMS_TO_TICKS(150); // 6.7 FPS
    
    Serial.println("[CORE1] Tarea de comunicacion iniciada");
    
    while (sistema_activo) {
        // Manejar servidor web
        server.handleClient();
        
        // Procesar vision periodicamente
        if (xTaskGetTickCount() - ultimo_frame > intervalo_frame) {
            ultimo_frame = xTaskGetTickCount();
            
            if (modo_automatico) {
                camera_fb_t* fb = esp_camera_fb_get();
                if (fb) {
                    procesarVision(fb);
                    esp_camera_fb_return(fb);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    vTaskDelete(NULL);
}

// ==========================================
// MANEJADORES HTTP SIMPLIFICADOS
// ==========================================

void handleRoot() {
    String html = "<!DOCTYPE html><html><head><title>Robot ESP32</title></head><body>";
    html += "<h2>Robot Seguidor de Linea</h2>";
    html += "<p>IP: " + WiFi.localIP().toString() + "</p>";
    html += "<p>Modo: " + String(modo_automatico ? "Auto" : "Manual") + "</p>";
    html += "<p>Estado: " + estado_actual + "</p>";
    if (linea_detectada) {
        html += "<p>Linea: Detectada (" + String(posicion_linea, 3) + ")</p>";
        html += "<p>Pixels: " + String(pixels_detectados) + "</p>";
    } else {
        html += "<p>Linea: No detectada</p>";
    }
    html += "<img src='/stream' style='max-width:100%'>";
    html += "<p>Control desde Python</p>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handleMove() {
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Sin datos");
        return;
    }
    
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, server.arg("plain"))) {
        server.send(400, "text/plain", "JSON invalido");
        return;
    }
    
    String dir = doc["direction"] | "stop";
    int vel = doc["speed"] | params.vel_base;
    
    ComandoMotor_t cmd = {dir, vel};
    if (xQueueSend(cola_comandos_motor, &cmd, 0) == pdPASS) {
        server.send(200, "text/plain", "OK");
    } else {
        server.send(500, "text/plain", "Cola llena");
    }
}

void handleAuto() {
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Sin datos");
        return;
    }
    
    DynamicJsonDocument doc(128);
    if (deserializeJson(doc, server.arg("plain"))) {
        server.send(400, "text/plain", "JSON invalido");
        return;
    }
    
    bool nuevo_modo = doc["auto"] | false;
    
    if (xSemaphoreTake(mutex_estado, pdMS_TO_TICKS(100)) == pdTRUE) {
        modo_automatico = nuevo_modo;
        xSemaphoreGive(mutex_estado);
        
        if (!modo_automatico) {
            ComandoMotor_t cmd = {"stop", 0};
            xQueueSend(cola_comandos_motor, &cmd, 0);
        }
        
        server.send(200, "text/plain", "OK");
    } else {
        server.send(500, "text/plain", "Error mutex");
    }
}

void handleStop() {
    if (xSemaphoreTake(mutex_estado, pdMS_TO_TICKS(100)) == pdTRUE) {
        modo_automatico = false;
        xSemaphoreGive(mutex_estado);
        
        ComandoMotor_t cmd = {"stop", 0};
        xQueueSend(cola_comandos_motor, &cmd, 0);
        
        server.send(200, "text/plain", "Detenido");
    } else {
        server.send(500, "text/plain", "Error");
    }
}

void handleParams() {
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Sin datos");
        return;
    }
    
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, server.arg("plain"))) {
        server.send(400, "text/plain", "JSON invalido");
        return;
    }
    
    Parametros_t nuevos = params; // Copia actual
    
    if (doc.containsKey("umbral")) nuevos.umbral = doc["umbral"];
    if (doc.containsKey("altura_roi")) nuevos.altura_roi = doc["altura_roi"];
    if (doc.containsKey("zona_muerta")) nuevos.zona_muerta = doc["zona_muerta"];
    if (doc.containsKey("velocidad_base")) nuevos.vel_base = doc["velocidad_base"];
    if (doc.containsKey("velocidad_giro")) nuevos.vel_giro = doc["velocidad_giro"];
    
    if (xQueueSend(cola_parametros, &nuevos, 0) == pdPASS) {
        server.send(200, "text/plain", "Parametros actualizados");
    } else {
        server.send(500, "text/plain", "Error actualizando");
    }
}

void handleStream() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Error camara");
        return;
    }
    
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache");
    
    if (fb->format == PIXFORMAT_GRAYSCALE) {
        uint8_t* jpg_buf = NULL;
        size_t jpg_len = 0;
        
        if (frame2jpg(fb, 80, &jpg_buf, &jpg_len)) {
            server.send_P(200, "image/jpeg", (const char*)jpg_buf, jpg_len);
            if (jpg_buf) free(jpg_buf);
        } else {
            server.send(500, "text/plain", "Error JPEG");
        }
    } else {
        server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
    }
    
    esp_camera_fb_return(fb);
}

// ==========================================
// SETUP Y INICIALIZACION
// ==========================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n===========================================");
    Serial.println("ROBOT ESP32-CAM OPTIMIZADO POR CORES");
    Serial.println("Core 0: Motores | Core 1: WiFi+Vision");
    Serial.println("===========================================");
    
    // Configurar GPIO
    pinMode(MOTOR_A_IN3, OUTPUT);
    pinMode(MOTOR_A_IN4, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_C_IN3, OUTPUT);
    pinMode(MOTOR_C_IN4, OUTPUT);
    detenerMotores();
    
    // Crear primitivas de sincronizacion
    cola_comandos_motor = xQueueCreate(10, sizeof(ComandoMotor_t));
    cola_parametros = xQueueCreate(5, sizeof(Parametros_t));
    mutex_estado = xSemaphoreCreateMutex();
    
    if (!cola_comandos_motor || !cola_parametros || !mutex_estado) {
        Serial.println("Error creando primitivas de sincronizacion");
        ESP.restart();
    }
    
    // Configurar camara (configuracion minima)
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
    config.xclk_freq_hz = 10000000; // Reducido para menor carga
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.grab_mode = CAMERA_GRAB_LATEST;
    
    // Configuracion conservadora
    if (psramFound()) {
        config.frame_size = FRAMESIZE_QVGA;    // 320x240
        config.jpeg_quality = 15;              // Calidad media
        config.fb_count = 1;                   // Solo 1 buffer
    } else {
        config.frame_size = FRAMESIZE_QQVGA;   // 160x120
        config.jpeg_quality = 20;
        config.fb_count = 1;
    }
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error camara: 0x%x\n", err);
        ESP.restart();
    }
    
    // Conectar WiFi
    Serial.println("Conectando WiFi...");
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    
    int intentos = 0;
    while (WiFi.status() != WL_CONNECTED && intentos < 20) {
        delay(500);
        Serial.print(".");
        intentos++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nError WiFi - Reiniciando");
        ESP.restart();
    }
    
    Serial.println("\nWiFi conectado");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    
    // Configurar servidor web
    server.on("/", HTTP_GET, handleRoot);
    server.on("/move", HTTP_POST, handleMove);
    server.on("/auto", HTTP_POST, handleAuto);
    server.on("/stop", HTTP_POST, handleStop);
    server.on("/params", HTTP_POST, handleParams);
    server.on("/stream", HTTP_GET, handleStream);
    server.enableCORS(true);
    server.begin();
    
    // Crear tareas optimizadas
    xTaskCreatePinnedToCore(
        tareaMotores,      // Funcion
        "TareaMotores",    // Nombre
        4096,              // Stack
        NULL,              // Parametros
        5,                 // Prioridad alta
        NULL,              // Handle
        0                  // Core 0 - GPIO y control
    );
    
    xTaskCreatePinnedToCore(
        tareaComWiFi,      // Funcion
        "TareaWiFi",       // Nombre
        8192,              // Stack mas grande
        NULL,              // Parametros
        4,                 // Prioridad media
        NULL,              // Handle
        1                  // Core 1 - WiFi y camara
    );
    
    Serial.println("===========================================");
    Serial.println("SISTEMA INICIADO CORRECTAMENTE");
    Serial.printf("Acceso web: http://%s/\n", WiFi.localIP().toString().c_str());
    Serial.println("Velocidades optimizadas para mejor conexion");
    Serial.println("===========================================");
    
    Serial.println("\nParametros optimizados:");
    Serial.printf("- Velocidad base: %d PWM\n", params.vel_base);
    Serial.printf("- Velocidad giro: %d PWM\n", params.vel_giro);
    Serial.printf("- Altura ROI: %d px\n", params.altura_roi);
    Serial.printf("- Zona muerta: %.3f\n", params.zona_muerta);
}

void loop() {
    // Loop principal vacio - todo manejado por tareas FreeRTOS
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Watchdog simple
    static int contador = 0;
    if (++contador >= 30) { // Cada 30 segundos
        contador = 0;
        Serial.printf("[WATCHDOG] Heap libre: %d bytes | Modo: %s\n", 
                     ESP.getFreeHeap(), modo_automatico ? "AUTO" : "MANUAL");
    }
}