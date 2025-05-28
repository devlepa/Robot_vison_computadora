/*
 * Robot Omnidireccional con ESP32-CAM - Visión en el borde y Control Multicore
 * Autor: Asistente IA
 * Descripción: Sistema de seguimiento de líneas con procesamiento de visión en la ESP32-CAM
 * Control de motores y visión en diferentes cores de FreeRTOS.
 * La ESP32 procesa la imagen y decide el movimiento.
 * El PC (Python) solo monitorea y cambia modos.
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
#include <esp_timer.h> // Para mediciones de tiempo precisas
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h> // Para comunicación entre tareas (cores)

// **********************************************************
//                  CONFIGURACIÓN WIFI
// **********************************************************
const char* ssid = "Esteban"; // <-- TU SSID
const char* password = "chanchan"; // <-- TU Contraseña

// **********************************************************
//                  OBJETOS GLOBALES
// **********************************************************
WebServer server(80);
QueueHandle_t motorCommandQueue; // Cola para comandos de motor (Vx, Vy, Omega)

// Estructura para los comandos de motor
typedef struct {
    float vx;
    float vy;
    float omega;
} MotorCommand_t;

// **********************************************************
//                  DEFINICIÓN DE PINES
// **********************************************************
// Pines para motores
#define MOTOR_A_IN3 2
#define MOTOR_A_IN4 4
#define MOTOR_B_IN1 12
#define MOTOR_B_IN2 13
#define MOTOR_C_IN3 14
#define MOTOR_C_IN4 15

// Pines para cámara ESP32-CAM
#define PWDN_GPIO_NUM       32
#define RESET_GPIO_NUM      -1 // No usado, puede ser -1
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
//                  VARIABLES DE ESTADO
// **********************************************************
bool autoMode = false; // Modo automático (IA)
float current_vx = 0.0;
float current_vy = 0.0;
float current_omega = 0.0;
unsigned long lastMotorCommandTime = 0;
const unsigned long MOTOR_UPDATE_INTERVAL_MS = 50; // Actualizar motores cada 50ms (20 veces/seg)

// **********************************************************
//                  FUNCIONES DE UTILIDAD DE MOTOR
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
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, 0, true);
    Serial.println("STOP");
}

// Cinemática inversa para robot omnidireccional de 3 ruedas
// Asumiendo Motor A a 0° (eje X), Motor B a 120°, Motor C a 240°
void moveFromAI(float vx, float vy, float omega) {
    // Radio del robot (distancia del centro a cada rueda), en metros
    // AJUSTA ESTE VALOR SEGÚN TU ROBOT REAL
    float robot_radius = 0.1f; // Ejemplo: 10 cm, usar 'f' para float

    // Velocidades de las ruedas
    float v_wheel1 = vy + omega * robot_radius; // Motor A (delantero/trasero)
    float v_wheel2 = -0.5f * vy + 0.866f * vx + omega * robot_radius; // Motor B (lateral-izq)
    float v_wheel3 = -0.5f * vy - 0.866f * vx + omega * robot_radius; // Motor C (lateral-der)

    // Encuentra la velocidad máxima para normalizar
    float max_abs_v = max(fabs(v_wheel1), max(fabs(v_wheel2), fabs(v_wheel3)));
    
    // Normalizar si alguna velocidad excede 1.0 (velocidad máxima lógica)
    if (max_abs_v > 1.0f) { // Usar 1.0f para asegurar tipo float
        v_wheel1 /= max_abs_v;
        v_wheel2 /= max_abs_v;
        v_wheel3 /= max_abs_v;
    }

    // Escalar a PWM (0-255)
    int pwm_wheel1 = abs(v_wheel1) * 255;
    int pwm_wheel2 = abs(v_wheel2) * 255;
    int pwm_wheel3 = abs(v_wheel3) * 255;

    // Asegurarse de que el PWM no exceda 255
    pwm_wheel1 = min(pwm_wheel1, 255);
    pwm_wheel2 = min(pwm_wheel2, 255);
    pwm_wheel3 = min(pwm_wheel3, 255);

    // Mover motores
    setMotor(MOTOR_A_IN3, MOTOR_A_IN4, pwm_wheel1, v_wheel1 >= 0);
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, pwm_wheel2, v_wheel2 >= 0);
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, pwm_wheel3, v_wheel3 >= 0);

    Serial.printf("MotorCmd: Vx=%.2f, Vy=%.2f, Omega=%.2f -> P1=%d, P2=%d, P3=%d\n", 
                   vx, vy, omega, pwm_wheel1, pwm_wheel2, pwm_wheel3);
}

// **********************************************************
//                  FUNCIONES DE PROCESAMIENTO DE IMAGEN
// **********************************************************

// Variables para PID (mover a una estructura si hay múltiples PID)
float Kp = 0.8f;   // Ganancia Proporcional
float Ki = 0.005f;  // Ganancia Integral
float Kd = 0.1f;   // Ganancia Derivativa
float previous_error = 0;
float integral_error = 0;
const float MAX_INTEGRAL_ERROR = 50.0f; // Límite para evitar wind-up

// Función para calcular la posición de la línea (centroide)
float calculateLinePosition(uint8_t *binary_roi, int width, int height, int start_row_offset = 0) {
    if (height <= 0 || width <= 0) return 0.0f; // Usar 0.0f

    int line_pixels_x_sum = 0;
    int line_pixels_count = 0;

    // Solo analiza las últimas filas (más cercanas al robot)
    int rows_to_analyze = min(15, height); // Analiza las últimas 15 filas
    for (int y = height - rows_to_analyze; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Un píxel blanco (255) representa la línea binarizada
            if (binary_roi[y * width + x] > 0) {
                line_pixels_x_sum += x;
                line_pixels_count++;
            }
        }
    }

    if (line_pixels_count > (width * rows_to_analyze * 0.02f)) { // Mínimo 2% de píxeles para ser considerada una línea
        float line_center_x = (float)line_pixels_x_sum / line_pixels_count;
        // Normalizar posición: 0 es el centro, -1 es el extremo izquierdo, 1 es el extremo derecho
        return (line_center_x - width / 2.0f) / (width / 2.0f); // Usar 2.0f
    }
    return 0.0f; // No se detectó una línea significativa
}

// Global para el buffer binarizado y su tamaño actual
static uint8_t *binary_image_buffer = NULL;
static size_t current_binary_buffer_size = 0; // Para almacenar el tamaño actual

// Función para el procesamiento de imagen y lógica de control
void processAndDecide(camera_fb_t *fb) {
    if (!autoMode) return;

    uint8_t *gray_image = fb->buf;
    int img_width = fb->width;
    int img_height = fb->height;
    size_t new_buffer_size = img_width * img_height;

    // Binarización (umbral fijo)
    uint8_t threshold_val = 80; // Experimenta con este valor (0-255). Valores más bajos para líneas más oscuras.
    
    // Reasignar si el buffer es NULL o el tamaño de la imagen ha cambiado
    if (binary_image_buffer == NULL || new_buffer_size != current_binary_buffer_size) {
        if (binary_image_buffer != NULL) {
            free(binary_image_buffer); // Liberar memoria antigua si existe
            binary_image_buffer = NULL;
        }
        // Asignar nueva memoria usando heap_caps_malloc para mejor rendimiento con DMA si es posible
        binary_image_buffer = (uint8_t *)heap_caps_malloc(new_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (binary_image_buffer == NULL) {
            Serial.println("Error: No se pudo asignar memoria para la imagen binaria.");
            current_binary_buffer_size = 0; // Resetear el tamaño si falla
            return;
        }
        current_binary_buffer_size = new_buffer_size; // Actualizar el tamaño actual del buffer
        Serial.printf("Buffer binario reasignado a %d bytes.\n", current_binary_buffer_size);
    }

    for (int i = 0; i < new_buffer_size; i++) { // Usar new_buffer_size para la iteración
        // Asumiendo línea negra sobre fondo claro
        binary_image_buffer[i] = (gray_image[i] < threshold_val) ? 255 : 0;
    }

    // Definir ROI (Región de Interés)
    int roi_height = 60; // Parte inferior de la imagen
    int roi_start_y = img_height - roi_height;
    if (roi_start_y < 0) roi_start_y = 0;

    // Obtener la ROI principal (parte inferior de la imagen)
    uint8_t *roi_main_ptr = binary_image_buffer + roi_start_y * img_width;
    
    // Calcular la posición de la línea en la ROI principal
    float line_position = calculateLinePosition(roi_main_ptr, img_width, roi_height);

    // Opcional: Calcular curvatura (ROI adelantada)
    int roi_ahead_height = 30; // Altura de la ROI adelantada
    int roi_ahead_start_y = roi_start_y - roi_ahead_height - 10; // Un poco más arriba que la ROI principal
    if (roi_ahead_start_y < 0) roi_ahead_start_y = 0;
    
    float line_position_ahead = line_position; // Por defecto es igual a la principal
    if (roi_ahead_start_y > 0) { // Solo si la ROI adelantada es válida
        uint8_t *roi_ahead_ptr = binary_image_buffer + roi_ahead_start_y * img_width;
        // Asegurarse de que el tamaño de la ROI adelantada sea válido
        int actual_roi_ahead_height = img_height - roi_ahead_start_y;
        if (actual_roi_ahead_height > roi_ahead_height) actual_roi_ahead_height = roi_ahead_height;
        
        line_position_ahead = calculateLinePosition(roi_ahead_ptr, img_width, actual_roi_ahead_height);
    }
    
    float curvature = line_position_ahead - line_position; // Diferencia para estimar curvatura

    // ******************************************************
    // LÓGICA DE CONTROL PID
    // ******************************************************
    float error = line_position; // El error es la desviación de la línea central

    // Componente Proporcional
    float p_term = Kp * error;

    // Componente Integral
    integral_error += error;
    // Limitar el integral para evitar el "wind-up"
    if (integral_error > MAX_INTEGRAL_ERROR) integral_error = MAX_INTEGRAL_ERROR;
    if (integral_error < -MAX_INTEGRAL_ERROR) integral_error = -MAX_INTEGRAL_ERROR;
    float i_term = Ki * integral_error;

    // Componente Derivativo
    float d_term = Kd * (error - previous_error);
    previous_error = error;

    // Calcular velocidad angular (omega)
    float omega = -(p_term + i_term + d_term);

    // Calcular velocidad lineal (vx)
    float vx = 0.5f; // Velocidad base hacia adelante (0.0 a 1.0)
    float vy = 0.0f; // Movimiento lateral (generalmente 0 para seguimiento de línea)

    // Reducir la velocidad lineal si se necesita un giro fuerte
    float abs_omega = fabs(omega);
    float max_omega_expected = 0.8f; // Máximo omega esperado (ajustar)
    if (abs_omega > max_omega_expected) {
        abs_omega = max_omega_expected; // Limitar para el cálculo de reducción de velocidad
    }
    vx = vx * (1.0f - (abs_omega / max_omega_expected * 0.7f)); // Reduce vx hasta un 70% si omega es máximo
    vx = max(0.1f, vx); // Asegurar una velocidad mínima (para no detenerse completamente en curvas)

    // Limitar omega a un rango sensato (ej. -1.0 a 1.0)
    omega = fmax(-1.0f, fmin(1.0f, omega)); // Usar fmax y fmin para floats

    // ******************************************************
    // ENVIAR COMANDO A LA TAREA DE MOTOR
    // ******************************************************
    MotorCommand_t cmd = {vx, vy, omega};
    if (xQueueSend(motorCommandQueue, &cmd, 0) != pdPASS) {
        // Serial.println("Advertencia: Cola de comandos de motor llena.");
    }
    
    // Debug info (siempre en serial, pero puedes hacer una ruta para que Python lo lea)
    Serial.printf("Line: %.2f | Curvature: %.2f | P:%.2f I:%.2f D:%.2f | Vx:%.2f Omega:%.2f\n", 
                  line_position, curvature, p_term, i_term, d_term, vx, omega);
}

// **********************************************************
//                  TAREAS DE FREERTOS (CORES)
// **********************************************************

// Tarea para el procesamiento de visión (Core 1)
// RENOMBRADA DE loopTask A visionProcessingTask
void visionProcessingTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Intenta 20 FPS (50ms por frame)
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Sincronizar tarea

        if (autoMode) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("Cámara: Error al obtener frame buffer");
                continue;
            }
            
            // Procesar la imagen y decidir el movimiento
            processAndDecide(fb);
            
            esp_camera_fb_return(fb); // Devolver el frame buffer
        } else {
            // Si no está en modo automático, simplemente libera el buffer si hay alguno
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                esp_camera_fb_return(fb);
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Pequeña pausa si no hay actividad
        }
    }
}

// Tarea para el control de motores (Core 0)
void motorTask(void *pvParameters) {
    MotorCommand_t received_cmd;

    while (true) {
        // Espera un nuevo comando o un timeout para asegurar actualizaciones regulares
        if (xQueueReceive(motorCommandQueue, &received_cmd, pdMS_TO_TICKS(MOTOR_UPDATE_INTERVAL_MS)) == pdPASS) {
            // Se recibió un nuevo comando, actualiza las velocidades actuales
            current_vx = received_cmd.vx;
            current_vy = received_cmd.vy;
            current_omega = received_cmd.omega;
            lastMotorCommandTime = millis(); // Resetea el contador de tiempo de inactividad
        }

        // Si ha pasado mucho tiempo sin comandos o estamos en modo automático, aplicar el último comando
        // Esto previene que el robot se detenga si la cola de comandos está vacía momentáneamente.
        if (autoMode) {
            moveFromAI(current_vx, current_vy, current_omega);
        } else if (millis() - lastMotorCommandTime > MOTOR_UPDATE_INTERVAL_MS * 2) {
            // Si el modo automático está desactivado y no ha habido comandos recientes (manuales o de la cola)
            // Asegurarse de que el robot se detenga después de un breve período de inactividad
            stopAllMotors();
            current_vx = current_vy = current_omega = 0; // Resetear velocidades
        }
        
        // Pequeño delay para que otras tareas (como el servidor web) puedan ejecutarse
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}


// **********************************************************
//                  SETUP Y LOOP PRINCIPAL
// **********************************************************
void setup() {
    Serial.begin(115200);

    // Configurar pines de motores como salida
    pinMode(MOTOR_A_IN3, OUTPUT);
    pinMode(MOTOR_A_IN4, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_C_IN3, OUTPUT);
    pinMode(MOTOR_C_IN4, OUTPUT);
    stopAllMotors();

    // Crear la cola de comandos de motor
    motorCommandQueue = xQueueCreate(5, sizeof(MotorCommand_t)); // Cola de 5 elementos
    if (motorCommandQueue == NULL) {
        Serial.println("Error: No se pudo crear la cola de comandos de motor.");
        ESP.restart();
    }

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
    
    // ******************************************************
    // ***** OPTIMIZACIÓN CLAVE: CAPTURAR EN ESCALA DE GRISES PARA VISIÓN
    // ***** Y MENOR RESOLUCIÓN PARA STREAMING (SI NECESARIO)
    // ******************************************************
    // Para el procesamiento de visión LOCAL, la escala de grises es la más eficiente.
    // Si necesitas el stream de video, puedes usar JPEG con baja calidad.
    // PIXFORMAT_GRAYSCALE es ideal para el procesamiento en el chip.
    config.pixel_format = PIXFORMAT_GRAYSCALE; // ¡¡¡CAMBIADO A GRAYSCALE PARA EFICIENCIA!!!
    
    if(psramFound()){
      config.frame_size = FRAMESIZE_QVGA;     // 320x240 (ideal para procesamiento)
      config.jpeg_quality = 10;               // Muy bajo, no relevante para GRAYSCALE
      config.fb_count = 2;                    // 2 frame buffers para captura continua
    } else {
      config.frame_size = FRAMESIZE_QQVGA;    // 160x120 (si no hay PSRAM)
      config.jpeg_quality = 10;
      config.fb_count = 1;
    }
    // ******************************************************
    // ******************************************************

    // Inicializar cámara
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error inicializando cámara: 0x%x", err);
        ESP.restart(); // Reiniciar si la cámara falla
        return;
    }

    // Conectar a WiFi
    WiFi.begin(ssid, password);
    unsigned long wifiConnectStart = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - wifiConnectStart > 15000) { // Timeout de 15 segundos
            Serial.println("\nError: No se pudo conectar a WiFi. Reintentando...");
            ESP.restart();
        }
    }
    Serial.println("\nWiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    // Configurar rutas del servidor web (principalmente para control de modo y stream de depuración)
    server.on("/", HTTP_GET, handleRoot);
    server.on("/move", HTTP_POST, handleMove); // Mantener para control manual externo si se desea
    server.on("/stop", HTTP_POST, handleStop);
    server.on("/auto", HTTP_POST, handleAutoMode);
    server.on("/stream", HTTP_GET, handleStream); // Stream de video para monitoreo (no para procesamiento)
    server.enableCORS(true); // Permitir CORS para peticiones desde el PC

    server.begin();
    Serial.println("Servidor web iniciado");

    // Crear tareas de FreeRTOS en los cores específicos
    xTaskCreatePinnedToCore(
        visionProcessingTask, // <-- ¡NOMBRE DE LA TAREA CAMBIADO AQUÍ!
        "VisionTask",         // Nombre de la tarea
        8192,                 // Tamaño de la pila (más grande para procesamiento de imagen)
        NULL,                 // Parámetro para la tarea
        5,                    // Prioridad de la tarea (mayor es más importante)
        NULL,                 // Handle de la tarea (opcional)
        1                     // Core a asignar (Core 1 para Wi-Fi y Cámara)
    );

    xTaskCreatePinnedToCore(
        motorTask,      // Función a ejecutar
        "MotorTask",    // Nombre de la tarea
        4096,           // Tamaño de la pila
        NULL,           // Parámetro para la tarea
        4,              // Prioridad de la tarea (ligeramente menor que visión)
        NULL,           // Handle de la tarea (opcional)
        0               // Core a asignar (Core 0 para GPIO y la mayoría de las cosas)
    );
}

void loop() {
    // El loop principal de Arduino ahora solo maneja las peticiones del servidor web.
    // Las tareas de visión y motor se ejecutan en segundo plano en sus propios cores.
    server.handleClient();
    vTaskDelay(1); // Pequeño delay para ceder el control
}

// **********************************************************
//                  MANEJADORES DE RUTAS WEB
// **********************************************************

// Página web principal (HTML para control manual y stream)
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Robot Omnidireccional (ESP32)</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial; text-align: center; margin: 20px; }";
  html += ".controls { margin: 20px; }";
  html += "button { padding: 15px 30px; margin: 10px; font-size: 16px; }";
  html += "#video { max-width: 100%; height: auto; border: 1px solid #ccc; }";
  html += ".status { background: #e0f2f7; padding: 10px; margin: 10px; border-radius: 8px; }";
  html += "</style></head><body>";

  html += "<h1>Robot Omnidireccional con Visión Interna</h1>";
  html += "<div class='status'>";
  html += "<h3>Modo: <span id='modeStatus'>" + String(autoMode ? "Automático (IA)" : "Manual") + "</span></h3>";
  html += "<p>IP: " + WiFi.localIP().toString() + "</p>";
  html += "</div>";

  html += "<div><img id='video' src='/stream' alt='Stream de la Cámara del Robot'></div>";

  html += "<div class='controls'>";
  html += "<h3>Control de Modos</h3>";
  html += "<button onclick='toggleAuto()' id='autoBtn' style='background-color: #4CAF50;'>";
  html += autoMode ? "DESACTIVAR IA" : "ACTIVAR IA";
  html += "</button>";
  html += "<button onclick='sendStop()'>DETENER ROBOT</button>";
  html += "</div>";

  html += "<p><em>El control manual de movimiento ahora se gestiona directamente en el robot.</em></p>";
  html += "<p><em>Use el cliente Python para ver más detalles y controlar la depuración.</em></p>";

  html += "<script>";
  // Corrección: Asegurar que uno de los operandos sea un objeto String explícito
  html += String("var autoModeJs = ") + (autoMode ? "true" : "false") + ";";

  html += "function toggleAuto() {";
  html += "   autoModeJs = !autoModeJs;";
  html += "   var btn = document.getElementById('autoBtn');";
  html += "   var status = document.getElementById('modeStatus');";
  html += "   if (autoModeJs) {";
  html += "     btn.textContent = 'DESACTIVAR IA';";
  html += "     btn.style.backgroundColor = '#f44336';";
  html += "     status.textContent = 'Automático (IA)';";
  html += "   } else {";
  html += "     btn.textContent = 'ACTIVAR IA';";
  html += "     btn.style.backgroundColor = '#4CAF50';";
  html += "     status.textContent = 'Manual';";
  html += "   }";
  html += "   var xhr = new XMLHttpRequest();";
  html += "   xhr.open('POST', '/auto', true);";
  html += "   xhr.setRequestHeader('Content-Type', 'application/json');";
  html += "   xhr.send(JSON.stringify({auto: autoModeJs}));";
  html += "}";
  
  html += "function sendStop() {";
  html += "   var xhr = new XMLHttpRequest();";
  html += "   xhr.open('POST', '/stop', true);";
  html += "   xhr.send();";
  html += "}";

  html += "function updateVideo() {";
  html += "   document.getElementById('video').src = '/stream?' + new Date().getTime();";
  html += "}";

  html += "setInterval(updateVideo, 200);"; // Actualiza el stream cada 200ms (5 FPS para depuración visual)
  html += "</script>";

  html += "</body></html>";

  server.send(200, "text/html", html);
}

// Este handler se mantiene para compatibilidad, pero ya no debería usarse para el control principal
// La IA en la ESP32 controla el movimiento.
void handleMove() {
  if (server.hasArg("plain")) {
    if (autoMode) {
      server.send(403, "application/json", "{\"status\":\"error\", \"message\":\"Robot en modo automatico\"}");
      return;
    }
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    String direction = doc["direction"];
    int speed = doc["speed"] | 150; // default speed

    Serial.println("Manual Move: " + direction + ", Speed: " + String(speed));
    
    // Mapeo simple de comandos discretos a la cinemática inversa
    // Asumimos velocidad base para todos los movimientos manuales
    float manual_vx = 0, manual_vy = 0, manual_omega = 0;
    float base_speed = (float)speed / 255.0f; // Normalizar la velocidad de 0-255 a 0-1

    if (direction == "forward") {
        manual_vx = base_speed; manual_vy = 0; manual_omega = 0;
    } else if (direction == "backward") {
        manual_vx = -base_speed; manual_vy = 0; manual_omega = 0;
    } else if (direction == "left") { // Movimiento lateral
        manual_vx = 0; manual_vy = -base_speed; manual_omega = 0;
    } else if (direction == "right") { // Movimiento lateral
        manual_vx = 0; manual_vy = base_speed; manual_omega = 0;
    } else if (direction == "rotate_left") {
        manual_vx = 0; manual_vy = 0; manual_omega = -base_speed;
    } else if (direction == "rotate_right") {
        manual_vx = 0; manual_vy = 0; manual_omega = base_speed;
    } else if (direction == "stop") {
        manual_vx = 0; manual_vy = 0; manual_omega = 0;
    }

    current_vx = manual_vx;
    current_vy = manual_vy;
    current_omega = manual_omega;
    lastMotorCommandTime = millis();
    moveFromAI(current_vx, current_vy, current_omega); // Ejecutar el comando manual

  }
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleStop() {
    autoMode = false; // Detener IA también
    stopAllMotors();
    current_vx = current_vy = current_omega = 0; // Resetear velocidades
    server.send(200, "application/json", "{\"status\":\"stopped\"}");
}

void handleAutoMode() {
    if (server.hasArg("plain")) {
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, server.arg("plain"));
        autoMode = doc["auto"];
        Serial.println("Modo automático: " + String(autoMode ? "ON" : "OFF"));
        // Detener motores al cambiar de modo
        stopAllMotors();
        current_vx = current_vy = current_omega = 0; // Resetear velocidades
        // Reiniciar PID al entrar en modo automático
        if(autoMode){
            previous_error = 0;
            integral_error = 0;
        }
    }
    server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// Manejador para el stream de video (solo monitoreo)
void handleStream() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Stream: Error al obtener frame buffer");
        server.send(500, "text/plain", "Error capturando imagen");
        return;
    }

    server.sendHeader("Access-Control-Allow-Origin", "*");
    
    // Si la cámara captura en GRAYSCALE, convertir a JPEG para el stream
    if (fb->format == PIXFORMAT_GRAYSCALE) {
        uint8_t *jpeg_buf = NULL; // Puntero al buffer JPEG de salida
        size_t jpeg_len = 0;      // Longitud del buffer JPEG de salida

        // frame2jpg espera camera_fb_t*, quality, **output_buffer, *output_length
        bool jpeg_converted = frame2jpg(fb, 80, &jpeg_buf, &jpeg_len); // Calidad JPEG 80 (0-100)
        
        if (!jpeg_converted) {
            Serial.println("Error convirtiendo a JPEG para stream");
            esp_camera_fb_return(fb); // Liberar el frame buffer original
            server.send(500, "text/plain", "Error de conversión JPEG");
            return;
        }
        
        // Enviar el buffer JPEG convertido
        server.sendHeader("Cache-Control", "no-cache");
        server.send_P(200, "image/jpeg", (const char *)jpeg_buf, jpeg_len);
        
        // ¡¡¡Importante: Liberar el buffer que frame2jpg asignó dinámicamente!!!
        if (jpeg_buf) {
            free(jpeg_buf);
        }
        esp_camera_fb_return(fb); // Y liberar el frame buffer original de la cámara
        return; // Salir de la función después de enviar
    } else {
        // Si el formato ya es JPEG (u otro formato que el navegador entienda), enviamos el buffer original
        server.sendHeader("Cache-Control", "no-cache");
        server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb); // Liberar el frame buffer
    }
}