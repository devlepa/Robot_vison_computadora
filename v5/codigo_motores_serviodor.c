#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <semphr.h>

// **********************************************************
//                  CONFIGURACIÓN WIFI
// **********************************************************
const char* ssid = "Esteban";
const char* password = "chanchan";

// **********************************************************
//                  OBJETOS GLOBALES
// **********************************************************
WebServer server(80);
QueueHandle_t motorCommandQueue;
SemaphoreHandle_t xFrameMutex;

// Estructura para comandos de motor
typedef struct {
    float vx;
    float vy;
    float omega;
} MotorCommand_t;

// Buffers para imágenes JPEG
uint8_t* original_jpeg = NULL;
size_t original_jpeg_len = 0;
uint8_t* threshold_jpeg = NULL;
size_t threshold_jpeg_len = 0;
uint8_t* contour_jpeg = NULL;
size_t contour_jpeg_len = 0;

// **********************************************************
//                  DEFINICIÓN DE PINES
// **********************************************************
#define MOTOR_A_IN3 2
#define MOTOR_A_IN4 4
#define MOTOR_B_IN1 12
#define MOTOR_B_IN2 13
#define MOTOR_C_IN3 14
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
//                  VARIABLES DE ESTADO
// **********************************************************
bool autoMode = true;
float current_vx = 0.0;
float current_vy = 0.0;
float current_omega = 0.0;
unsigned long lastMotorCommandTime = 0;
const unsigned long MOTOR_UPDATE_INTERVAL_MS = 50;
const unsigned long FRAME_TIMEOUT_MS = 500;

// Variables para PID
float Kp = 1.0f;  // Ajustado
float Ki = 0.0f;  // Ajustado
float Kd = 0.15f; // Ajustado
float previous_error = 0;
float integral_error = 0;
const float MAX_INTEGRAL_ERROR = 50.0f;

// Buffer para imagen binaria
static uint8_t *binary_image_buffer = NULL;
static size_t current_binary_buffer_size = 0;

// **********************************************************
//                  FUNCIONES DE MOTOR
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

void moveFromAI(float vx, float vy, float omega) {
    float robot_radius = 0.1f;
    float v_wheel1 = vy + omega * robot_radius;
    float v_wheel2 = -0.5f * vy + 0.866f * vx + omega * robot_radius;
    float v_wheel3 = -0.5f * vy - 0.866f * vx + omega * robot_radius;

    float max_abs_v = max(fabs(v_wheel1), max(fabs(v_wheel2), fabs(v_wheel3)));
    if (max_abs_v > 1.0f) {
        v_wheel1 /= max_abs_v;
        v_wheel2 /= max_abs_v;
        v_wheel3 /= max_abs_v;
    }

    int pwm_wheel1 = abs(v_wheel1) * 255;
    int pwm_wheel2 = abs(v_wheel2) * 255;
    int pwm_wheel3 = abs(v_wheel3) * 255;

    pwm_wheel1 = min(pwm_wheel1, 255);
    pwm_wheel2 = min(pwm_wheel2, 255);
    pwm_wheel3 = min(pwm_wheel3, 255);

    setMotor(MOTOR_A_IN3, MOTOR_A_IN4, pwm_wheel1, v_wheel1 >= 0);
    setMotor(MOTOR_B_IN1, MOTOR_B_IN2, pwm_wheel2, v_wheel2 >= 0);
    setMotor(MOTOR_C_IN3, MOTOR_C_IN4, pwm_wheel3, v_wheel3 >= 0);

    Serial.printf("MotorCmd: Vx=%.2f, Vy=%.2f, Omega=%.2f -> P1=%d, P2=%d, P3=%d\n",
                  vx, vy, omega, pwm_wheel1, pwm_wheel2, pwm_wheel3);
}

// **********************************************************
//                  FUNCIONES DE VISIÓN
// **********************************************************
float calculateLinePosition(uint8_t *binary_roi, int width, int height, int start_row_offset = 0) {
    if (height <= 0 || width <= 0) return 0.0f;

    int line_pixels_x_sum = 0;
    int line_pixels_count = 0;
    int rows_to_analyze = min(15, height);

    for (int y = height - rows_to_analyze; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (binary_roi[y * width + x] > 0) {
                line_pixels_x_sum += x;
                line_pixels_count++;
            }
        }
    }

    if (line_pixels_count > (width * rows_to_analyze * 0.02f)) {
        float line_center_x = (float)line_pixels_x_sum / line_pixels_count;
        return (line_center_x - width / 2.0f) / (width / 2.0f);
    }
    return 0.0f;
}

void generateImageStreams(camera_fb_t *fb, uint8_t *binary_buf, float line_position) {
    // Liberar buffers anteriores si existen
    if (original_jpeg) free(original_jpeg);
    if (threshold_jpeg) free(threshold_jpeg);
    if (contour_jpeg) free(contour_jpeg);

    original_jpeg = NULL;
    threshold_jpeg = NULL;
    contour_jpeg = NULL;

    // 1. Imagen original (convertir a JPEG)
    if (fb->format != PIXFORMAT_JPEG) {
        frame2jpg(fb, 80, &original_jpeg, &original_jpeg_len);
    } else {
        original_jpeg = (uint8_t*)malloc(fb->len);
        if (original_jpeg) {
            memcpy(original_jpeg, fb->buf, fb->len);
            original_jpeg_len = fb->len;
        }
    }

    // 2. Imagen umbralizada
    camera_fb_t threshold_fb = {
        .width = fb->width,
        .height = fb->height,
        .format = PIXFORMAT_GRAYSCALE,
        .buf = binary_buf,
        .len = current_binary_buffer_size
    };
    frame2jpg(&threshold_fb, 80, &threshold_jpeg, &threshold_jpeg_len);

    // 3. Imagen de contornos (dibujamos sobre la original)
    uint8_t *contour_buf = (uint8_t*)malloc(fb->width * fb->height * 3);
    if (contour_buf) {
        // Convertir a RGB (para dibujar en color)
        for (int i = 0; i < fb->width * fb->height; i++) {
            uint8_t gray_val = fb->buf[i];
            contour_buf[i*3] = gray_val;
            contour_buf[i*3+1] = gray_val;
            contour_buf[i*3+2] = gray_val;
        }

        // Dibujar ROI y centroide si se detectó línea
        if (line_position != 0.0f) {
            int cx = (int)((line_position + 1.0f) * (fb->width / 2.0f));
            int cy = fb->height - 20; // Parte inferior

            // Dibujar línea horizontal
            for (int x = cx-50; x <= cx+50; x++) {
                if (x >= 0 && x < fb->width) {
                    int idx = (cy * fb->width + x) * 3;
                    contour_buf[idx] = 255;   // Rojo
                    contour_buf[idx+1] = 0;
                    contour_buf[idx+2] = 0;
                }
            }

            // Dibujar línea vertical
            for (int y = cy-50; y <= cy+50; y++) {
                if (y >= 0 && y < fb->height) {
                    int idx = (y * fb->width + cx) * 3;
                    contour_buf[idx] = 255;   // Rojo
                    contour_buf[idx+1] = 0;
                    contour_buf[idx+2] = 0;
                }
            }

            // Dibujar círculo en el centroide
            for (int dy = -10; dy <= 10; dy++) {
                for (int dx = -10; dx <= 10; dx++) {
                    if (dx*dx + dy*dy <= 100) {
                        int x = cx + dx;
                        int y = cy + dy;
                        if (x >= 0 && x < fb->width && y >= 0 && y < fb->height) {
                            int idx = (y * fb->width + x) * 3;
                            contour_buf[idx] = 0;     // Azul
                            contour_buf[idx+1] = 0;
                            contour_buf[idx+2] = 255;
                        }
                    }
                }
            }
        }

        camera_fb_t contour_fb = {
            .width = fb->width,
            .height = fb->height,
            .format = PIXFORMAT_RGB888,
            .buf = contour_buf,
            .len = fb->width * fb->height * 3
        };
        frame2jpg(&contour_fb, 80, &contour_jpeg, &contour_jpeg_len);
        free(contour_buf);
    }
}

void processAndDecide(camera_fb_t *fb) {
    if (!autoMode) return;

    uint8_t *gray_image = fb->buf;
    int img_width = fb->width;
    int img_height = fb->height;
    size_t new_buffer_size = img_width * img_height;

    // Asignar memoria para imagen binaria
    if (binary_image_buffer == NULL || new_buffer_size != current_binary_buffer_size) {
        if (binary_image_buffer) free(binary_image_buffer);
        binary_image_buffer = (uint8_t *)heap_caps_malloc(new_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!binary_image_buffer) {
            Serial.println("Error: No memory for binary image");
            return;
        }
        current_binary_buffer_size = new_buffer_size;
    }

    // Binarización
    uint8_t threshold_val = 70;  // Ajustado
    for (int i = 0; i < new_buffer_size; i++) {
        binary_image_buffer[i] = (gray_image[i] < threshold_val) ? 255 : 0;
    }

    // Definir ROI
    int roi_height = 80;  // Ajustado
    int roi_start_y = img_height - roi_height;
    if (roi_start_y < 0) roi_start_y = 0;
    uint8_t *roi_main_ptr = binary_image_buffer + roi_start_y * img_width;

    // Calcular posición de la línea
    float line_position = calculateLinePosition(roi_main_ptr, img_width, roi_height);

    // Generar streams de imágenes
    if (xSemaphoreTake(xFrameMutex, portMAX_DELAY)) {
        generateImageStreams(fb, binary_image_buffer, line_position);
        xSemaphoreGive(xFrameMutex);
    }

    // Lógica de control PID
    float error = line_position;
    float p_term = Kp * error;

    integral_error += error;
    if (integral_error > MAX_INTEGRAL_ERROR) integral_error = MAX_INTEGRAL_ERROR;
    if (integral_error < -MAX_INTEGRAL_ERROR) integral_error = -MAX_INTEGRAL_ERROR;
    float i_term = Ki * integral_error;

    float d_term = Kd * (error - previous_error);
    previous_error = error;

    float omega = -(p_term + i_term + d_term);
    float vx = 0.5f;
    float vy = 0.0f;

    float abs_omega = fabs(omega);
    float max_omega_expected = 0.8f;
    if (abs_omega > max_omega_expected) abs_omega = max_omega_expected;
    vx = vx * (1.0f - (abs_omega / max_omega_expected * 0.5f));  // Ajustado
    vx = max(0.1f, vx);

    omega = fmax(-1.0f, fmin(1.0f, omega));

    // Enviar comando a motores
    MotorCommand_t cmd = {vx, vy, omega};
    if (xQueueSend(motorCommandQueue, &cmd, 0) != pdPASS) {
        Serial.println("Advertencia: Cola de motores llena");
    }

    Serial.printf("Line: %.2f | P:%.2f I:%.2f D:%.2f | Vx:%.2f Omega:%.2f\n",
                  line_position, p_term, i_term, d_term, vx, omega);
}

// **********************************************************
//                  TAREAS FREERTOS
// **********************************************************
void visionProcessingTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (autoMode) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("Cámara: Error al obtener frame");
                continue;
            }

            processAndDecide(fb);
            esp_camera_fb_return(fb);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void motorTask(void *pvParameters) {
    MotorCommand_t received_cmd;

    while (true) {
        if (xQueueReceive(motorCommandQueue, &received_cmd, pdMS_TO_TICKS(MOTOR_UPDATE_INTERVAL_MS)) == pdPASS) {
            current_vx = received_cmd.vx;
            current_vy = received_cmd.vy;
            current_omega = received_cmd.omega;
            lastMotorCommandTime = millis();
        }

        if (autoMode) {
            moveFromAI(current_vx, current_vy, current_omega);
        } else if (millis() - lastMotorCommandTime > FRAME_TIMEOUT_MS) {
            stopAllMotors();
            current_vx = current_vy = current_omega = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// **********************************************************
//                  MANEJADORES HTTP
// **********************************************************
void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<title>Robot Omnidireccional (ESP32)</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>";
    html += "body { font-family: Arial; text-align: center; margin: 20px; }";
    html += ".container { display: flex; flex-wrap: wrap; justify-content: center; }";
    html += ".stream { margin: 10px; border: 1px solid #ccc; padding: 5px; }";
    html += "img { max-width: 100%; height: auto; }";
    html += "button { padding: 15px 30px; margin: 10px; font-size: 16px; }";
    html += "</style></head><body>";
    html += "<h1>Robot Omnidireccional con Visión Interna</h1>";
    html += "<div class='container'>";
    html += "<div class='stream'><h3>Original</h3><img src='/stream_original'></div>";
    html += "<div class='stream'><h3>Umbralizada</h3><img src='/stream_threshold'></div>";
    html += "<div class='stream'><h3>Contornos</h3><img src='/stream_contour'></div>";
    html += "</div>";
    html += "<button onclick=\"fetch('/auto', {method:'POST',body:JSON.stringify({auto:true})})\">ACTIVAR IA</button>";
    html += "<button onclick=\"fetch('/auto', {method:'POST',body:JSON.stringify({auto:false})})\">DESACTIVAR IA</button>";
    html += "<button onclick=\"fetch('/stop', {method:'POST'})\">DETENER</button>";
    html += "<script>";
    html += "setInterval(() => {";
    html += "  document.querySelectorAll('img').forEach(img => {";
    html += "    img.src = img.src.split('?')[0] + '?' + Date.now();";
    html += "  });";
    html += "}, 200);";
    html += "</script>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleAutoMode() {
    if (server.hasArg("plain")) {
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, server.arg("plain"));
        autoMode = doc["auto"];
        Serial.println("Modo automático: " + String(autoMode ? "ON" : "OFF"));
        stopAllMotors();
        current_vx = current_vy = current_omega = 0;
        if(autoMode){
            previous_error = 0;
            integral_error = 0;
        }
    }
    server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleStop() {
    autoMode = false;
    stopAllMotors();
    server.send(200, "application/json", "{\"status\":\"stopped\"}");
}

void handleStream(const char* stream_name, uint8_t** buffer, size_t* len) {
    if (xSemaphoreTake(xFrameMutex, pdMS_TO_TICKS(1000))) {
        if (*buffer && *len > 0) {
            server.send_P(200, "image/jpeg", (const char*)*buffer, *len);
        } else {
            server.send(500, "text/plain", "No frame available");
        }
        xSemaphoreGive(xFrameMutex);
    } else {
        server.send(500, "text/plain", "Frame timeout");
    }
}

void handleStreamOriginal() {
    handleStream("original", &original_jpeg, &original_jpeg_len);
}

void handleStreamThreshold() {
    handleStream("threshold", &threshold_jpeg, &threshold_jpeg_len);
}

void handleStreamContour() {
    handleStream("contour", &contour_jpeg, &contour_jpeg_len);
}

// **********************************************************
//                  SETUP Y LOOP PRINCIPAL
// **********************************************************
void setup() {
    Serial.begin(115200);

    // Configurar pines de motores
    pinMode(MOTOR_A_IN3, OUTPUT);
    pinMode(MOTOR_A_IN4, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_C_IN3, OUTPUT);
    pinMode(MOTOR_C_IN4, OUTPUT);
    stopAllMotors();

    // Crear cola y semáforo
    motorCommandQueue = xQueueCreate(5, sizeof(MotorCommand_t));
    xFrameMutex = xSemaphoreCreateMutex();
    if (!motorCommandQueue || !xFrameMutex) {
        Serial.println("Error creando objetos RTOS");
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
    config.pixel_format = PIXFORMAT_GRAYSCALE;

    if(psramFound()){
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_QQVGA;
        config.jpeg_quality = 10;
        config.fb_count = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error inicializando cámara: 0x%x", err);
        ESP.restart();
    }

    // Conectar WiFi
    WiFi.begin(ssid, password);
    Serial.print("Conectando a WiFi");
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFallo conexión WiFi. Modo AP activado.");
        WiFi.softAP("Robot-ESP32", "12345678");
        Serial.print("IP AP: ");
        Serial.println(WiFi.softAPIP());
    } else {
        Serial.println("\nWiFi conectado!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    }

    // Configurar rutas del servidor
    server.on("/", HTTP_GET, handleRoot);
    server.on("/stop", HTTP_POST, handleStop);
    server.on("/auto", HTTP_POST, handleAutoMode);
    server.on("/stream_original", HTTP_GET, handleStreamOriginal);
    server.on("/stream_threshold", HTTP_GET, handleStreamThreshold);
    server.on("/stream_contour", HTTP_GET, handleStreamContour);
    server.enableCORS(true);

    server.begin();
    Serial.println("Servidor web iniciado");

    // Crear tareas
    xTaskCreatePinnedToCore(visionProcessingTask, "VisionTask", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, NULL, 4, NULL, 0);
}

void loop() {
    server.handleClient();
    delay(2);
}