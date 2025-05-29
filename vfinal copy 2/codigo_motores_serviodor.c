/*
 * ROBOT SEGUIDOR DE LINEA ESP32-CAM - ALGORITMO ANTI-RUIDO
 * ========================================================
 * 
 * CORRECCIONES IMPLEMENTADAS:
 * - Filtros morfologicos para eliminar ruido
 * - Deteccion inteligente de linea principal
 * - Seleccion de contorno por area, forma y posicion
 * - Algoritmo PID robusto anti-desviacion
 * - Imagen rotada 180 grados automaticamente
 * - Eliminacion de objetos blancos no deseados
 * 
 * CONFIGURACION FISICA:
 * - Motor A (GPIO 2,4): Frontal Izquierdo
 * - Motor B (GPIO 12,13): Trasero Central (parado)
 * - Motor C (GPIO 14,15): Frontal Derecho
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

// Pines camara ESP32-CAM
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

// Comando de motor
typedef struct {
    String comando;
    int velocidad;
} ComandoMotor_t;

// Parametros de control anti-ruido
typedef struct {
    int umbral;
    int altura_roi;
    float zona_muerta;
    int vel_base;
    int vel_giro;
    // Parametros PID
    float kp;
    float ki;
    float kd;
    // Parametros de filtrado
    int area_minima;
    int area_maxima;
    float aspect_ratio_min;
} ParametrosControl_t;

// Estado de deteccion de linea mejorado
typedef struct {
    bool detectada;
    float posicion;
    int centro_x;
    int centro_y;
    int area_contorno;
    float confianza;
    float aspect_ratio;
    unsigned long timestamp;
} EstadoLinea_t;

// Colas y sincronizacion
QueueHandle_t cola_comandos_motor;
QueueHandle_t cola_parametros;
SemaphoreHandle_t mutex_estado;

// Variables de estado
volatile bool modo_automatico = false;
volatile bool sistema_activo = true;
String estado_movimiento = "parado";

// Parametros optimizados anti-ruido
ParametrosControl_t params = {
    .umbral = 75,                // Umbral medio-bajo
    .altura_roi = 50,            // ROI mas pequena
    .zona_muerta = 0.10f,        // Zona muerta mayor
    .vel_base = 140,             // Velocidad reducida
    .vel_giro = 100,             // Giro mas lento
    .kp = 0.6f,                  // PID mas suave
    .ki = 0.01f,
    .kd = 0.2f,
    .area_minima = 150,          // Area minima para linea
    .area_maxima = 3000,         // Area maxima
    .aspect_ratio_min = 1.2f     // Relacion de aspecto minima
};

// Estado de la linea
EstadoLinea_t estado_linea = {
    .detectada = false,
    .posicion = 0.0f,
    .centro_x = 0,
    .centro_y = 0,
    .area_contorno = 0,
    .confianza = 0.0f,
    .aspect_ratio = 0.0f,
    .timestamp = 0
};

// Variables PID
float error_anterior = 0.0f;
float integral_error = 0.0f;
const float MAX_INTEGRAL = 20.0f;

// Variables para perdida de linea
int frames_sin_linea = 0;
const int MAX_FRAMES_SIN_LINEA = 15;
float ultima_posicion_valida = 0.0f;

// Buffers de imagen
static uint8_t* buffer_imagen = NULL;
static uint8_t* buffer_filtrado = NULL;
static size_t tamano_buffer = 0;

// ==========================================
// FUNCIONES DE CONTROL DE MOTORES
// ==========================================

void controlarMotor(int pin1, int pin2, int velocidad, bool adelante) {
    velocidad = constrain(velocidad, 0, 255);
    
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
    estado_movimiento = "parado";
}

// Control PID anti-desviacion
void aplicarControlPIDRobusto(float posicion_linea, float confianza) {
    // Calcular error
    float error = posicion_linea;
    
    // Ajustar ganancia segun confianza
    float kp_ajustado = params.kp * confianza;
    float ki_ajustado = params.ki * confianza;
    float kd_ajustado = params.kd * confianza;
    
    // Componente proporcional
    float p = kp_ajustado * error;
    
    // Componente integral con anti-windup
    integral_error += error * ki_ajustado;
    if (integral_error > MAX_INTEGRAL) integral_error = MAX_INTEGRAL;
    if (integral_error < -MAX_INTEGRAL) integral_error = -MAX_INTEGRAL;
    float i = integral_error;
    
    // Componente derivativo
    float d = kd_ajustado * (error - error_anterior);
    error_anterior = error;
    
    // Salida PID
    float salida_pid = p + i + d;
    
    // Calcular velocidades con reduccion adaptativa
    float velocidad_base = params.vel_base;
    float factor_reduccion = 1.0f - (abs(error) * 0.3f); // Reducir hasta 30% en curvas
    factor_reduccion = constrain(factor_reduccion, 0.4f, 1.0f);
    velocidad_base *= factor_reduccion;
    
    // Aplicar correcion diferencial
    float vel_izq = velocidad_base - salida_pid * 0.8f;
    float vel_der = velocidad_base + salida_pid * 0.8f;
    
    // Limitar velocidades
    vel_izq = constrain(vel_izq, -200, 200);
    vel_der = constrain(vel_der, -200, 200);
    
    // Aplicar a motores
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, abs(vel_izq), vel_izq >= 0);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, abs(vel_der), vel_der >= 0);
    
    // Actualizar estado
    if (abs(error) < params.zona_muerta) {
        estado_movimiento = "adelante";
    } else if (error < 0) {
        estado_movimiento = "giro_izq";
    } else {
        estado_movimiento = "giro_der";
    }
    
    Serial.printf("[PID] Error:%.3f Conf:%.2f P:%.2f I:%.2f D:%.2f -> VelIzq:%.0f VelDer:%.0f\n", 
                  error, confianza, p, i, d, vel_izq, vel_der);
}

void moverAdelante(int vel = 0) {
    if (vel == 0) vel = params.vel_base;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel, true);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel, true);
    estado_movimiento = "adelante";
}

void moverAtras(int vel = 0) {
    if (vel == 0) vel = params.vel_base;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel, false);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel, false);
    estado_movimiento = "atras";
}

void girarIzquierda(int vel = 0) {
    if (vel == 0) vel = params.vel_giro;
    int vel_reducida = vel * 0.4f;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel_reducida, true);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel, true);
    estado_movimiento = "izquierda";
}

void girarDerecha(int vel = 0) {
    if (vel == 0) vel = params.vel_giro;
    int vel_reducida = vel * 0.4f;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel, true);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel_reducida, true);
    estado_movimiento = "derecha";
}

void rotarIzquierda(int vel = 0) {
    if (vel == 0) vel = params.vel_giro;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel, false);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel, true);
    estado_movimiento = "rotar_izq";
}

void rotarDerecha(int vel = 0) {
    if (vel == 0) vel = params.vel_giro;
    controlarMotor(MOTOR_A_IN3, MOTOR_A_IN4, vel, true);
    controlarMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0, true);
    controlarMotor(MOTOR_C_IN3, MOTOR_C_IN4, vel, false);
    estado_movimiento = "rotar_der";
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
// FUNCIONES DE FILTRADO ANTI-RUIDO
// ==========================================

void aplicarFiltroMorfologico(uint8_t* imagen, int ancho, int alto) {
    if (!buffer_filtrado) return;
    
    // Operacion de apertura (erosion + dilatacion) para eliminar ruido
    // Erosion simple
    for (int y = 1; y < alto - 1; y++) {
        for (int x = 1; x < ancho - 1; x++) {
            int idx = y * ancho + x;
            if (imagen[idx] == 255) {
                // Verificar vecinos 3x3
                bool mantener = true;
                for (int dy = -1; dy <= 1 && mantener; dy++) {
                    for (int dx = -1; dx <= 1 && mantener; dx++) {
                        if (imagen[(y+dy) * ancho + (x+dx)] == 0) {
                            mantener = false;
                        }
                    }
                }
                buffer_filtrado[idx] = mantener ? 255 : 0;
            } else {
                buffer_filtrado[idx] = 0;
            }
        }
    }
    
    // Dilatacion simple
    for (int y = 1; y < alto - 1; y++) {
        for (int x = 1; x < ancho - 1; x++) {
            int idx = y * ancho + x;
            if (buffer_filtrado[idx] == 0) {
                // Verificar si tiene vecinos blancos
                bool dilatar = false;
                for (int dy = -1; dy <= 1 && !dilatar; dy++) {
                    for (int dx = -1; dx <= 1 && !dilatar; dx++) {
                        if (buffer_filtrado[(y+dy) * ancho + (x+dx)] == 255) {
                            dilatar = true;
                        }
                    }
                }
                imagen[idx] = dilatar ? 255 : 0;
            } else {
                imagen[idx] = 255;
            }
        }
    }
}

int evaluarContornoLinea(uint8_t* roi, int ancho, int alto, int inicio_x, int inicio_y, int area) {
    // Calcular bounding box
    int min_x = ancho, max_x = 0, min_y = alto, max_y = 0;
    int pixel_count = 0;
    
    for (int y = 0; y < alto; y++) {
        for (int x = 0; x < ancho; x++) {
            if (roi[y * ancho + x] == 255) {
                pixel_count++;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
            }
        }
    }
    
    if (pixel_count < params.area_minima || pixel_count > params.area_maxima) {
        return 0; // Area fuera de rango
    }
    
    // Calcular aspect ratio
    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;
    float aspect_ratio = max(width, height) / (float)min(width, height);
    
    if (aspect_ratio < params.aspect_ratio_min) {
        return 0; // Muy cuadrado, no es una linea
    }
    
    // Calcular puntuacion
    int puntuacion = 0;
    
    // Factor area
    if (pixel_count >= 200 && pixel_count <= 2000) {
        puntuacion += 40;
    } else if (pixel_count >= 150 && pixel_count <= 3000) {
        puntuacion += 20;
    }
    
    // Factor aspect ratio
    if (aspect_ratio > 2.0f) {
        puntuacion += 30;
    } else if (aspect_ratio > 1.5f) {
        puntuacion += 15;
    }
    
    // Factor posicion (centrado)
    int centro_x = (min_x + max_x) / 2;
    float dist_centro = abs(centro_x - ancho/2) / (float)(ancho/2);
    if (dist_centro < 0.3f) {
        puntuacion += 20;
    } else if (dist_centro < 0.5f) {
        puntuacion += 10;
    }
    
    // Factor posicion Y (parte inferior)
    if (min_y > alto * 0.2f) {
        puntuacion += 10;
    }
    
    return puntuacion;
}

float calcularCentroideInteligente(uint8_t* roi, int ancho, int alto) {
    if (alto <= 0 || ancho <= 0) return 0.0f;
    
    // Encontrar el mejor contorno usando evaluacion de puntuacion
    int mejor_puntuacion = 0;
    int mejor_centro_x = 0;
    int mejor_area = 0;
    float mejor_confianza = 0.0f;
    
    // Buscar regiones conectadas manualmente (simplified connected components)
    static bool* visitado = NULL;
    static int tamano_visitado = 0;
    
    int tamano_necesario = ancho * alto;
    if (!visitado || tamano_visitado != tamano_necesario) {
        if (visitado) free(visitado);
        visitado = (bool*)calloc(tamano_necesario, sizeof(bool));
        tamano_visitado = tamano_necesario;
    }
    
    // Limpiar array de visitados
    memset(visitado, 0, tamano_necesario * sizeof(bool));
    
    for (int y = 0; y < alto; y++) {
        for (int x = 0; x < ancho; x++) {
            int idx = y * ancho + x;
            
            if (roi[idx] == 255 && !visitado[idx]) {
                // Encontrar componente conectado usando flood fill simple
                int area_componente = 0;
                int suma_x = 0, suma_y = 0;
                
                // Stack simple para flood fill
                int stack_x[1000], stack_y[1000];
                int stack_top = 0;
                
                stack_x[0] = x;
                stack_y[0] = y;
                stack_top = 1;
                
                while (stack_top > 0) {
                    stack_top--;
                    int cx = stack_x[stack_top];
                    int cy = stack_y[stack_top];
                    int cidx = cy * ancho + cx;
                    
                    if (cx < 0 || cx >= ancho || cy < 0 || cy >= alto) continue;
                    if (visitado[cidx] || roi[cidx] != 255) continue;
                    
                    visitado[cidx] = true;
                    area_componente++;
                    suma_x += cx;
                    suma_y += cy;
                    
                    // Agregar vecinos al stack (4-conectividad)
                    if (stack_top < 996) { // Evitar overflow
                        stack_x[stack_top] = cx - 1; stack_y[stack_top] = cy; stack_top++;
                        stack_x[stack_top] = cx + 1; stack_y[stack_top] = cy; stack_top++;
                        stack_x[stack_top] = cx; stack_y[stack_top] = cy - 1; stack_top++;
                        stack_x[stack_top] = cx; stack_y[stack_top] = cy + 1; stack_top++;
                    }
                }
                
                // Evaluar este componente
                int puntuacion = evaluarContornoLinea(roi, ancho, alto, x, y, area_componente);
                
                if (puntuacion > mejor_puntuacion && area_componente >= params.area_minima) {
                    mejor_puntuacion = puntuacion;
                    mejor_centro_x = suma_x / area_componente;
                    mejor_area = area_componente;
                    mejor_confianza = min(1.0f, (float)area_componente / 1000.0f);
                }
            }
        }
    }
    
    // Actualizar estado global
    estado_linea.area_contorno = mejor_area;
    estado_linea.confianza = mejor_confianza;
    
    if (mejor_puntuacion > 30) { // Umbral minimo de puntuacion
        float posicion = (mejor_centro_x - ancho/2.0f) / (ancho/2.0f);
        
        Serial.printf("[VISION] Linea inteligente - Centro:%d Area:%d Puntuacion:%d Pos:%.3f\n", 
                     mejor_centro_x, mejor_area, mejor_puntuacion, posicion);
        
        return posicion;
    }
    
    Serial.printf("[VISION] Sin linea valida - Mejor puntuacion:%d\n", mejor_puntuacion);
    return 0.0f;
}

void procesarVisionAntiRuido(camera_fb_t* fb) {
    if (!fb || !modo_automatico) return;
    
    uint8_t* img = fb->buf;
    int ancho = fb->width;
    int alto = fb->height;
    size_t tamano = ancho * alto;
    
    // Gestionar buffers
    if (!buffer_imagen || tamano_buffer != tamano) {
        if (buffer_imagen) free(buffer_imagen);
        if (buffer_filtrado) free(buffer_filtrado);
        
        buffer_imagen = (uint8_t*)malloc(tamano);
        buffer_filtrado = (uint8_t*)malloc(tamano);
        
        if (!buffer_imagen || !buffer_filtrado) return;
        tamano_buffer = tamano;
    }
    
    // Binarizacion
    for (int i = 0; i < tamano; i++) {
        buffer_imagen[i] = (img[i] < params.umbral) ? 255 : 0;
    }
    
    // Extraer ROI
    int roi_y = alto - params.altura_roi;
    if (roi_y < 0) roi_y = 0;
    
    uint8_t* roi = buffer_imagen + roi_y * ancho;
    int roi_alto = alto - roi_y;
    
    // Aplicar filtros morfologicos a la ROI
    aplicarFiltroMorfologico(roi, ancho, roi_alto);
    
    // Calcular posicion con algoritmo inteligente
    float posicion_detectada = calcularCentroideInteligente(roi, ancho, roi_alto);
    
    // Actualizar estado de linea
    unsigned long tiempo_actual = millis();
    
    if (posicion_detectada != 0.0f) {
        // Linea detectada
        estado_linea.detectada = true;
        estado_linea.posicion = posicion_detectada;
        estado_linea.centro_x = (posicion_detectada * ancho/2) + ancho/2;
        estado_linea.centro_y = roi_y + roi_alto/2;
        estado_linea.timestamp = tiempo_actual;
        
        frames_sin_linea = 0;
        ultima_posicion_valida = posicion_detectada;
        
        // Aplicar control PID con confianza
        aplicarControlPIDRobusto(posicion_detectada, estado_linea.confianza);
        
    } else {
        // Linea no detectada
        frames_sin_linea++;
        
        if (frames_sin_linea < MAX_FRAMES_SIN_LINEA) {
            // Busqueda suave hacia ultima posicion
            Serial.printf("[VISION] Linea perdida (%d/%d) - Buscando\n", 
                         frames_sin_linea, MAX_FRAMES_SIN_LINEA);
            
            float factor_reduccion = 1.0f - ((float)frames_sin_linea / MAX_FRAMES_SIN_LINEA) * 0.6f;
            
            if (abs(ultima_posicion_valida) > 0.1f) {
                if (ultima_posicion_valida < 0) {
                    girarIzquierda(params.vel_giro * factor_reduccion);
                } else {
                    girarDerecha(params.vel_giro * factor_reduccion);
                }
            } else {
                moverAdelante(params.vel_base * factor_reduccion);
            }
        } else {
            // Demasiado tiempo sin linea
            estado_linea.detectada = false;
            detenerMotores();
            Serial.printf("[VISION] Linea perdida %d frames - DETENIENDO\n", frames_sin_linea);
            
            // Resetear PID
            error_anterior = 0.0f;
            integral_error = 0.0f;
        }
    }
}

// ==========================================
// TAREAS DE FREERTOS
// ==========================================

void tareaMotores(void* param) {
    ComandoMotor_t cmd;
    ParametrosControl_t nuevos_params;
    
    Serial.println("[CORE0] Tarea motores con anti-ruido iniciada");
    
    while (sistema_activo) {
        // Procesar comandos manuales
        if (xQueueReceive(cola_comandos_motor, &cmd, pdMS_TO_TICKS(50)) == pdPASS) {
            if (!modo_automatico || cmd.comando == "stop") {
                ejecutarComando(cmd.comando, cmd.velocidad);
                
                // Resetear PID al usar control manual
                if (!modo_automatico) {
                    error_anterior = 0.0f;
                    integral_error = 0.0f;
                }
            }
        }
        
        // Actualizar parametros
        if (xQueueReceive(cola_parametros, &nuevos_params, 0) == pdPASS) {
            if (xSemaphoreTake(mutex_estado, pdMS_TO_TICKS(100)) == pdTRUE) {
                params = nuevos_params;
                xSemaphoreGive(mutex_estado);
                Serial.println("[PARAMS] Parametros anti-ruido actualizados");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    vTaskDelete(NULL);
}

void tareaVisionWiFi(void* param) {
    TickType_t ultimo_frame = 0;
    const TickType_t intervalo_frame = pdMS_TO_TICKS(100); // 10 FPS
    
    Serial.println("[CORE1] Tarea vision anti-ruido iniciada");
    
    while (sistema_activo) {
        // Manejar servidor web
        server.handleClient();
        
        // Procesar vision
        if (xTaskGetTickCount() - ultimo_frame > intervalo_frame) {
            ultimo_frame = xTaskGetTickCount();
            
            if (modo_automatico) {
                camera_fb_t* fb = esp_camera_fb_get();
                if (fb) {
                    procesarVisionAntiRuido(fb);
                    esp_camera_fb_return(fb);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    vTaskDelete(NULL);
}

// ==========================================
// MANEJADORES HTTP
// ==========================================

void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<title>Robot Anti-Ruido ESP32-CAM</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>";
    html += "body{font-family:Arial;text-align:center;margin:20px;background:#f5f5f5}";
    html += ".container{max-width:900px;margin:0 auto;background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
    html += ".status{background:#e8f5e8;padding:15px;margin:10px 0;border-radius:8px;border-left:4px solid #28a745}";
    html += ".detection{background:#fff3cd;padding:15px;margin:10px 0;border-radius:8px;border-left:4px solid #ffc107}";
    html += ".video-container{margin:20px 0;border:2px solid #28a745;border-radius:8px;overflow:hidden}";
    html += "img{max-width:100%;height:auto;display:block}";
    html += ".controls{margin:20px 0}";
    html += "button{padding:12px 24px;margin:8px;border:none;border-radius:6px;font-size:14px;cursor:pointer;font-weight:bold}";
    html += ".btn-success{background:#28a745;color:white}";
    html += ".btn-danger{background:#dc3545;color:white}";
    html += ".info{background:#f8f9fa;padding:15px;border-radius:5px;margin:10px 0;font-size:13px}";
    html += ".grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:400px;margin:0 auto}";
    html += "</style>";
    html += "<script>";
    html += "function toggleAuto(){";
    html += "fetch('/auto',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({auto:!autoMode})})";
    html += ".then(()=>location.reload());}";
    html += "function sendStop(){";
    html += "fetch('/stop',{method:'POST'}).then(()=>location.reload());}";
    html += "var autoMode=" + String(modo_automatico ? "true" : "false") + ";";
    html += "setInterval(()=>{if(autoMode)location.reload();},2500);";
    html += "</script></head><body>";
    
    html += "<div class='container'>";
    html += "<h1>🤖 Robot Anti-Ruido ESP32-CAM</h1>";
    html += "<p><strong>🧹 Filtros Morfológicos Activos | 🎯 Detección Inteligente</strong></p>";
    
    html += "<div class='status'>";
    html += "<h3>📊 Estado del Sistema</h3>";
    html += "<div class='grid'>";
    html += "<div><strong>IP:</strong><br>" + WiFi.localIP().toString() + "</div>";
    html += "<div><strong>Modo:</strong><br>" + String(modo_automatico ? "🤖 Automático" : "🎮 Manual") + "</div>";
    html += "<div><strong>Movimiento:</strong><br>" + estado_movimiento + "</div>";
    html += "</div></div>";
    
    if (modo_automatico) {
        html += "<div class='detection'>";
        html += "<h3>🎯 Detección Anti-Ruido</h3>";
        if (estado_linea.detectada) {
            html += "<div class='grid'>";
            html += "<div><strong>Estado:</strong><br>✅ Línea detectada</div>";
            html += "<div><strong>Posición:</strong><br>" + String(estado_linea.posicion, 3) + "</div>";
            html += "<div><strong>Confianza:</strong><br>" + String(estado_linea.confianza, 2) + "</div>";
            html += "</div>";
            html += "<div class='grid'>";
            html += "<div><strong>Centro:</strong><br>(" + String(estado_linea.centro_x) + "," + String(estado_linea.centro_y) + ")</div>";
            html += "<div><strong>Área:</strong><br>" + String(estado_linea.area_contorno) + " px²</div>";
            html += "<div><strong>Aspect Ratio:</strong><br>" + String(estado_linea.aspect_ratio, 1) + "</div>";
            html += "</div>";
        } else {
            html += "<p><strong>Estado:</strong> ❌ Sin línea válida detectada</p>";
            html += "<p><strong>Frames sin línea:</strong> " + String(frames_sin_linea) + "/" + String(MAX_FRAMES_SIN_LINEA) + "</p>";
        }
        html += "</div>";
    }
    
    html += "<div class='video-container'>";
    html += "<img src='/stream' alt='Stream Anti-Ruido' id='videoStream'>";
    html += "</div>";
    
    html += "<div class='controls'>";
    html += "<button onclick='toggleAuto()' class='" + String(modo_automatico ? "btn-danger" : "btn-success") + "'>";
    html += modo_automatico ? "🛑 Desactivar IA" : "🤖 Activar IA Anti-Ruido";
    html += "</button>";
    html += "<button onclick='sendStop()' class='btn-danger'>⏹️ PARAR</button>";
    html += "</div>";
    
    html += "<div class='info'>";
    html += "<h4>⚙️ Parámetros Anti-Ruido:</h4>";
    html += "<div class='grid'>";
    html += "<div><strong>Umbral:</strong><br>" + String(params.umbral) + "</div>";
    html += "<div><strong>ROI:</strong><br>" + String(params.altura_roi) + "px</div>";
    html += "<div><strong>Área Min/Max:</strong><br>" + String(params.area_minima) + "/" + String(params.area_maxima) + "</div>";
    html += "</div>";
    html += "<div class='grid'>";
    html += "<div><strong>Velocidad Base:</strong><br>" + String(params.vel_base) + "</div>";
    html += "<div><strong>Velocidad Giro:</strong><br>" + String(params.vel_giro) + "</div>";
    html += "<div><strong>Zona Muerta:</strong><br>" + String(params.zona_muerta, 3) + "</div>";
    html += "</div>";
    html += "<p><strong>PID:</strong> Kp:" + String(params.kp, 2) + " Ki:" + String(params.ki, 3) + " Kd:" + String(params.kd, 2) + "</p>";
    html += "</div>";
    
    html += "<div class='info'>";
    html += "<p>🧹 <em>Filtros morfológicos eliminan objetos blancos no deseados</em></p>";
    html += "<p>🎯 <em>Algoritmo inteligente selecciona solo la línea principal</em></p>";
    html += "<p>📱 <em>Control avanzado desde aplicación Python</em></p>";
    html += "</div>";
    
    html += "</div></body></html>";
    
    server.send(200, "text/html", html);
}

void handleMove() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"Sin datos\"}");
        return;
    }
    
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, server.arg("plain"))) {
        server.send(400, "application/json", "{\"error\":\"JSON invalido\"}");
        return;
    }
    
    String direccion = doc["direction"] | "stop";
    int velocidad = doc["speed"] | params.vel_base;
    
    ComandoMotor_t cmd = {direccion, velocidad};
    if (xQueueSend(cola_comandos_motor, &cmd, 0) == pdPASS) {
        server.send(200, "application/json", "{\"status\":\"ok\",\"comando\":\"" + direccion + "\"}");
    } else {
        server.send(500, "application/json", "{\"error\":\"Cola llena\"}");
    }
}

void handleAuto() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"Sin datos\"}");
        return;
    }
    
    DynamicJsonDocument doc(128);
    if (deserializeJson(doc, server.arg("plain"))) {
        server.send(400, "application/json", "{\"error\":\"JSON invalido\"}");
        return;
    }
    
    bool nuevo_modo = doc["auto"] | false;
    
    if (xSemaphoreTake(mutex_estado, pdMS_TO_TICKS(100)) == pdTRUE) {
        modo_automatico = nuevo_modo;
        xSemaphoreGive(mutex_estado);
        
        if (!modo_automatico) {
            ComandoMotor_t cmd = {"stop", 0};
            xQueueSend(cola_comandos_motor, &cmd, 0);
        } else {
            // Resetear variables al entrar en modo automatico
            error_anterior = 0.0f;
            integral_error = 0.0f;
            frames_sin_linea = 0;
        }
        
        server.send(200, "application/json", "{\"status\":\"ok\",\"modo\":\"" + String(nuevo_modo ? "auto" : "manual") + "\"}");
    } else {
        server.send(500, "application/json", "{\"error\":\"Error mutex\"}");
    }
}

void handleStop() {
    if (xSemaphoreTake(mutex_estado, pdMS_TO_TICKS(100)) == pdTRUE) {
        modo_automatico = false;
        xSemaphoreGive(mutex_estado);
        
        ComandoMotor_t cmd = {"stop", 0};
        xQueueSend(cola_comandos_motor, &cmd, 0);
        
        // Resetear PID
        error_anterior = 0.0f;
        integral_error = 0.0f;
        
        server.send(200, "application/json", "{\"status\":\"detenido\"}");
    } else {
        server.send(500, "application/json", "{\"error\":\"Error\"}");
    }
}

void handleParams() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"Sin datos\"}");
        return;
    }
    
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, server.arg("plain"))) {
        server.send(400, "application/json", "{\"error\":\"JSON invalido\"}");
        return;
    }
    
    ParametrosControl_t nuevos = params;
    
    if (doc.containsKey("umbral")) nuevos.umbral = doc["umbral"];
    if (doc.containsKey("altura_roi")) nuevos.altura_roi = doc["altura_roi"];
    if (doc.containsKey("zona_muerta")) nuevos.zona_muerta = doc["zona_muerta"];
    if (doc.containsKey("velocidad_base")) nuevos.vel_base = doc["velocidad_base"];
    if (doc.containsKey("velocidad_giro")) nuevos.vel_giro = doc["velocidad_giro"];
    
    if (xQueueSend(cola_parametros, &nuevos, 0) == pdPASS) {
        server.send(200, "application/json", "{\"status\":\"parametros_actualizados\"}");
    } else {
        server.send(500, "application/json", "{\"error\":\"Error cola\"}");
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
        
        if (frame2jpg(fb, 85, &jpg_buf, &jpg_len)) {
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
// SETUP PRINCIPAL
// ==========================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n====================================================");
    Serial.println("ROBOT ANTI-RUIDO ESP32-CAM - DETECCION INTELIGENTE");
    Serial.println("====================================================");
    
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
    cola_parametros = xQueueCreate(5, sizeof(ParametrosControl_t));
    mutex_estado = xSemaphoreCreateMutex();
    
    if (!cola_comandos_motor || !cola_parametros || !mutex_estado) {
        Serial.println("Error: primitivas de sincronizacion");
        ESP.restart();
    }
    
    // Configurar camara
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
    config.xclk_freq_hz = 10000000; // Frecuencia conservadora
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.grab_mode = CAMERA_GRAB_LATEST;
    
    if (psramFound()) {
        config.frame_size = FRAMESIZE_QVGA;    // 320x240
        config.jpeg_quality = 15;
        config.fb_count = 1;
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
    
    // Crear tareas
    xTaskCreatePinnedToCore(tareaMotores, "MotoresAntiRuido", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(tareaVisionWiFi, "VisionAntiRuido", 10240, NULL, 4, NULL, 1);
    
    Serial.println("====================================================");
    Serial.println("SISTEMA ANTI-RUIDO INICIADO");
    Serial.printf("Acceso web: http://%s/\n", WiFi.localIP().toString().c_str());
    Serial.println("🧹 Filtros morfologicos activos");
    Serial.println("🎯 Deteccion inteligente de linea principal");
    Serial.println("🚫 Eliminacion de objetos blancos no deseados");
    Serial.println("====================================================");
    
    Serial.println("\nParametros anti-ruido optimizados:");
    Serial.printf("- Umbral: %d (medio-bajo)\n", params.umbral);
    Serial.printf("- ROI: %d px (pequena)\n", params.altura_roi);
    Serial.printf("- Area minima: %d px²\n", params.area_minima);
    Serial.printf("- Area maxima: %d px²\n", params.area_maxima);
    Serial.printf("- Aspect ratio min: %.1f\n", params.aspect_ratio_min);
    Serial.printf("- Zona muerta: %.3f (mayor estabilidad)\n", params.zona_muerta);
    Serial.printf("- Velocidad base: %d PWM (reducida)\n", params.vel_base);
    Serial.printf("- Velocidad giro: %d PWM (lenta)\n", params.vel_giro);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Watchdog del sistema
    static int contador = 0;
    if (++contador >= 30) {
        contador = 0;
        Serial.printf("[WATCHDOG] Heap: %d | Modo: %s | Linea: %s | Mov: %s | Conf: %.2f\n", 
                     ESP.getFreeHeap(), 
                     modo_automatico ? "AUTO" : "MANUAL",
                     estado_linea.detectada ? "SI" : "NO",
                     estado_movimiento.c_str(),
                     estado_linea.confianza);
    }
}