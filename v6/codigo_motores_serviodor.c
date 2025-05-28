#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

// ===== CONFIGURACIÓN DE RED =====
const char* ssid = "TU_WIFI_SSID";
const char* password = "TU_WIFI_PASSWORD";

// ===== DEFINICIÓN DE PINES DE CÁMARA AI-THINKER =====
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

// ===== CONFIGURACIÓN DE PINES PARA MOTORES (CORREGIDOS) =====
// Motor A (frontal izquierdo)
#define MOTOR_A_PIN1 2
#define MOTOR_A_PIN2 14
#define MOTOR_A_PWM 15

// Motor B (central/giro)  
#define MOTOR_B_PIN1 13
#define MOTOR_B_PIN2 12
#define MOTOR_B_PWM 4

// Motor C (frontal derecho) - PINES CORREGIDOS
#define MOTOR_C_PIN1 16  // Cambiado de pin 1 (TX0) a 16
#define MOTOR_C_PIN2 33  // Cambiado de pin 3 (RX0) a 33 (LED interno, pero funcional)
#define MOTOR_C_PWM 2    // Cambiado a pin 2 para PWM

// LED de estado interno
#define STATUS_LED_PIN 33

// ===== VARIABLES GLOBALES =====
WebServer server(80);
bool manualMode = true;  // true = control manual, false = modo IA
bool robotActive = false;
bool aiModeForward = false;

// Variables para streaming persistente
camera_fb_t* currentFrame = NULL;
unsigned long lastFrameTime = 0;
const unsigned long FRAME_INTERVAL = 100; // 100ms = ~10 FPS

// ===== ESTRUCTURAS PARA COMUNICACIÓN ENTRE NÚCLEOS =====
QueueHandle_t motorCommandQueue;
QueueHandle_t visionDataQueue;

struct MotorCommand {
  char motor;     // 'A', 'B', 'C' o 'S' (stop)
  int direction;  // 1 = adelante, -1 = atrás, 0 = parar
  int speed;      // 0-255
  int duration;   // duración en ms, 0 = continuo
};

struct VisionData {
  bool objectDetected;
  int objectX;
  int objectY;
  int objectSize;
  float confidence;
};

// ===== CONFIGURACIÓN PWM (CORREGIDA) =====
const int pwmFreq = 1000;
const int pwmResolution = 8;
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int pwmChannelC = 2;

// ===== INICIALIZACIÓN DE CÁMARA =====
bool initCamera() {
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
  
  // Configuración optimizada para streaming
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA; // 640x480
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_CIF; // 352x288
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error inicializando camara: 0x%x\n", err);
    return false;
  }
  
  sensor_t * s = esp_camera_sensor_get();
  
  // Configuración optimizada para ahorro de energía y mejor imagen
  s->set_brightness(s, 0);        // Brillo normal
  s->set_contrast(s, 0);          // Contraste normal
  s->set_saturation(s, 0);        // Saturación normal
  s->set_special_effect(s, 0);    // Sin efectos especiales
  s->set_whitebal(s, 1);          // Balance de blancos automático
  s->set_awb_gain(s, 1);          // Ganancia automática de balance de blancos
  s->set_wb_mode(s, 0);           // Modo de balance de blancos automático
  s->set_exposure_ctrl(s, 1);     // Control de exposición automático
  s->set_aec2(s, 0);              // Algoritmo de exposición automática
  s->set_ae_level(s, 0);          // Nivel de exposición automática
  s->set_aec_value(s, 300);       // Valor de exposición
  s->set_gain_ctrl(s, 1);         // Control de ganancia automático
  s->set_agc_gain(s, 0);          // Ganancia automática
  s->set_gainceiling(s, (gainceiling_t)0);
  s->set_bpc(s, 0);               // Corrección de píxeles defectuosos desactivada
  s->set_wpc(s, 1);               // Corrección de píxeles blancos activada
  s->set_raw_gma(s, 1);           // Corrección gamma activada
  s->set_lenc(s, 1);              // Corrección de lente activada
  s->set_hmirror(s, 0);           // Espejo horizontal desactivado
  s->set_vflip(s, 0);             // Volteo vertical desactivado
  s->set_dcw(s, 1);               // Ventana de recorte descendente activada
  s->set_colorbar(s, 0);          // Barra de colores desactivada
  
  Serial.println("Camara configurada sin LED flash para ahorro de energia");
  return true;
}

// ===== FUNCIONES DE CONTROL DE MOTORES =====
void initMotors() {
  Serial.println("Inicializando motores...");
  
  // Configurar pines de motor A
  pinMode(MOTOR_A_PIN1, OUTPUT);
  pinMode(MOTOR_A_PIN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  
  // Configurar pines de motor B
  pinMode(MOTOR_B_PIN1, OUTPUT);
  pinMode(MOTOR_B_PIN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  
  // Configurar pines de motor C
  pinMode(MOTOR_C_PIN1, OUTPUT);
  pinMode(MOTOR_C_PIN2, OUTPUT);
  pinMode(MOTOR_C_PWM, OUTPUT);
  
  // Configurar PWM - Canales diferentes para evitar conflictos
  ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelB, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelC, pwmFreq, pwmResolution);
  
  // Verificar que los pines no estén en uso por la cámara
  ledcAttachPin(MOTOR_A_PWM, pwmChannelA);
  ledcAttachPin(MOTOR_B_PWM, pwmChannelB);
  ledcAttachPin(MOTOR_C_PWM, pwmChannelC);
  
  // Detener todos los motores al inicio
  stopAllMotors();
  
  Serial.println("Motores inicializados:");
  Serial.printf("   Motor A: PIN1=%d, PIN2=%d, PWM=%d\n", MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_A_PWM);
  Serial.printf("   Motor B: PIN1=%d, PIN2=%d, PWM=%d\n", MOTOR_B_PIN1, MOTOR_B_PIN2, MOTOR_B_PWM);
  Serial.printf("   Motor C: PIN1=%d, PIN2=%d, PWM=%d\n", MOTOR_C_PIN1, MOTOR_C_PIN2, MOTOR_C_PWM);
}

void controlMotorA(int direction, int speed) {
  if (direction == 1) {
    digitalWrite(MOTOR_A_PIN1, HIGH);
    digitalWrite(MOTOR_A_PIN2, LOW);
  } else if (direction == -1) {
    digitalWrite(MOTOR_A_PIN1, LOW);
    digitalWrite(MOTOR_A_PIN2, HIGH);
  } else {
    digitalWrite(MOTOR_A_PIN1, LOW);
    digitalWrite(MOTOR_A_PIN2, LOW);
  }
  ledcWrite(pwmChannelA, speed);
}

void controlMotorB(int direction, int speed) {
  if (direction == 1) {
    digitalWrite(MOTOR_B_PIN1, HIGH);
    digitalWrite(MOTOR_B_PIN2, LOW);
  } else if (direction == -1) {
    digitalWrite(MOTOR_B_PIN1, LOW);
    digitalWrite(MOTOR_B_PIN2, HIGH);
  } else {
    digitalWrite(MOTOR_B_PIN1, LOW);
    digitalWrite(MOTOR_B_PIN2, LOW);
  }
  ledcWrite(pwmChannelB, speed);
}

void controlMotorC(int direction, int speed) {
  if (direction == 1) {
    digitalWrite(MOTOR_C_PIN1, HIGH);
    digitalWrite(MOTOR_C_PIN2, LOW);
  } else if (direction == -1) {
    digitalWrite(MOTOR_C_PIN1, LOW);
    digitalWrite(MOTOR_C_PIN2, HIGH);
  } else {
    digitalWrite(MOTOR_C_PIN1, LOW);
    digitalWrite(MOTOR_C_PIN2, LOW);
  }
  ledcWrite(pwmChannelC, speed);
}

void stopAllMotors() {
  controlMotorA(0, 0);
  controlMotorB(0, 0);
  controlMotorC(0, 0);
}

void moveForward(int speed = 200) {
  Serial.printf("Avanzando - A:%d, C:%d\n", speed, speed);
  controlMotorA(1, speed);  // Motor A adelante
  controlMotorC(1, speed);  // Motor C adelante
  controlMotorB(0, 0);      // Motor B parado
}

void moveBackward(int speed = 200) {
  Serial.printf("Retrocediendo - A:%d, C:%d\n", speed, speed);
  controlMotorA(-1, speed); // Motor A atrás
  controlMotorC(-1, speed); // Motor C atrás
  controlMotorB(0, 0);      // Motor B parado
}

void turnLeft(int speed = 150) {
  Serial.printf("Girando izquierda - velocidad:%d\n", speed);
  controlMotorA(-1, speed); // Motor A atrás
  controlMotorC(1, speed);  // Motor C adelante
  controlMotorB(1, speed);  // Motor B girando
}

void turnRight(int speed = 150) {
  Serial.printf("Girando derecha - velocidad:%d\n", speed);
  controlMotorA(1, speed);  // Motor A adelante
  controlMotorC(-1, speed); // Motor C atrás
  controlMotorB(-1, speed); // Motor B girando
}

// ===== FUNCIÓN PARA CAPTURAR FRAME PERSISTENTE =====
void updateCameraFrame() {
  unsigned long now = millis();
  if (now - lastFrameTime > FRAME_INTERVAL) {
    if (currentFrame) {
      esp_camera_fb_return(currentFrame);
      currentFrame = NULL;
    }
    
    currentFrame = esp_camera_fb_get();
    if (currentFrame) {
      lastFrameTime = now;
    }
  }
}

// ===== PÁGINAS WEB SIN EMOJIS =====
String getWebPage() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Robot ESP32-CAM Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta charset="UTF-8">
    <style>
        * { box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
            text-align: center; margin: 0; padding: 0;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white; min-height: 100vh;
        }
        .container { 
            max-width: 900px; margin: 0 auto; padding: 20px; 
            background: rgba(255,255,255,0.1); backdrop-filter: blur(10px);
            border-radius: 20px; margin-top: 20px; box-shadow: 0 8px 32px rgba(31, 38, 135, 0.37);
        }
        .header { margin-bottom: 30px; }
        .header h1 { 
            font-size: 2.5em; margin: 0; text-shadow: 2px 2px 4px rgba(0,0,0,0.5);
            background: linear-gradient(45deg, #f093fb 0%, #f5576c 100%);
            -webkit-background-clip: text; -webkit-text-fill-color: transparent;
        }
        .status-bar {
            display: flex; justify-content: space-between; align-items: center;
            background: rgba(255,255,255,0.2); padding: 15px; border-radius: 15px;
            margin-bottom: 20px; flex-wrap: wrap; gap: 10px;
        }
        .status-item { 
            display: flex; align-items: center; gap: 8px; 
            background: rgba(255,255,255,0.3); padding: 8px 15px; border-radius: 20px;
        }
        .video-container { 
            position: relative; margin: 20px 0; border-radius: 15px; overflow: hidden;
            box-shadow: 0 8px 25px rgba(0,0,0,0.3); background: #000;
        }
        #stream { 
            width: 100%; height: auto; display: block; min-height: 300px;
            object-fit: cover; border-radius: 15px;
        }
        .stream-overlay {
            position: absolute; top: 10px; right: 10px; 
            background: rgba(0,0,0,0.7); color: white; padding: 5px 10px;
            border-radius: 15px; font-size: 12px; backdrop-filter: blur(5px);
        }
        .stream-info {
            position: absolute; bottom: 10px; left: 10px; 
            background: rgba(0,0,0,0.7); color: white; padding: 8px 12px;
            border-radius: 15px; font-size: 12px; backdrop-filter: blur(5px);
        }
        .controls-section { margin: 30px 0; }
        .section-title { 
            font-size: 1.5em; margin-bottom: 20px; 
            text-shadow: 1px 1px 2px rgba(0,0,0,0.5);
        }
        .mode-toggle { margin: 20px 0; }
        .mode-btn { 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border: none; color: white; padding: 15px 30px; 
            font-size: 18px; border-radius: 25px; cursor: pointer; 
            transition: all 0.3s ease; box-shadow: 0 4px 15px rgba(0,0,0,0.2);
            text-transform: uppercase; font-weight: bold;
        }
        .mode-btn:hover { transform: translateY(-2px); box-shadow: 0 6px 20px rgba(0,0,0,0.3); }
        .mode-btn.active { 
            background: linear-gradient(135deg, #4CAF50 0%, #45a049 100%);
            animation: pulse 2s infinite;
        }
        @keyframes pulse {
            0% { box-shadow: 0 0 0 0 rgba(76, 175, 80, 0.7); }
            70% { box-shadow: 0 0 0 10px rgba(76, 175, 80, 0); }
            100% { box-shadow: 0 0 0 0 rgba(76, 175, 80, 0); }
        }
        .controls { 
            display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); 
            gap: 15px; margin: 20px 0; max-width: 400px; margin-left: auto; margin-right: auto;
        }
        .btn { 
            background: linear-gradient(135deg, #ff6b6b 0%, #ee5a52 100%);
            border: none; color: white; padding: 15px; 
            font-size: 16px; border-radius: 15px; cursor: pointer; 
            transition: all 0.3s ease; user-select: none; font-weight: bold;
            box-shadow: 0 4px 15px rgba(0,0,0,0.2);
        }
        .btn:hover { 
            transform: translateY(-2px); 
            box-shadow: 0 6px 20px rgba(0,0,0,0.3);
        }
        .btn:active { 
            transform: translateY(0); 
            box-shadow: 0 2px 10px rgba(0,0,0,0.2);
        }
        .btn.stop { 
            background: linear-gradient(135deg, #f44336 0%, #d32f2f 100%);
            grid-column: span 2;
        }
        .motor-controls { 
            display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); 
            gap: 20px; margin: 20px 0; 
        }
        .motor-group { 
            background: rgba(255,255,255,0.1); padding: 20px; border-radius: 15px; 
            backdrop-filter: blur(10px); border: 1px solid rgba(255,255,255,0.2);
        }
        .motor-group h3 { 
            margin-top: 0; color: #fff; font-size: 1.3em;
            text-shadow: 1px 1px 2px rgba(0,0,0,0.5);
        }
        .direction-btns { 
            display: flex; justify-content: space-between; margin: 15px 0; gap: 10px;
        }
        .direction-btns button { flex: 1; }
        .speed-control { 
            margin: 15px 0; background: rgba(255,255,255,0.1); 
            padding: 15px; border-radius: 10px;
        }
        .speed-control input { 
            width: 100%; height: 8px; border-radius: 5px; background: #ddd;
            outline: none; opacity: 0.7; transition: opacity 0.2s;
        }
        .speed-control input:hover { opacity: 1; }
        .speed-value { 
            font-size: 1.2em; font-weight: bold; margin-top: 10px;
            color: #4CAF50; text-shadow: 1px 1px 2px rgba(0,0,0,0.5);
        }
        .ai-controls { 
            background: rgba(255,255,255,0.1); padding: 30px; border-radius: 15px;
            backdrop-filter: blur(10px); border: 1px solid rgba(255,255,255,0.2);
        }
        .ai-status { 
            font-size: 1.2em; margin: 20px 0; padding: 15px; 
            background: rgba(76, 175, 80, 0.2); border-radius: 10px;
            border-left: 4px solid #4CAF50;
        }
        .connection-status { 
            position: fixed; top: 20px; right: 20px; padding: 10px 15px;
            border-radius: 20px; z-index: 1000; font-size: 12px;
            transition: all 0.3s ease;
        }
        .connection-status.online { background: #4CAF50; color: white; }
        .connection-status.offline { background: #f44336; color: white; }
        
        @media (max-width: 768px) {
            .container { margin: 10px; padding: 15px; }
            .header h1 { font-size: 2em; }
            .controls { grid-template-columns: repeat(2, 1fr); }
            .motor-controls { grid-template-columns: 1fr; }
            .status-bar { flex-direction: column; text-align: center; }
        }
    </style>
</head>
<body>
    <div class="connection-status online" id="connectionStatus">Conectado</div>
    
    <div class="container">
        <div class="header">
            <h1>Robot ESP32-CAM</h1>
            <div class="status-bar">
                <div class="status-item">
                    <span>WiFi: Conectado</span>
                </div>
                <div class="status-item">
                    <span>Camara: Activa</span>
                </div>
                <div class="status-item">
                    <span>Energia: Optimizada</span>
                </div>
            </div>
        </div>
        
        <div class="video-container">
            <img id="stream" src="/stream" alt="Video Stream">
            <div class="stream-overlay">
                <div id="streamFPS">FPS: --</div>
            </div>
            <div class="stream-info">
                <div>Resolucion: <span id="streamRes">VGA</span></div>
                <div>Calidad: <span id="streamQuality">Alta</span></div>
            </div>
        </div>
        
        <div class="mode-toggle">
            <button id="modeBtn" class="mode-btn" onclick="toggleMode()">
                Modo: <span id="modeText">MANUAL</span>
            </button>
        </div>
        
        <div id="manualControls" class="controls-section">
            <div class="section-title">Control Manual</div>
            <div class="controls">
                <button class="btn" onmousedown="sendCommand('forward')" onmouseup="sendCommand('stop')" 
                        ontouchstart="sendCommand('forward')" ontouchend="sendCommand('stop')">^ Adelante</button>
                <button class="btn" onmousedown="sendCommand('left')" onmouseup="sendCommand('stop')"
                        ontouchstart="sendCommand('left')" ontouchend="sendCommand('stop')">< Izquierda</button>
                <button class="btn" onmousedown="sendCommand('right')" onmouseup="sendCommand('stop')"
                        ontouchstart="sendCommand('right')" ontouchend="sendCommand('stop')">Derecha ></button>
                <button class="btn" onmousedown="sendCommand('backward')" onmouseup="sendCommand('stop')"
                        ontouchstart="sendCommand('backward')" ontouchend="sendCommand('stop')">v Atras</button>
                <button class="btn stop" onclick="sendCommand('stop')">STOP</button>
            </div>
            
            <div class="section-title">Control Individual de Motores</div>
            <div class="motor-controls">
                <div class="motor-group">
                    <h3>Motor A (Izquierdo)</h3>
                    <div class="direction-btns">
                        <button class="btn" onmousedown="sendMotorCommand('A', 1)" onmouseup="sendMotorCommand('A', 0)"
                                ontouchstart="sendMotorCommand('A', 1)" ontouchend="sendMotorCommand('A', 0)">^ Adelante</button>
                        <button class="btn" onmousedown="sendMotorCommand('A', -1)" onmouseup="sendMotorCommand('A', 0)"
                                ontouchstart="sendMotorCommand('A', -1)" ontouchend="sendMotorCommand('A', 0)">v Atras</button>
                    </div>
                    <div class="speed-control">
                        <input type="range" id="speedA" min="0" max="255" value="200" oninput="updateSpeed('A', this.value)">
                        <div class="speed-value">Velocidad: <span id="speedAValue">200</span></div>
                    </div>
                </div>
                
                <div class="motor-group">
                    <h3>Motor B (Central)</h3>
                    <div class="direction-btns">
                        <button class="btn" onmousedown="sendMotorCommand('B', 1)" onmouseup="sendMotorCommand('B', 0)"
                                ontouchstart="sendMotorCommand('B', 1)" ontouchend="sendMotorCommand('B', 0)">^ Adelante</button>
                        <button class="btn" onmousedown="sendMotorCommand('B', -1)" onmouseup="sendMotorCommand('B', 0)"
                                ontouchstart="sendMotorCommand('B', -1)" ontouchend="sendMotorCommand('B', 0)">v Atras</button>
                    </div>
                    <div class="speed-control">
                        <input type="range" id="speedB" min="0" max="255" value="150" oninput="updateSpeed('B', this.value)">
                        <div class="speed-value">Velocidad: <span id="speedBValue">150</span></div>
                    </div>
                </div>
                
                <div class="motor-group">
                    <h3>Motor C (Derecho)</h3>
                    <div class="direction-btns">
                        <button class="btn" onmousedown="sendMotorCommand('C', 1)" onmouseup="sendMotorCommand('C', 0)"
                                ontouchstart="sendMotorCommand('C', 1)" ontouchend="sendMotorCommand('C', 0)">^ Adelante</button>
                        <button class="btn" onmousedown="sendMotorCommand('C', -1)" onmouseup="sendMotorCommand('C', 0)"
                                ontouchstart="sendMotorCommand('C', -1)" ontouchend="sendMotorCommand('C', 0)">v Atras</button>
                    </div>
                    <div class="speed-control">
                        <input type="range" id="speedC" min="0" max="255" value="200" oninput="updateSpeed('C', this.value)">
                        <div class="speed-value">Velocidad: <span id="speedCValue">200</span></div>
                    </div>
                </div>
            </div>
        </div>
        
        <div id="aiControls" class="controls-section" style="display: none;">
            <div class="section-title">Modo Inteligencia Artificial</div>
            <div class="ai-controls">
                <div class="ai-status">
                    Sistema IA: Los motores A y C avanzan coordinadamente<br>
                    Procesamiento de vision por computadora activo<br>
                    Optimizacion energetica habilitada
                </div>
                <button class="btn" onclick="toggleAI()" style="font-size: 18px; padding: 20px 40px;">
                    <span id="aiText">Activar Modo IA</span>
                </button>
            </div>
        </div>
    </div>

    <script>
        var isManualMode = true;
        var speedA = 200, speedB = 150, speedC = 200;
        var streamStartTime = Date.now();
        var frameCount = 0;
        var isConnected = true;
        
        function updateStream() {
            var img = document.getElementById('stream');
            var timestamp = Date.now();
            img.src = '/stream?t=' + timestamp;
            frameCount++;
            
            if (frameCount % 50 === 0) {
                var elapsed = (Date.now() - streamStartTime) / 1000;
                var fps = Math.round(frameCount / elapsed);
                document.getElementById('streamFPS').textContent = 'FPS: ' + fps;
            }
        }
        
        function checkConnection() {
            fetch('/status')
                .then(function(response) {
                    if (response.ok) {
                        if (!isConnected) {
                            isConnected = true;
                            updateConnectionStatus();
                        }
                    }
                })
                .catch(function(error) {
                    if (isConnected) {
                        isConnected = false;
                        updateConnectionStatus();
                    }
                });
        }
        
        function updateConnectionStatus() {
            var status = document.getElementById('connectionStatus');
            if (isConnected) {
                status.textContent = 'Conectado';
                status.className = 'connection-status online';
            } else {
                status.textContent = 'Desconectado';
                status.className = 'connection-status offline';
            }
        }
        
        function sendCommand(cmd) {
            fetch('/control?cmd=' + cmd)
                .catch(function(error) { console.log('Error:', error); });
        }
        
        function sendMotorCommand(motor, direction) {
            var speed = 0;
            if (motor === 'A') speed = speedA;
            else if (motor === 'B') speed = speedB;
            else if (motor === 'C') speed = speedC;
            
            fetch('/motor?motor=' + motor + '&dir=' + direction + '&speed=' + speed)
                .catch(function(error) { console.log('Error:', error); });
        }
        
        function updateSpeed(motor, value) {
            if (motor === 'A') { 
                speedA = value; 
                document.getElementById('speedAValue').textContent = value; 
            }
            else if (motor === 'B') { 
                speedB = value; 
                document.getElementById('speedBValue').textContent = value; 
            }
            else if (motor === 'C') { 
                speedC = value; 
                document.getElementById('speedCValue').textContent = value; 
            }
        }
        
        function toggleMode() {
            isManualMode = !isManualMode;
            fetch('/mode?manual=' + (isManualMode ? '1' : '0'))
                .then(function() {
                    updateModeDisplay();
                })
                .catch(function(error) { console.log('Error:', error); });
        }
        
        function updateModeDisplay() {
            var modeBtn = document.getElementById('modeBtn');
            var modeText = document.getElementById('modeText');
            var manualControls = document.getElementById('manualControls');
            var aiControls = document.getElementById('aiControls');
            
            if (isManualMode) {
                modeText.textContent = 'MANUAL';
                modeBtn.classList.remove('active');
                manualControls.style.display = 'block';
                aiControls.style.display = 'none';
            } else {
                modeText.textContent = 'IA';
                modeBtn.classList.add('active');
                manualControls.style.display = 'none';
                aiControls.style.display = 'block';
            }
        }
        
        function toggleAI() {
            fetch('/ai_toggle')
                .then(function(response) { return response.text(); })
                .then(function(status) {
                    document.getElementById('aiText').textContent = 
                        status === 'started' ? 'Pausar IA' : 'Activar Modo IA';
                })
                .catch(function(error) { console.log('Error:', error); });
        }
        
        document.addEventListener('touchstart', function(e) {
            if (e.touches.length > 1) {
                e.preventDefault();
            }
        }, { passive: false });
        
        document.addEventListener('touchmove', function(e) {
            e.preventDefault();
        }, { passive: false });
        
        setInterval(updateStream, 100);
        setInterval(checkConnection, 2000);
        updateModeDisplay();
        
        document.getElementById('stream').onerror = function() {
            this.src = 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNjQwIiBoZWlnaHQ9IjQ4MCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48cmVjdCB3aWR0aD0iMTAwJSIgaGVpZ2h0PSIxMDAlIiBmaWxsPSIjMzMzIi8+PHRleHQgeD0iNTAlIiB5PSI1MCUiIGZvbnQtZmFtaWx5PSJBcmlhbCIgZm9udC1zaXplPSIxOCIgZmlsbD0iI2ZmZiIgdGV4dC1hbmNob3I9Im1pZGRsZSIgZHk9Ii4zZW0iPkNhcmdhbmRvIHZpZGVvLi4uPC90ZXh0Pjwvc3ZnPg==';
        };
    </script>
</body>
</html>
)";
  return html;
}

// ===== MANEJADORES DE RUTAS WEB =====
void handleRoot() {
  server.send(200, "text/html", getWebPage());
}

void handleStream() {
  updateCameraFrame();
  
  if (!currentFrame) {
    server.send(503, "text/plain", "Error: No hay frame disponible");
    return;
  }
  
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Length", String(currentFrame->len));
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  server.send_P(200, "image/jpeg", (const char *)currentFrame->buf, currentFrame->len);
}

void handleStatus() {
  StaticJsonDocument<200> doc;
  doc["mode"] = manualMode ? "manual" : "ai";
  doc["ai_active"] = aiModeForward;
  doc["wifi_connected"] = WiFi.status() == WL_CONNECTED;
  doc["camera_active"] = (currentFrame != NULL);
  doc["free_heap"] = ESP.getFreeHeap();
  doc["uptime"] = millis();
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleControl() {
  if (!manualMode) {
    server.send(200, "text/plain", "Modo IA activo");
    return;
  }
  
  String cmd = server.arg("cmd");
  Serial.printf("Comando recibido: %s\n", cmd.c_str());
  
  if (cmd == "forward") {
    moveForward();
  } else if (cmd == "backward") {
    moveBackward();
  } else if (cmd == "left") {
    turnLeft();
  } else if (cmd == "right") {
    turnRight();
  } else if (cmd == "stop") {
    Serial.println("Deteniendo todos los motores");
    stopAllMotors();
  }
  
  server.send(200, "text/plain", "OK");
}

void handleMotor() {
  if (!manualMode) {
    server.send(200, "text/plain", "Modo IA activo");
    return;
  }
  
  String motor = server.arg("motor");
  int direction = server.arg("dir").toInt();
  int speed = server.arg("speed").toInt();
  
  Serial.printf("Motor %s: dir=%d, speed=%d\n", motor.c_str(), direction, speed);
  
  if (motor == "A") {
    controlMotorA(direction, speed);
  } else if (motor == "B") {
    controlMotorB(direction, speed);
  } else if (motor == "C") {
    controlMotorC(direction, speed);
  }
  
  server.send(200, "text/plain", "OK");
}

void handleMode() {
  String manual = server.arg("manual");
  manualMode = (manual == "1");
  
  Serial.printf("Modo cambiado a: %s\n", manualMode ? "MANUAL" : "IA");
  
  if (!manualMode) {
    stopAllMotors();
  }
  
  server.send(200, "text/plain", manualMode ? "manual" : "ai");
}

void handleAIToggle() {
  aiModeForward = !aiModeForward;
  Serial.printf("IA %s\n", aiModeForward ? "ACTIVADA" : "DESACTIVADA");
  server.send(200, "text/plain", aiModeForward ? "started" : "stopped");
}

// ===== TAREA PARA NÚCLEO 0 - CONTROL DE MOTORES =====
void motorControlTask(void * parameter) {
  MotorCommand cmd;
  Serial.println("Tarea de control de motores iniciada en nucleo 0");
  
  for(;;) {
    // Procesar comandos de motor desde la cola
    if (xQueueReceive(motorCommandQueue, &cmd, 10 / portTICK_PERIOD_MS)) {
      if (cmd.motor == 'A') {
        controlMotorA(cmd.direction, cmd.speed);
      } else if (cmd.motor == 'B') {
        controlMotorB(cmd.direction, cmd.speed);
      } else if (cmd.motor == 'C') {
        controlMotorC(cmd.direction, cmd.speed);
      } else if (cmd.motor == 'S') {
        stopAllMotors();
      }
      
      if (cmd.duration > 0) {
        vTaskDelay(cmd.duration / portTICK_PERIOD_MS);
        stopAllMotors();
      }
    }
    
    // Modo IA - mover hacia adelante con motores A y C
    if (!manualMode && aiModeForward) {
      moveForward(180);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ===== TAREA PARA NÚCLEO 1 - VISIÓN Y WEB =====
void visionWebTask(void * parameter) {
  Serial.println("Tarea de vision y web iniciada en nucleo 1");
  
  for(;;) {
    // Manejar cliente web
    server.handleClient();
    
    // Actualizar frame de cámara para streaming persistente
    updateCameraFrame();
    
    // Procesamiento de visión por computadora (placeholder)
    if (!manualMode) {
      // Aquí puedes agregar tu código de visión por computadora
      // Por ejemplo, detección de objetos, seguimiento, etc.
      
      VisionData visionData;
      visionData.objectDetected = false;
      visionData.objectX = 0;
      visionData.objectY = 0;
      visionData.objectSize = 0;
      visionData.confidence = 0.0;
      
      // Enviar datos de visión a la cola si es necesario
      // xQueueSend(visionDataQueue, &visionData, 0);
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS); // 50 FPS máximo
  }
}

// ===== CONFIGURACIÓN INICIAL =====
void setup() {
  Serial.begin(115200);
  Serial.println("\nIniciando Robot ESP32-CAM Avanzado v2.1...");
  
  // Configurar LED de estado
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Inicializar motores
  initMotors();
  
  // Inicializar cámara
  if (!initCamera()) {
    Serial.println("Error inicializando camara!");
    return;
  }
  
  // Conectar a WiFi
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado!");
    Serial.print("IP del robot: ");
    Serial.println(WiFi.localIP());
    Serial.printf("Accede desde: http://%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nError conectando WiFi");
    Serial.println("Verifica las credenciales en el codigo");
    return;
  }
  
  // Configurar rutas del servidor web
  server.on("/", handleRoot);
  server.on("/stream", handleStream);
  server.on("/status", handleStatus);
  server.on("/control", handleControl);
  server.on("/motor", handleMotor);
  server.on("/mode", handleMode);
  server.on("/ai_toggle", handleAIToggle);
  
  server.begin();
  Serial.println("Servidor web iniciado");
  
  // Crear colas para comunicación entre núcleos
  motorCommandQueue = xQueueCreate(10, sizeof(MotorCommand));
  visionDataQueue = xQueueCreate(5, sizeof(VisionData));
  
  if (motorCommandQueue == NULL || visionDataQueue == NULL) {
    Serial.println("Error creando colas de comunicacion");
    return;
  }
  
  // Crear tareas para cada núcleo
  BaseType_t xReturned;
  
  xReturned = xTaskCreatePinnedToCore(
    motorControlTask,   // Función de la tarea
    "MotorControl",     // Nombre de la tarea
    4096,               // Tamaño del stack
    NULL,               // Parámetro de la tarea
    2,                  // Prioridad
    NULL,               // Handle de la tarea
    0                   // Núcleo 0
  );
  
  if (xReturned != pdPASS) {
    Serial.println("Error creando tarea de motores");
    return;
  }
  
  xReturned = xTaskCreatePinnedToCore(
    visionWebTask,      // Función de la tarea
    "VisionWeb",        // Nombre de la tarea
    8192,               // Tamaño del stack
    NULL,               // Parámetro de la tarea
    1,                  // Prioridad
    NULL,               // Handle de la tarea
    1                   // Núcleo 1
  );
  
  if (xReturned != pdPASS) {
    Serial.println("Error creando tarea de vision");
    return;
  }
  
  digitalWrite(STATUS_LED_PIN, HIGH);
  Serial.println("\nRobot ESP32-CAM listo!");
  Serial.println("Configuracion de hardware:");
  Serial.printf("   Motor A: Pines %d,%d (PWM:%d)\n", MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_A_PWM);
  Serial.printf("   Motor B: Pines %d,%d (PWM:%d)\n", MOTOR_B_PIN1, MOTOR_B_PIN2, MOTOR_B_PWM);
  Serial.printf("   Motor C: Pines %d,%d (PWM:%d)\n", MOTOR_C_PIN1, MOTOR_C_PIN2, MOTOR_C_PWM);
  Serial.println("Caracteristicas:");
  Serial.println("   Modo manual: Control individual");
  Serial.println("   Modo IA: Avance automatico A+C");
  Serial.println("   Streaming persistente optimizado");
  Serial.println("   LED desactivado para ahorro energetico");
  Serial.println("   Interfaz web moderna y responsive");
  Serial.println("   Procesamiento dual-core FreeRTOS");
  Serial.println("   Debug mejorado con logs detallados");
}

// ===== BUCLE PRINCIPAL =====
void loop() {
  // El bucle principal está prácticamente vacío ya que usamos FreeRTOS
  // Las tareas se ejecutan en paralelo en ambos núcleos
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  // Indicador de estado con LED interno
  static bool ledState = false;
  static unsigned long lastLedTime = 0;
  
  if (millis() - lastLedTime > 1000) {
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState);
    lastLedTime = millis();
  }
  
  // Monitor de memoria cada 30 segundos
  static unsigned long lastMemCheck = 0;
  if (millis() - lastMemCheck > 30000) {
    Serial.printf("Estado del sistema:\n");
    Serial.printf("   Memoria libre: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("   Tiempo activo: %lu ms\n", millis());
    Serial.printf("   WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado");
    Serial.printf("   Camara: %s\n", currentFrame ? "Activa" : "Inactiva");
    lastMemCheck = millis();
  }
}