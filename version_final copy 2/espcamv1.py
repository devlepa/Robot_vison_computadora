"""
ROBOT SEGUIDOR DE LÍNEA ESP32-CAM - VERSIÓN OPTIMIZADA PARA CURVAS PRONUNCIADAS
==============================================================================
Control tri-motor inteligente con detección avanzada de curvas
Tecnologías: Python 3, OpenCV, Pillow, Tkinter, NumPy
"""

import cv2
import numpy as np
import requests
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageTk
import io
import json
import logging
from datetime import datetime
from collections import deque
import os

# Configuración
ESP32_CAM_IP = "192.168.196.182"  # CAMBIAR POR TU IP
BASE_URL = f"http://{ESP32_CAM_IP}"
STREAM_URL = f"{BASE_URL}/stream"
AUTO_MODE_URL = f"{BASE_URL}/auto"
STOP_URL = f"{BASE_URL}/stop"
MOVE_URL = f"{BASE_URL}/move"
PARAMS_URL = f"{BASE_URL}/params"

TIMEOUT_COMANDO = 8
TIMEOUT_STREAM = 12

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Parámetros por defecto
PARAMETROS_DEFAULT = {
    'umbral_binario': 80, 'altura_roi': 60, 'zona_muerta_centro': 0.08,
    'velocidad_base': 180, 'velocidad_giro': 140,
    'canny_low': 50, 'canny_high': 150, 'canny_aperture': 3,
    'morph_kernel_size': 5, 'opening_iterations': 2, 'closing_iterations': 1,
    'blur_kernel': 5, 'contrast_alpha': 1.2, 'brightness_beta': 10,
    'hough_threshold': 50, 'min_line_length': 30, 'max_line_gap': 10,
    'pid_kp': 0.8, 'pid_ki': 0.1, 'pid_kd': 0.3,
    'area_min_threshold': 100, 'area_max_threshold': 5000, 'confidence_threshold': 0.3
}

# Presets profesionales
PRESETS = {
    "🌞 Normal": {'umbral_binario': 80, 'canny_low': 50, 'canny_high': 150},
    "🌙 Poca Luz": {'umbral_binario': 60, 'canny_low': 30, 'canny_high': 120, 'contrast_alpha': 1.5},
    "☀️ Mucha Luz": {'umbral_binario': 100, 'canny_low': 70, 'canny_high': 180, 'contrast_alpha': 1.0},
    "🏁 Línea Gruesa": {'area_min_threshold': 200, 'area_max_threshold': 8000},
    "📏 Línea Delgada": {'area_min_threshold': 50, 'area_max_threshold': 3000},
    "🏃 Alta Velocidad": {'velocidad_base': 220, 'velocidad_giro': 180, 'zona_muerta_centro': 0.05},
    "🐌 Precisión": {'velocidad_base': 120, 'velocidad_giro': 90, 'zona_muerta_centro': 0.12}
}

class VisionProcessor:
    def __init__(self):
        self.parametros = PARAMETROS_DEFAULT.copy()
        self.ultima_deteccion = {
            'linea_detectada': False, 'posicion_linea': 0.0, 'angulo_linea': 0.0,
            'centro_x': 0, 'centro_y': 0, 'confianza': 0.0, 'area_detectada': 0,
            'num_contornos': 0, 'calidad_imagen': 0.0, 'estabilidad': 0.0
        }
        self.pipeline_frames = {'original': None, 'binary': None, 'opening': None, 
                               'closing': None, 'edges': None, 'final': None}
        self.historia_posiciones = deque(maxlen=20)
        self.historia_confianza = deque(maxlen=20)
        self.error_anterior = 0.0
        self.integral_error = 0.0
        self.max_integral = 15.0
        self.fps_actual = 0
        self.fps_counter = 0
        self.tiempo_ultimo_fps = time.time()
        self.frames_procesados = 0
        self.detecciones_exitosas = 0

    def actualizar_parametros(self, nuevos_params):
        self.parametros.update(nuevos_params)

    def calcular_fps(self):
        self.fps_counter += 1
        tiempo_actual = time.time()
        if tiempo_actual - self.tiempo_ultimo_fps >= 1.0:
            self.fps_actual = self.fps_counter
            self.fps_counter = 0
            self.tiempo_ultimo_fps = tiempo_actual

    def preprocessar_imagen(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) if len(frame.shape) == 3 else frame.copy()
        alpha = max(0.5, min(3.0, self.parametros['contrast_alpha']))
        beta = max(-50, min(50, int(self.parametros['brightness_beta'])))
        gray = cv2.convertScaleAbs(gray, alpha=alpha, beta=beta)
        
        blur_size = max(1, int(self.parametros['blur_kernel']))
        if blur_size > 1 and blur_size % 2 == 0:
            blur_size += 1
        if blur_size > 1:
            gray = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
        
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        return clahe.apply(gray)

    def binarizar_imagen(self, gray):
        umbral = max(0, min(255, int(self.parametros['umbral_binario'])))
        _, binary = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY_INV)
        return binary

    def aplicar_morfologia(self, binary, operacion):
        kernel_size = max(3, int(self.parametros['morph_kernel_size']))
        if kernel_size % 2 == 0: kernel_size += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        
        if operacion == 'opening':
            iterations = max(0, int(self.parametros['opening_iterations']))
            return cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=iterations)
        else:  # closing
            iterations = max(0, int(self.parametros['closing_iterations']))
            return cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=iterations)

    def detectar_bordes_canny(self, processed):
        low = max(10, int(self.parametros['canny_low']))
        high = max(low + 20, int(self.parametros['canny_high']))
        return cv2.Canny(processed, low, high)

    def extraer_roi(self, frame):
        h, w = frame.shape[:2]
        roi_height = min(max(20, int(self.parametros['altura_roi'])), h//2)
        y_start = h - roi_height
        y_end = h
        roi = frame[y_start:y_end, 0:w]
        return roi, (0, y_start, w, y_end)

    def analizar_contornos(self, binary_roi):
        contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 0.0, 0.0, []

        h, w = binary_roi.shape[:2]
        area_min = self.parametros['area_min_threshold']
        area_max = self.parametros['area_max_threshold']
        
        mejor_contorno = None
        mejor_score = -1
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area_min <= area <= area_max:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Calcular score basado en posición central y área
                    dist_centro = abs(cx - w/2) / (w/2)
                    score = (1.0 - dist_centro) * 0.6 + min(area / 1000.0, 1.0) * 0.4
                    
                    if score > mejor_score:
                        mejor_score = score
                        mejor_contorno = {'contour': contour, 'area': area, 'centro': (cx, cy)}

        if mejor_contorno and mejor_score > 0.3:
            cx, cy = mejor_contorno['centro']
            posicion_normalizada = (cx - w/2) / (w/2)
            return (cx, cy), posicion_normalizada, 0.0, [mejor_contorno]
        
        return None, 0.0, 0.0, []

    def calcular_confianza(self, contornos_info, area_total, roi_shape):
        if not contornos_info:
            return 0.0
        
        h, w = roi_shape[:2]
        mejor = contornos_info[0]
        cx, cy = mejor['centro']
        area = mejor['area']
        
        # Factores de confianza
        dist_centro = abs(cx - w/2) / (w/2)
        factor_posicion = 1.0 - dist_centro if dist_centro < 0.5 else 0.2
        
        area_normalizada = area / (w * h)
        factor_area = 1.0 if 0.05 <= area_normalizada <= 0.3 else 0.5
        
        return min(factor_posicion * 0.6 + factor_area * 0.4, 1.0)

    def detectar_tipo_curva(self):
        if len(self.historia_posiciones) < 10:
            return "recta", 0.0
        
        posiciones = list(self.historia_posiciones)[-10:]
        varianza = np.var(posiciones)
        tendencia = np.mean(np.diff(posiciones[-5:]))
        
        if varianza > 0.15 and abs(tendencia) > 0.1:
            return "curva_pronunciada", abs(tendencia)
        elif varianza > 0.08:
            return "curva_suave", abs(tendencia)
        elif abs(tendencia) > 0.4:
            return "giro_cerrado", abs(tendencia)
        else:
            return "recta", 0.0

    def procesar_frame_completo(self, frame):
        try:
            self.calcular_fps()
            h_orig, w_orig = frame.shape[:2]
            
            # Pipeline de procesamiento
            self.pipeline_frames['original'] = frame.copy()
            
            preprocessed = self.preprocessar_imagen(frame)
            roi_preprocessed, roi_coords = self.extraer_roi(preprocessed)
            
            binary_roi = self.binarizar_imagen(roi_preprocessed)
            self.pipeline_frames['binary'] = self.crear_frame_completo(binary_roi, roi_coords, h_orig, w_orig)
            
            opened_roi = self.aplicar_morfologia(binary_roi, 'opening')
            self.pipeline_frames['opening'] = self.crear_frame_completo(opened_roi, roi_coords, h_orig, w_orig)
            
            closed_roi = self.aplicar_morfologia(opened_roi, 'closing')
            self.pipeline_frames['closing'] = self.crear_frame_completo(closed_roi, roi_coords, h_orig, w_orig)
            
            edges_roi = self.detectar_bordes_canny(closed_roi)
            self.pipeline_frames['edges'] = self.crear_frame_completo(edges_roi, roi_coords, h_orig, w_orig)
            
            centro, posicion, angulo, contornos_info = self.analizar_contornos(closed_roi)
            area_total = sum(info['area'] for info in contornos_info) if contornos_info else 0
            confianza = self.calcular_confianza(contornos_info, area_total, closed_roi.shape)
            
            # Actualizar detección
            self.frames_procesados += 1
            if centro is not None and confianza > self.parametros['confidence_threshold']:
                self.historia_posiciones.append(posicion)
                self.historia_confianza.append(confianza)
                self.detecciones_exitosas += 1
                
                self.ultima_deteccion.update({
                    'linea_detectada': True,
                    'posicion_linea': posicion,
                    'centro_x': int(centro[0]),
                    'centro_y': int(centro[1] + roi_coords[1]),
                    'confianza': confianza,
                    'area_detectada': int(area_total),
                    'num_contornos': len(contornos_info)
                })
            else:
                self.ultima_deteccion.update({
                    'linea_detectada': False,
                    'confianza': confianza,
                    'num_contornos': len(contornos_info) if contornos_info else 0
                })
            
            # Frame final con visualización
            frame_final = self.crear_frame_visualizacion(frame, roi_coords, contornos_info)
            self.pipeline_frames['final'] = frame_final
            
            return True
            
        except Exception as e:
            logger.error(f"Error procesando frame: {e}")
            return False

    def crear_frame_completo(self, roi_frame, roi_coords, h_orig, w_orig):
        if len(roi_frame.shape) == 2:
            frame_completo = np.zeros((h_orig, w_orig), dtype=np.uint8)
        else:
            frame_completo = np.zeros((h_orig, w_orig, 3), dtype=np.uint8)
        
        x1, y1, x2, y2 = roi_coords
        frame_completo[y1:y2, x1:x2] = roi_frame
        return frame_completo

    def crear_frame_visualizacion(self, frame_original, roi_coords, contornos_info):
        if len(frame_original.shape) == 2:
            vis_frame = cv2.cvtColor(frame_original, cv2.COLOR_GRAY2RGB)
        else:
            vis_frame = frame_original.copy()
        
        x1, y1, x2, y2 = roi_coords
        cv2.rectangle(vis_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(vis_frame, "ROI", (x1+5, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if contornos_info:
            for info in contornos_info:
                contour_adjusted = info['contour'].copy()
                contour_adjusted[:, :, 1] += y1
                cv2.drawContours(vis_frame, [contour_adjusted], -1, (255, 0, 255), 2)
                
                cx, cy = info['centro']
                cy_adjusted = cy + y1
                cv2.circle(vis_frame, (cx, cy_adjusted), 8, (255, 0, 0), -1)
        
        # Panel de información
        info_bg = np.zeros((100, 250, 3), dtype=np.uint8)
        cv2.putText(info_bg, f"FPS: {self.fps_actual}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(info_bg, f"Pos: {self.ultima_deteccion['posicion_linea']:.3f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(info_bg, f"Conf: {self.ultima_deteccion['confianza']:.2f}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        vis_frame[10:110, 10:260] = info_bg
        return vis_frame

    def obtener_frame_pipeline(self, etapa):
        frame = self.pipeline_frames.get(etapa, np.zeros((240, 320, 3), dtype=np.uint8))
        if frame is not None and frame.size > 0:
            if len(frame.shape) == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        return frame

    def calcular_comando_movimiento(self):
        if not self.ultima_deteccion['linea_detectada']:
            return "stop", 0, 0
        
        error = self.ultima_deteccion['posicion_linea']
        confianza = self.ultima_deteccion['confianza']
        tipo_curva, severidad = self.detectar_tipo_curva()
        
        # Zona muerta adaptativa
        zona_muerta = self.parametros['zona_muerta_centro']
        if tipo_curva == "curva_pronunciada":
            zona_muerta *= 0.5
        
        if abs(error) < zona_muerta and tipo_curva == "recta":
            return "forward", int(self.parametros['velocidad_base']), 0
        
        # PID adaptativo
        if tipo_curva == "curva_pronunciada":
            kp = self.parametros['pid_kp'] * 1.5 * confianza
            ki = self.parametros['pid_ki'] * 0.5 * confianza
            kd = self.parametros['pid_kd'] * 2.0 * confianza
        elif tipo_curva == "giro_cerrado":
            kp = self.parametros['pid_kp'] * 2.0 * confianza
            ki = self.parametros['pid_ki'] * 0.3 * confianza
            kd = self.parametros['pid_kd'] * 1.5 * confianza
        else:
            kp = self.parametros['pid_kp'] * confianza
            ki = self.parametros['pid_ki'] * confianza
            kd = self.parametros['pid_kd'] * confianza
        
        # Cálculo PID
        self.integral_error += error * ki
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        derivativo = kd * (error - self.error_anterior)
        self.error_anterior = error
        
        # Factor de velocidad
        if tipo_curva == "curva_pronunciada":
            factor_velocidad = 0.6
        elif tipo_curva == "giro_cerrado":
            factor_velocidad = 0.4
        else:
            factor_velocidad = 1.0 - min(abs(error) * 0.5, 0.7)
        
        factor_velocidad *= (0.7 + confianza * 0.3)
        
        velocidad_base = int(self.parametros['velocidad_base'] * factor_velocidad)
        velocidad_giro = int(self.parametros['velocidad_giro'] * factor_velocidad)
        
        # Comandos según error
        abs_error = abs(error)
        if abs_error < 0.08:
            return "forward", velocidad_base, 0
        elif abs_error < 0.20:
            if error < 0:
                return "left_smooth", velocidad_base, int(velocidad_base * 0.3)
            else:
                return "right_smooth", velocidad_base, int(velocidad_base * 0.3)
        elif abs_error < 0.45:
            if error < 0:
                return "left_turn", int(velocidad_giro * 0.9), int(velocidad_giro * 0.6)
            else:
                return "right_turn", int(velocidad_giro * 0.9), int(velocidad_giro * 0.6)
        elif abs_error < 0.70:
            if error < 0:
                return "left_sharp", int(velocidad_giro * 0.7), velocidad_giro
            else:
                return "right_sharp", int(velocidad_giro * 0.7), velocidad_giro
        else:
            if error < 0:
                return "left_extreme", int(velocidad_giro * 0.5), int(velocidad_giro * 1.2)
            else:
                return "right_extreme", int(velocidad_giro * 0.5), int(velocidad_giro * 1.2)

class RobotController:
    def __init__(self):
        self.conectado = False
        self.modo_automatico = False
        self.vision_processor = VisionProcessor()
        self.session = requests.Session()
        self.historial_comandos = deque(maxlen=50)

    def enviar_comando_http(self, url, datos=None, timeout=None):
        if timeout is None:
            timeout = TIMEOUT_COMANDO
        
        try:
            if datos:
                response = self.session.post(url, json=datos, timeout=timeout)
            else:
                response = self.session.get(url, timeout=timeout)
            
            response.raise_for_status()
            self.conectado = True
            try:
                return response.json()
            except:
                return {"status": "ok"}
        except Exception as e:
            self.conectado = False
            logger.warning(f"Error HTTP: {e}")
            return None

    def obtener_configuracion_motores(self, comando, vel_principal, vel_auxiliar):
        config = {
            "direction": "custom",
            "motor_a_speed": 0, "motor_a_direction": "forward",
            "motor_b_speed": 0, "motor_b_direction": "stop",
            "motor_c_speed": 0, "motor_c_direction": "forward",
            "maniobra": comando
        }
        
        if comando == "forward":
            config.update({"motor_a_speed": vel_principal, "motor_c_speed": vel_principal})
        elif comando == "left_smooth":
            config.update({"motor_a_speed": int(vel_principal * 0.6), "motor_c_speed": vel_principal})
        elif comando == "right_smooth":
            config.update({"motor_a_speed": vel_principal, "motor_c_speed": int(vel_principal * 0.6)})
        elif comando == "left_turn":
            config.update({"motor_a_speed": int(vel_principal * 0.3), "motor_c_speed": vel_principal})
        elif comando == "right_turn":
            config.update({"motor_a_speed": vel_principal, "motor_c_speed": int(vel_principal * 0.3)})
        elif comando == "left_sharp":
            config.update({"motor_a_speed": vel_auxiliar, "motor_a_direction": "backward",
                          "motor_b_speed": int(vel_auxiliar * 0.5), "motor_b_direction": "forward",
                          "motor_c_speed": vel_principal, "motor_c_direction": "forward"})
        elif comando == "right_sharp":
            config.update({"motor_a_speed": vel_principal, "motor_a_direction": "forward",
                          "motor_b_speed": int(vel_auxiliar * 0.5), "motor_b_direction": "forward",
                          "motor_c_speed": vel_auxiliar, "motor_c_direction": "backward"})
        elif comando == "left_extreme":
            config.update({"motor_a_speed": vel_auxiliar, "motor_a_direction": "backward",
                          "motor_b_speed": vel_principal, "motor_b_direction": "forward",
                          "motor_c_speed": vel_auxiliar, "motor_c_direction": "forward"})
        elif comando == "right_extreme":
            config.update({"motor_a_speed": vel_auxiliar, "motor_a_direction": "forward",
                          "motor_b_speed": vel_principal, "motor_b_direction": "forward",
                          "motor_c_speed": vel_auxiliar, "motor_c_direction": "backward"})
        elif comando == "stop":
            config.update({"direction": "stop", "motor_a_speed": 0, "motor_b_speed": 0, "motor_c_speed": 0})
        elif comando in ["left", "right", "backward"]:
            config = {"direction": comando, "speed": vel_principal}
        
        return config

    def enviar_movimiento_avanzado(self, comando, vel_principal, vel_auxiliar=0):
        if self.modo_automatico and comando != "stop":
            pass
        elif not self.modo_automatico and comando != "stop":
            return False
        
        configuracion = self.obtener_configuracion_motores(comando, vel_principal, vel_auxiliar)
        resultado = self.enviar_comando_http(MOVE_URL, configuracion)
        
        self.historial_comandos.append({
            'comando': comando, 'velocidad_principal': vel_principal,
            'velocidad_auxiliar': vel_auxiliar, 'timestamp': datetime.now(),
            'exitoso': resultado is not None
        })
        
        return resultado is not None

    def alternar_modo_automatico(self):
        nuevo_modo = not self.modo_automatico
        resultado = self.enviar_comando_http(AUTO_MODE_URL, {"auto": nuevo_modo})
        
        if resultado:
            self.modo_automatico = nuevo_modo
            logger.info(f"Modo: {'Automático' if nuevo_modo else 'Manual'}")
            return True
        return False

    def parar_robot(self):
        resultado = self.enviar_comando_http(STOP_URL, {})
        if resultado:
            self.modo_automatico = False
            return True
        return False

    def control_automatico(self):
        if not self.modo_automatico:
            return
        
        resultado = self.vision_processor.calcular_comando_movimiento()
        if len(resultado) == 3:
            comando, vel_principal, vel_auxiliar = resultado
            if comando and comando != "stop":
                self.enviar_movimiento_avanzado(comando, vel_principal, vel_auxiliar)
            elif not self.vision_processor.ultima_deteccion['linea_detectada']:
                self.enviar_movimiento_avanzado("stop", 0, 0)

    def obtener_estadisticas(self):
        if not self.historial_comandos:
            return {'total_comandos': 0, 'curvas_pronunciadas': 0, 'exito_porcentaje': 0}
        
        comandos = list(self.historial_comandos)[-30:]
        stats = {'total_comandos': len(comandos), 'curvas_pronunciadas': 0, 'exito_porcentaje': 0}
        
        for cmd in comandos:
            if 'sharp' in cmd['comando'] or 'turn' in cmd['comando']:
                stats['curvas_pronunciadas'] += 1
            if cmd['exitoso']:
                stats['exito_porcentaje'] += 1
        
        if stats['total_comandos'] > 0:
            stats['exito_porcentaje'] = (stats['exito_porcentaje'] / stats['total_comandos']) * 100
        
        return stats

# Variables globales
robot_controller = RobotController()
stop_stream_flag = threading.Event()
parametros_actuales = PARAMETROS_DEFAULT.copy()
sliders = {}
pipeline_labels = {}
estado_label = None
modo_label = None
linea_label = None
auto_button = None

def hilo_stream_video():
    logger.info("Iniciando stream de video...")
    errores_consecutivos = 0
    
    while not stop_stream_flag.is_set():
        try:
            with requests.get(STREAM_URL, stream=True, timeout=TIMEOUT_STREAM) as r:
                r.raise_for_status()
                robot_controller.conectado = True
                errores_consecutivos = 0
                
                bytes_data = b''
                for chunk in r.iter_content(chunk_size=4096):
                    if stop_stream_flag.is_set():
                        break
                    
                    bytes_data += chunk
                    inicio = bytes_data.find(b'\xff\xd8')
                    fin = bytes_data.find(b'\xff\xd9')
                    
                    if inicio != -1 and fin != -1 and fin > inicio:
                        jpg = bytes_data[inicio:fin+2]
                        bytes_data = bytes_data[fin+2:]
                        
                        try:
                            img = Image.open(io.BytesIO(jpg))
                            frame_np = np.array(img)
                            
                            robot_controller.vision_processor.actualizar_parametros(parametros_actuales)
                            
                            if robot_controller.vision_processor.procesar_frame_completo(frame_np):
                                mostrar_pipeline()
                                
                                if robot_controller.modo_automatico:
                                    robot_controller.control_automatico()
                                
                                actualizar_interfaz()
                        except Exception as e:
                            logger.error(f"Error procesando frame: {e}")
        
        except Exception as e:
            errores_consecutivos += 1
            robot_controller.conectado = False
            if errores_consecutivos >= 5:
                break
            time.sleep(min(2 ** errores_consecutivos, 30))

def mostrar_pipeline():
    etapas = ['original', 'binary', 'opening', 'closing', 'edges', 'final']
    
    for etapa in etapas:
        if etapa in pipeline_labels and pipeline_labels[etapa].winfo_exists():
            frame = robot_controller.vision_processor.obtener_frame_pipeline(etapa)
            
            if frame is not None and frame.size > 0:
                h, w = frame.shape[:2]
                ancho_objetivo = 300
                alto_objetivo = int(h * (ancho_objetivo / w))
                
                frame_resized = cv2.resize(frame, (ancho_objetivo, alto_objetivo))
                img_pil = Image.fromarray(frame_resized)
                foto = ImageTk.PhotoImage(image=img_pil)
                
                pipeline_labels[etapa].config(image=foto)
                pipeline_labels[etapa].image = foto

def actualizar_interfaz():
    try:
        if linea_label and linea_label.winfo_exists():
            det = robot_controller.vision_processor.ultima_deteccion
            fps = robot_controller.vision_processor.fps_actual
            tipo_curva, severidad = robot_controller.vision_processor.detectar_tipo_curva()
            stats = robot_controller.obtener_estadisticas()
            
            if det['linea_detectada']:
                status_icon = "✅"
                color = "#27ae60"
                texto_base = "LÍNEA DETECTADA"
            else:
                status_icon = "❌"
                color = "#e74c3c"
                texto_base = "SIN LÍNEA"
            
            if tipo_curva == "curva_pronunciada":
                texto_base += " | 🔄 CURVA PRONUNCIADA"
                color = "#f39c12"
            elif tipo_curva == "giro_cerrado":
                texto_base += " | ↩️ GIRO CERRADO"
                color = "#e67e22"
            
            texto_completo = (f"{status_icon} {texto_base} | Pos: {det['posicion_linea']:.3f} | "
                            f"Conf: {det['confianza']:.2f} | FPS: {fps} | "
                            f"Curvas: {stats['curvas_pronunciadas']}")
            
            linea_label.config(text=texto_completo, background=color)
        
        if estado_label and estado_label.winfo_exists():
            conexion_icon = "🟢" if robot_controller.conectado else "🔴"
            precision = (robot_controller.vision_processor.detecciones_exitosas / 
                        max(robot_controller.vision_processor.frames_procesados, 1)) * 100
            estado_label.config(text=f"{conexion_icon} Conectado | Precisión: {precision:.1f}%")
    except:
        pass

def callback_slider(param_name):
    def callback(valor):
        try:
            if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro',
                             'canny_low', 'canny_high', 'canny_aperture', 'morph_kernel_size',
                             'opening_iterations', 'closing_iterations', 'blur_kernel', 'brightness_beta',
                             'hough_threshold', 'min_line_length', 'max_line_gap',
                             'area_min_threshold', 'area_max_threshold']:
                parametros_actuales[param_name] = int(float(valor))
            else:
                parametros_actuales[param_name] = float(valor)
            
            if param_name in sliders:
                slider_info = sliders[param_name]
                if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro',
                                 'canny_low', 'canny_high', 'canny_aperture', 'morph_kernel_size',
                                 'opening_iterations', 'closing_iterations', 'blur_kernel', 'brightness_beta',
                                 'hough_threshold', 'min_line_length', 'max_line_gap',
                                 'area_min_threshold', 'area_max_threshold']:
                    slider_info['valor_label'].config(text=str(int(float(valor))))
                else:
                    slider_info['valor_label'].config(text=f"{float(valor):.3f}")
        except Exception as e:
            logger.error(f"Error callback slider {param_name}: {e}")
    
    return callback

def aplicar_preset(preset_name):
    if preset_name in PRESETS:
        preset_params = PRESETS[preset_name]
        
        for key, value in preset_params.items():
            if key in parametros_actuales:
                parametros_actuales[key] = value
        
        for param_name, value in preset_params.items():
            if param_name in sliders:
                sliders[param_name]['slider'].set(value)
                if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro',
                                 'canny_low', 'canny_high', 'canny_aperture', 'morph_kernel_size',
                                 'opening_iterations', 'closing_iterations', 'blur_kernel', 'brightness_beta',
                                 'hough_threshold', 'min_line_length', 'max_line_gap',
                                 'area_min_threshold', 'area_max_threshold']:
                    sliders[param_name]['valor_label'].config(text=str(value))
                else:
                    sliders[param_name]['valor_label'].config(text=f"{value:.3f}")
        
        messagebox.showinfo("Preset Aplicado", f"✅ {preset_name} aplicado correctamente")

def resetear_parametros():
    if messagebox.askyesno("Confirmar Reset", "¿Resetear todos los parámetros?"):
        global parametros_actuales
        parametros_actuales = PARAMETROS_DEFAULT.copy()
        
        for param_name, slider_info in sliders.items():
            if param_name in parametros_actuales:
                slider_info['slider'].set(parametros_actuales[param_name])
                if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro',
                                 'canny_low', 'canny_high', 'canny_aperture', 'morph_kernel_size',
                                 'opening_iterations', 'closing_iterations', 'blur_kernel', 'brightness_beta',
                                 'hough_threshold', 'min_line_length', 'max_line_gap',
                                 'area_min_threshold', 'area_max_threshold']:
                    slider_info['valor_label'].config(text=str(parametros_actuales[param_name]))
                else:
                    slider_info['valor_label'].config(text=f"{parametros_actuales[param_name]:.3f}")
        
        messagebox.showinfo("Reset Completo", "✅ Parámetros reseteados")

def crear_slider_compacto(parent, param_name, config):
    slider_frame = tk.Frame(parent, bg='#34495e', relief='raised', bd=1)
    slider_frame.pack(fill="x", padx=2, pady=1)
    
    label = tk.Label(slider_frame, text=config['label'], 
                    font=("Arial", 8, "bold"), bg='#34495e', fg='white', anchor='w')
    label.pack(side="top", fill="x", padx=5, pady=1)
    
    control_frame = tk.Frame(slider_frame, bg='#34495e')
    control_frame.pack(fill="x", padx=5, pady=2)
    
    slider = tk.Scale(control_frame, from_=config['min'], to=config['max'], 
                     resolution=config['step'], orient="horizontal", 
                     command=callback_slider(param_name),
                     bg='#2c3e50', fg='white', highlightthickness=0, 
                     troughcolor='#7f8c8d', activebackground='#3498db',
                     length=180, width=15)
    slider.set(parametros_actuales[param_name])
    slider.pack(side="left", fill="x", expand=True)
    
    if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro',
                     'canny_low', 'canny_high', 'canny_aperture', 'morph_kernel_size',
                     'opening_iterations', 'closing_iterations', 'blur_kernel', 'brightness_beta',
                     'hough_threshold', 'min_line_length', 'max_line_gap',
                     'area_min_threshold', 'area_max_threshold']:
        valor_texto = str(parametros_actuales[param_name])
    else:
        valor_texto = f"{parametros_actuales[param_name]:.3f}"
        
    valor_label = tk.Label(control_frame, text=valor_texto, 
                          font=("Arial", 9, "bold"), bg='#e74c3c', fg='white', 
                          width=8, relief='raised', bd=1)
    valor_label.pack(side="right", padx=(5, 0))
    
    sliders[param_name] = {'slider': slider, 'valor_label': valor_label}

def crear_interfaz():
    global estado_label, modo_label, linea_label, auto_button, pipeline_labels
    
    root = tk.Tk()
    root.title("🤖 Robot Seguidor Ultra - Curvas Pronunciadas")
    root.state('zoomed')
    root.configure(bg='#1a1a1a')
    
    # Header
    header_frame = tk.Frame(root, bg='#2c3e50', height=80, relief='raised', bd=2)
    header_frame.pack(fill="x", padx=5, pady=5)
    header_frame.pack_propagate(False)
    
    titulo_label = tk.Label(header_frame, 
                           text="🤖 ROBOT SEGUIDOR ULTRA - CONTROL TRI-MOTOR PARA CURVAS", 
                           font=("Arial", 16, "bold"), bg='#2c3e50', fg='#ecf0f1')
    titulo_label.pack(pady=5)
    
    estado_frame = tk.Frame(header_frame, bg='#2c3e50')
    estado_frame.pack(fill="x", padx=20)
    
    estado_label = tk.Label(estado_frame, text="🔴 Conectando...", 
                           font=("Arial", 11, "bold"), bg='#34495e', fg='white', 
                           relief='raised', bd=2, padx=10, pady=5)
    estado_label.pack(side="left", padx=5)
    
    modo_label = tk.Label(estado_frame, text="📱 Modo: Manual", 
                         font=("Arial", 11, "bold"), bg='#34495e', fg='white', 
                         relief='raised', bd=2, padx=10, pady=5)
    modo_label.pack(side="left", padx=5)
    
    linea_label = tk.Label(estado_frame, text="🎯 Iniciando detección...", 
                          font=("Arial", 11, "bold"), bg='#f39c12', fg='white', 
                          relief='raised', bd=2, padx=10, pady=5)
    linea_label.pack(side="right", padx=5)
    
    # Main container
    main_container = tk.Frame(root, bg='#1a1a1a')
    main_container.pack(fill="both", expand=True, padx=5)
    
    # Left side - Images
    images_frame = tk.Frame(main_container, bg='#2c3e50', relief='raised', bd=3)
    images_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
    
    pipeline_titulo = tk.Label(images_frame, 
                              text="🔍 PIPELINE EN TIEMPO REAL", 
                              font=("Arial", 14, "bold"), bg='#2c3e50', fg='#ecf0f1')
    pipeline_titulo.pack(pady=10)
    
    grid_frame = tk.Frame(images_frame, bg='#2c3e50')
    grid_frame.pack(fill="both", expand=True, padx=10, pady=10)
    
    for i in range(2):
        grid_frame.grid_rowconfigure(i, weight=1)
    for j in range(3):
        grid_frame.grid_columnconfigure(j, weight=1)
    
    etapas_pipeline = [
        ('original', '📹 ORIGINAL', '#3498db'),
        ('binary', '⚫ BINARIO', '#9b59b6'),
        ('opening', '🔵 APERTURA', '#2ecc71'),
        ('closing', '🔴 CIERRE', '#e74c3c'),
        ('edges', '⚡ BORDES', '#f39c12'),
        ('final', '🎯 FINAL', '#1abc9c')
    ]
    
    for idx, (etapa, titulo, color) in enumerate(etapas_pipeline):
        fila = idx // 3
        columna = idx % 3
        
        etapa_frame = tk.Frame(grid_frame, bg=color, relief='raised', bd=3)
        etapa_frame.grid(row=fila, column=columna, padx=5, pady=5, sticky="nsew")
        
        etapa_titulo = tk.Label(etapa_frame, text=titulo, 
                               font=("Arial", 11, "bold"), bg=color, fg='white')
        etapa_titulo.pack(pady=5)
        
        etapa_label = tk.Label(etapa_frame, bg="black", fg="white", 
                              text=f"{titulo}\n\nProcesando...", 
                              font=("Arial", 10), relief='sunken', bd=2)
        etapa_label.pack(fill="both", expand=True, padx=5, pady=(0, 5))
        
        pipeline_labels[etapa] = etapa_label
    
    # Right side - Controls
    control_frame = tk.Frame(main_container, bg='#34495e', width=350, relief='raised', bd=3)
    control_frame.pack(side="right", fill="y", padx=(5, 0))
    control_frame.pack_propagate(False)
    
    control_titulo = tk.Label(control_frame, 
                             text="🎛️ CONTROL ULTRA", 
                             font=("Arial", 13, "bold"), bg='#34495e', fg='white')
    control_titulo.pack(pady=10)
    
    # Main buttons
    botones_frame = tk.Frame(control_frame, bg='#34495e')
    botones_frame.pack(fill="x", padx=10, pady=10)
    
    auto_button = tk.Button(botones_frame, text="🤖 ACTIVAR IA ULTRA", 
                           command=robot_controller.alternar_modo_automatico,
                           font=("Arial", 11, "bold"), bg="#27ae60", fg="white",
                           relief='raised', bd=3, height=2)
    auto_button.pack(fill="x", pady=2)
    
    stop_button = tk.Button(botones_frame, text="🛑 PARADA EMERGENCIA", 
                           command=robot_controller.parar_robot,
                           font=("Arial", 11, "bold"), bg="#e74c3c", fg="white",
                           relief='raised', bd=3, height=2)
    stop_button.pack(fill="x", pady=2)
    
    reset_button = tk.Button(botones_frame, text="🔄 RESET", 
                            command=resetear_parametros,
                            font=("Arial", 11, "bold"), bg="#f39c12", fg="white",
                            relief='raised', bd=3, height=2)
    reset_button.pack(fill="x", pady=2)
    
    # Presets
    preset_frame = tk.LabelFrame(control_frame, text="🎨 Presets", 
                                bg='#34495e', fg='white', font=("Arial", 10, "bold"))
    preset_frame.pack(fill="x", padx=10, pady=10)
    
    preset_var = tk.StringVar(value="Seleccionar...")
    preset_combo = ttk.Combobox(preset_frame, textvariable=preset_var, 
                               values=list(PRESETS.keys()),
                               state="readonly", font=("Arial", 9))
    preset_combo.pack(fill="x", padx=10, pady=10)
    
    def on_preset_change(event):
        preset_seleccionado = preset_var.get()
        if preset_seleccionado in PRESETS:
            aplicar_preset(preset_seleccionado)
    
    preset_combo.bind('<<ComboboxSelected>>', on_preset_change)
    
    # Parameters notebook
    notebook = ttk.Notebook(control_frame)
    notebook.pack(fill="both", expand=True, padx=10, pady=10)
    
    # Tab 1: Basic
    tab1 = tk.Frame(notebook, bg='#34495e')
    notebook.add(tab1, text="🔧 Básico")
    
    params_basicos = {
        'umbral_binario': {'label': 'Umbral (0-255)', 'min': 0, 'max': 255, 'step': 1},
        'altura_roi': {'label': 'ROI (20-120px)', 'min': 20, 'max': 120, 'step': 5},
        'zona_muerta_centro': {'label': 'Zona Muerta (0.02-0.3)', 'min': 0.02, 'max': 0.3, 'step': 0.01},
        'velocidad_base': {'label': 'Velocidad Base (100-255)', 'min': 100, 'max': 255, 'step': 5},
        'velocidad_giro': {'label': 'Velocidad Giro (80-200)', 'min': 80, 'max': 200, 'step': 5}
    }
    
    for param_name, config in params_basicos.items():
        crear_slider_compacto(tab1, param_name, config)
    
    # Tab 2: Canny
    tab2 = tk.Frame(notebook, bg='#34495e')
    notebook.add(tab2, text="⚡ Canny")
    
    params_canny = {
        'canny_low': {'label': 'Umbral Bajo (10-150)', 'min': 10, 'max': 150, 'step': 5},
        'canny_high': {'label': 'Umbral Alto (50-300)', 'min': 50, 'max': 300, 'step': 5},
        'blur_kernel': {'label': 'Desenfoque (1-15)', 'min': 1, 'max': 15, 'step': 2}
    }
    
    for param_name, config in params_canny.items():
        crear_slider_compacto(tab2, param_name, config)
    
    # Tab 3: Morphology
    tab3 = tk.Frame(notebook, bg='#34495e')
    notebook.add(tab3, text="🔵🔴 Morfología")
    
    params_morph = {
        'morph_kernel_size': {'label': 'Kernel (3-15)', 'min': 3, 'max': 15, 'step': 2},
        'opening_iterations': {'label': 'Apertura (0-5)', 'min': 0, 'max': 5, 'step': 1},
        'closing_iterations': {'label': 'Cierre (0-5)', 'min': 0, 'max': 5, 'step': 1}
    }
    
    for param_name, config in params_morph.items():
        crear_slider_compacto(tab3, param_name, config)
    
    # Tab 4: PID & Maniobras
    tab4 = tk.Frame(notebook, bg='#34495e')
    notebook.add(tab4, text="🎯 PID & Curvas")
    
    params_pid = {
        'pid_kp': {'label': 'PID Kp (0.1-2.0)', 'min': 0.1, 'max': 2.0, 'step': 0.1},
        'pid_ki': {'label': 'PID Ki (0.0-0.5)', 'min': 0.0, 'max': 0.5, 'step': 0.01},
        'pid_kd': {'label': 'PID Kd (0.0-1.0)', 'min': 0.0, 'max': 1.0, 'step': 0.05}
    }
    
    for param_name, config in params_pid.items():
        crear_slider_compacto(tab4, param_name, config)
    
    # Maniobras de prueba
    separador = tk.Frame(tab4, bg='#2c3e50', height=2)
    separador.pack(fill="x", padx=10, pady=10)
    
    maniobras_label = tk.Label(tab4, text="🏁 MANIOBRAS DE PRUEBA", 
                              font=("Arial", 9, "bold"), bg='#34495e', fg='#f39c12')
    maniobras_label.pack(pady=5)
    
    maniobras_frame = tk.Frame(tab4, bg='#34495e')
    maniobras_frame.pack(fill="x", padx=5)
    
    # Fila 1
    fila1 = tk.Frame(maniobras_frame, bg='#34495e')
    fila1.pack(fill="x", pady=1)
    
    btn_left_smooth = tk.Button(fila1, text="↖️ Suave Izq", 
                               command=lambda: robot_controller.enviar_movimiento_avanzado("left_smooth", 150, 50),
                               font=("Arial", 7, "bold"), bg="#3498db", fg="white", width=10)
    btn_left_smooth.pack(side="left", padx=1)
    
    btn_right_smooth = tk.Button(fila1, text="↗️ Suave Der", 
                                command=lambda: robot_controller.enviar_movimiento_avanzado("right_smooth", 150, 50),
                                font=("Arial", 7, "bold"), bg="#3498db", fg="white", width=10)
    btn_right_smooth.pack(side="right", padx=1)
    
    # Fila 2
    fila2 = tk.Frame(maniobras_frame, bg='#34495e')
    fila2.pack(fill="x", pady=1)
    
    btn_left_sharp = tk.Button(fila2, text="↩️ Cerrado Izq", 
                              command=lambda: robot_controller.enviar_movimiento_avanzado("left_sharp", 120, 100),
                              font=("Arial", 7, "bold"), bg="#f39c12", fg="white", width=10)
    btn_left_sharp.pack(side="left", padx=1)
    
    btn_right_sharp = tk.Button(fila2, text="↪️ Cerrado Der", 
                               command=lambda: robot_controller.enviar_movimiento_avanzado("right_sharp", 120, 100),
                               font=("Arial", 7, "bold"), bg="#f39c12", fg="white", width=10)
    btn_right_sharp.pack(side="right", padx=1)
    
    # Fila 3
    fila3 = tk.Frame(maniobras_frame, bg='#34495e')
    fila3.pack(fill="x", pady=1)
    
    btn_left_extreme = tk.Button(fila3, text="🔄 Extremo Izq", 
                                command=lambda: robot_controller.enviar_movimiento_avanzado("left_extreme", 100, 120),
                                font=("Arial", 7, "bold"), bg="#e74c3c", fg="white", width=10)
    btn_left_extreme.pack(side="left", padx=1)
    
    btn_right_extreme = tk.Button(fila3, text="🔄 Extremo Der", 
                                 command=lambda: robot_controller.enviar_movimiento_avanzado("right_extreme", 100, 120),
                                 font=("Arial", 7, "bold"), bg="#e74c3c", fg="white", width=10)
    btn_right_extreme.pack(side="right", padx=1)
    
    # Info
    info_frame = tk.Frame(control_frame, bg='#2c3e50', relief='raised', bd=2)
    info_frame.pack(fill="x", side="bottom", padx=10, pady=10)
    
    info_text = f"""💻 Python 3, OpenCV, Pillow, Tkinter, NumPy
📡 ESP32-CAM: {ESP32_CAM_IP}
🎯 Pipeline: 6 etapas + 8 maniobras
⚙️ Motores: A(Izq), B(Centro), C(Der)"""
    
    info_label = tk.Label(info_frame, text=info_text, 
                         font=("Arial", 8), bg='#2c3e50', fg='#bdc3c7',
                         justify='left', padx=10, pady=10)
    info_label.pack()
    
    # Eventos
    def al_cerrar():
        logger.info("Cerrando aplicación...")
        stop_stream_flag.set()
        robot_controller.parar_robot()
        time.sleep(1)
        root.quit()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", al_cerrar)
    
    def manejar_teclas(event):
        key = event.char.lower()
        if key == 'q':
            robot_controller.alternar_modo_automatico()
        elif key == 's':
            robot_controller.parar_robot()
        elif key == 'r':
            resetear_parametros()
    
    root.bind('<KeyPress>', manejar_teclas)
    root.focus_set()
    
    # Iniciar stream
    video_thread = threading.Thread(target=hilo_stream_video, daemon=True)
    video_thread.start()
    
    logger.info("=" * 80)
    logger.info("🤖 ROBOT SEGUIDOR ULTRA PARA CURVAS PRONUNCIADAS INICIADO")
    logger.info("=" * 80)
    logger.info(f"📡 ESP32-CAM: {ESP32_CAM_IP}")
    logger.info("🏁 8 tipos de maniobras para curvas complejas")
    logger.info("⚙️ Control tri-motor: A(Frontal-Izq), B(Trasero-Centro), C(Frontal-Der)")
    logger.info("🎮 Teclas: Q(Auto), S(Stop), R(Reset)")
    logger.info("=" * 80)
    
    root.mainloop()

if __name__ == "__main__":
    try:
        crear_interfaz()
    except KeyboardInterrupt:
        logger.info("⚠️ Aplicación terminada por usuario")
    except Exception as e:
        logger.error(f"❌ Error crítico: {e}")
        messagebox.showerror("Error", f"Error: {e}")
    finally:
        logger.info("👋 Aplicación terminada")