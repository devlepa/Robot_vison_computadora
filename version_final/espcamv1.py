"""
ROBOT SEGUIDOR DE LÍNEA ESP32-CAM - INTERFAZ VISUAL PROFESIONAL
===============================================================

Interfaz gráfica optimizada con:
- Vista de pipeline en grid 3x2 con imágenes grandes
- Diseño responsivo que ocupa toda la pantalla
- Visualización en tiempo real de todas las etapas
- Sliders organizados en pestañas compactas
- Estética profesional y moderna

Tecnologías: Python 3, OpenCV, Pillow, Tkinter, Numpy
"""

import cv2
import numpy as np
import requests
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import io
import json
import logging
from datetime import datetime
from collections import deque
import math
import socket

# ==========================================
# CONFIGURACIÓN Y CONSTANTES
# ==========================================

# CONFIGURAR AQUÍ LA IP DE TU ESP32-CAM
ESP32_CAM_IP = "192.168.196.182"

# URLs de la API
BASE_URL = f"http://{ESP32_CAM_IP}"
STREAM_URL = f"{BASE_URL}/stream"
AUTO_MODE_URL = f"{BASE_URL}/auto"
STOP_URL = f"{BASE_URL}/stop"
MOVE_URL = f"{BASE_URL}/move"
PARAMS_URL = f"{BASE_URL}/params"

# Configuración de timeouts
TIMEOUT_COMANDO = 10
TIMEOUT_STREAM = 15
TIMEOUT_PARAMS = 8

# Configuración de logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Parámetros DEFAULT optimizados
PARAMETROS_DEFAULT = {
    # Parámetros básicos de imagen
    'umbral_binario': 80,
    'altura_roi': 60,
    'zona_muerta_centro': 0.08,
    'velocidad_base': 180,
    'velocidad_giro': 140,
    
    # Parámetros Canny Edge Detection
    'canny_low': 50,
    'canny_high': 150,
    'canny_aperture': 3,
    
    # Parámetros Morfológicos
    'morph_kernel_size': 5,
    'opening_iterations': 2,
    'closing_iterations': 1,
    'erosion_iterations': 1,
    'dilation_iterations': 1,
    
    # Parámetros de Preprocessing
    'blur_kernel': 5,
    'contrast_alpha': 1.2,
    'brightness_beta': 10,
    
    # Parámetros Hough Transform
    'hough_threshold': 50,
    'min_line_length': 30,
    'max_line_gap': 10,
    
    # Parámetros PID
    'pid_kp': 0.8,
    'pid_ki': 0.1,
    'pid_kd': 0.3,
    
    # Parámetros de filtrado
    'area_min_threshold': 100,
    'area_max_threshold': 5000,
    'confidence_threshold': 0.3
}

class VisionProcessorAdvanced:
    """Procesador avanzado de visión con pipeline completo visualizable"""
    
    def __init__(self):
        self.parametros = PARAMETROS_DEFAULT.copy()
        
        # Variables de estado
        self.ultima_deteccion = {
            'linea_detectada': False,
            'posicion_linea': 0.0,
            'angulo_linea': 0.0,
            'centro_x': 0,
            'centro_y': 0,
            'confianza': 0.0,
            'area_detectada': 0,
            'puntos_linea': [],
            'num_contornos': 0
        }
        
        # Frames del pipeline para visualización
        self.pipeline_frames = {
            'original': None,
            'preprocessed': None,
            'binary': None,
            'opening': None,
            'closing': None,
            'edges': None,
            'final': None
        }
        
        # Historia para suavizado
        self.historia_posiciones = deque(maxlen=7)
        self.historia_angulos = deque(maxlen=5)
        
        # Control PID
        self.error_anterior = 0.0
        self.integral_error = 0.0
        self.max_integral = 20.0
        
        # Métricas de rendimiento
        self.fps_counter = 0
        self.tiempo_ultimo_fps = time.time()
        self.fps_actual = 0
    
    def actualizar_parametros(self, nuevos_params):
        """Actualiza parámetros de procesamiento"""
        self.parametros.update(nuevos_params)
        
    def calcular_fps(self):
        """Calcula FPS actual"""
        self.fps_counter += 1
        tiempo_actual = time.time()
        
        if tiempo_actual - self.tiempo_ultimo_fps >= 1.0:
            self.fps_actual = self.fps_counter
            self.fps_counter = 0
            self.tiempo_ultimo_fps = tiempo_actual
    
    def preprocessar_imagen(self, frame):
        """Preprocesamiento avanzado con ajustes de contraste y brillo"""
        # Convertir a escala de grises
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        else:
            gray = frame.copy()
        
        # Ajustar contraste y brillo
        alpha = max(0.5, min(3.0, self.parametros['contrast_alpha']))
        beta = max(-50, min(50, int(self.parametros['brightness_beta'])))
        gray = cv2.convertScaleAbs(gray, alpha=alpha, beta=beta)
        
        # Aplicar desenfoque gaussiano
        blur_size = max(1, int(self.parametros['blur_kernel']))
        if blur_size > 1 and blur_size % 2 == 0:
            blur_size += 1
        if blur_size > 1:
            gray = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
        
        # Ecualización adaptativa del histograma
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        
        return gray
    
    def binarizar_imagen(self, gray):
        """Binarización avanzada con múltiples métodos"""
        umbral = max(0, min(255, int(self.parametros['umbral_binario'])))
        
        # Método 1: Umbralización simple
        _, binary1 = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY_INV)
        
        # Método 2: Umbralización adaptativa (para comparación)
        binary2 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                       cv2.THRESH_BINARY_INV, 11, 2)
        
        # Combinar métodos (usar el mejor según condiciones)
        mean_intensity = np.mean(gray)
        if mean_intensity < 100:  # Imagen oscura
            binary = cv2.bitwise_or(binary1, binary2)
        else:  # Imagen clara
            binary = binary1
        
        return binary
    
    def aplicar_opening(self, binary):
        """Operación morfológica de apertura (erosión + dilatación)"""
        kernel_size = max(3, int(self.parametros['morph_kernel_size']))
        if kernel_size % 2 == 0:
            kernel_size += 1
            
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        
        # Erosión
        eroded = binary.copy()
        erosion_iter = max(0, int(self.parametros['erosion_iterations']))
        if erosion_iter > 0:
            eroded = cv2.erode(binary, kernel, iterations=erosion_iter)
        
        # Dilatación
        opening_iter = max(0, int(self.parametros['opening_iterations']))
        if opening_iter > 0:
            opened = cv2.dilate(eroded, kernel, iterations=opening_iter)
        else:
            opened = eroded
            
        return opened
    
    def aplicar_closing(self, opened):
        """Operación morfológica de cierre (dilatación + erosión)"""
        kernel_size = max(3, int(self.parametros['morph_kernel_size']))
        if kernel_size % 2 == 0:
            kernel_size += 1
            
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        
        # Dilatación
        dilated = opened.copy()
        dilation_iter = max(0, int(self.parametros['dilation_iterations']))
        if dilation_iter > 0:
            dilated = cv2.dilate(opened, kernel, iterations=dilation_iter)
        
        # Erosión (cierre)
        closing_iter = max(0, int(self.parametros['closing_iterations']))
        if closing_iter > 0:
            closed = cv2.erode(dilated, kernel, iterations=closing_iter)
        else:
            closed = dilated
            
        return closed
    
    def detectar_bordes_canny(self, processed):
        """Detección de bordes Canny optimizada"""
        low = max(10, int(self.parametros['canny_low']))
        high = max(low + 20, int(self.parametros['canny_high']))
        aperture = max(3, min(7, int(self.parametros['canny_aperture'])))
        if aperture % 2 == 0:
            aperture += 1
        
        edges = cv2.Canny(processed, low, high, apertureSize=aperture)
        return edges
    
    def extraer_roi(self, frame):
        """Extrae región de interés optimizada"""
        h, w = frame.shape[:2]
        
        roi_height = min(max(20, int(self.parametros['altura_roi'])), h//2)
        
        # ROI en la parte inferior con offset
        offset = 10
        y_start = h - roi_height - offset
        y_end = h - offset
        
        if y_start < 0:
            y_start = 0
            y_end = roi_height
        
        roi = frame[y_start:y_end, 0:w]
        return roi, (0, y_start, w, y_end)
    
    def analizar_contornos(self, binary_roi):
        """Análisis avanzado de contornos"""
        contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, 0.0, 0.0, []
        
        h, w = binary_roi.shape[:2]
        
        # Filtrar contornos por área
        contornos_validos = []
        area_min = self.parametros['area_min_threshold']
        area_max = self.parametros['area_max_threshold']
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area_min <= area <= area_max:
                # Calcular propiedades del contorno
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Calcular bounding box
                    x, y, w_box, h_box = cv2.boundingRect(contour)
                    aspect_ratio = w_box / h_box if h_box > 0 else 0
                    
                    # Calcular solidez
                    hull = cv2.convexHull(contour)
                    hull_area = cv2.contourArea(hull)
                    solidity = area / hull_area if hull_area > 0 else 0
                    
                    contornos_validos.append({
                        'contour': contour,
                        'area': area,
                        'centro': (cx, cy),
                        'aspect_ratio': aspect_ratio,
                        'solidity': solidity,
                        'bbox': (x, y, w_box, h_box)
                    })
        
        if not contornos_validos:
            return None, 0.0, 0.0, []
        
        # Seleccionar mejor contorno
        mejor_contorno = None
        mejor_score = -1
        
        for cont_info in contornos_validos:
            cx, cy = cont_info['centro']
            area = cont_info['area']
            aspect_ratio = cont_info['aspect_ratio']
            solidity = cont_info['solidity']
            
            # Calcular score basado en múltiples factores
            dist_centro = abs(cx - w/2) / (w/2)
            score_distancia = 1.0 - dist_centro
            
            score_area = min(area / 1000.0, 1.0)
            score_forma = min(aspect_ratio / 2.0, 1.0) if aspect_ratio > 1 else aspect_ratio
            score_solidez = solidity
            
            # Score total
            score_total = (score_distancia * 0.4 + score_area * 0.3 + 
                          score_forma * 0.2 + score_solidez * 0.1)
            
            if score_total > mejor_score:
                mejor_score = score_total
                mejor_contorno = cont_info
        
        if mejor_contorno:
            cx, cy = mejor_contorno['centro']
            posicion_normalizada = (cx - w/2) / (w/2)
            
            # Calcular ángulo aproximado del contorno
            angulo = 0.0
            if len(mejor_contorno['contour']) >= 5:
                ellipse = cv2.fitEllipse(mejor_contorno['contour'])
                angulo = ellipse[2]
            
            return (cx, cy), posicion_normalizada, angulo, [mejor_contorno]
        
        return None, 0.0, 0.0, []
    
    def calcular_confianza_avanzada(self, contornos_info, area_total, roi_shape):
        """Calcula confianza avanzada de la detección"""
        if not contornos_info:
            return 0.0
        
        h, w = roi_shape[:2]
        
        confianza = 0.0
        
        # Factor número de contornos válidos
        num_contornos = len(contornos_info)
        if num_contornos >= 1:
            confianza += 0.3
        if num_contornos >= 2:
            confianza += 0.1
        
        # Factor área total
        area_normalizada = area_total / (w * h)
        if 0.05 <= area_normalizada <= 0.25:
            confianza += 0.4
        elif 0.02 <= area_normalizada <= 0.4:
            confianza += 0.2
        
        # Factor mejor contorno
        if contornos_info:
            mejor = contornos_info[0]
            
            # Factor posición central
            cx, cy = mejor['centro']
            dist_centro = abs(cx - w/2) / (w/2)
            if dist_centro < 0.2:
                confianza += 0.2
            elif dist_centro < 0.4:
                confianza += 0.1
            
            # Factor forma
            if 1.5 <= mejor['aspect_ratio'] <= 4.0:
                confianza += 0.1
            
            # Factor solidez
            if mejor['solidity'] > 0.7:
                confianza += 0.1
        
        return min(confianza, 1.0)
    
    def suavizar_deteccion(self, posicion, angulo):
        """Suaviza la detección usando historia ponderada"""
        self.historia_posiciones.append(posicion)
        self.historia_angulos.append(angulo)
        
        # Suavizado de posición con pesos exponenciales
        if len(self.historia_posiciones) > 1:
            pesos = np.exp(np.linspace(-1, 0, len(self.historia_posiciones)))
            pesos /= np.sum(pesos)
            posicion_suave = np.average(list(self.historia_posiciones), weights=pesos)
        else:
            posicion_suave = posicion
        
        # Suavizado de ángulo
        if len(self.historia_angulos) > 1:
            angulo_suave = np.mean(list(self.historia_angulos))
        else:
            angulo_suave = angulo
        
        return posicion_suave, angulo_suave
    
    def procesar_frame_completo(self, frame):
        """Procesamiento completo del frame con pipeline visualizable"""
        try:
            self.calcular_fps()
            
            h_orig, w_orig = frame.shape[:2]
            
            # 1. Guardar frame original
            self.pipeline_frames['original'] = frame.copy()
            
            # 2. Preprocesamiento
            preprocessed = self.preprocessar_imagen(frame)
            self.pipeline_frames['preprocessed'] = preprocessed
            
            # 3. Extraer ROI
            roi_preprocessed, roi_coords = self.extraer_roi(preprocessed)
            
            # 4. Binarización
            binary_roi = self.binarizar_imagen(roi_preprocessed)
            self.pipeline_frames['binary'] = self.crear_frame_completo(binary_roi, roi_coords, h_orig, w_orig)
            
            # 5. Apertura morfológica
            opened_roi = self.aplicar_opening(binary_roi)
            self.pipeline_frames['opening'] = self.crear_frame_completo(opened_roi, roi_coords, h_orig, w_orig)
            
            # 6. Cierre morfológico
            closed_roi = self.aplicar_closing(opened_roi)
            self.pipeline_frames['closing'] = self.crear_frame_completo(closed_roi, roi_coords, h_orig, w_orig)
            
            # 7. Detección de bordes
            edges_roi = self.detectar_bordes_canny(closed_roi)
            self.pipeline_frames['edges'] = self.crear_frame_completo(edges_roi, roi_coords, h_orig, w_orig)
            
            # 8. Análisis de contornos
            centro, posicion, angulo, contornos_info = self.analizar_contornos(closed_roi)
            
            # 9. Calcular área total y confianza
            area_total = sum(info['area'] for info in contornos_info) if contornos_info else 0
            confianza = self.calcular_confianza_avanzada(contornos_info, area_total, closed_roi.shape)
            
            # Actualizar detección
            if centro is not None and confianza > self.parametros['confidence_threshold']:
                # Suavizar detección
                posicion_suave, angulo_suave = self.suavizar_deteccion(posicion, angulo)
                
                self.ultima_deteccion.update({
                    'linea_detectada': True,
                    'posicion_linea': posicion_suave,
                    'angulo_linea': angulo_suave,
                    'centro_x': int(centro[0]),
                    'centro_y': int(centro[1] + roi_coords[1]),
                    'confianza': confianza,
                    'area_detectada': int(area_total),
                    'num_contornos': len(contornos_info)
                })
            else:
                self.ultima_deteccion.update({
                    'linea_detectada': False,
                    'posicion_linea': 0.0,
                    'angulo_linea': 0.0,
                    'confianza': confianza,
                    'num_contornos': len(contornos_info) if contornos_info else 0
                })
            
            # 10. Crear frame de visualización final
            frame_final = self.crear_frame_visualizacion_completo(frame, roi_coords, contornos_info)
            self.pipeline_frames['final'] = frame_final
            
            return True
            
        except Exception as e:
            logger.error(f"Error procesando frame: {e}")
            # Crear frames vacíos en caso de error
            frame_error = np.zeros((h_orig, w_orig, 3), dtype=np.uint8)
            for key in self.pipeline_frames:
                self.pipeline_frames[key] = frame_error
            return False
    
    def crear_frame_completo(self, roi_frame, roi_coords, h_orig, w_orig):
        """Crea frame completo colocando ROI en posición original"""
        if len(roi_frame.shape) == 2:
            frame_completo = np.zeros((h_orig, w_orig), dtype=np.uint8)
        else:
            frame_completo = np.zeros((h_orig, w_orig, 3), dtype=np.uint8)
        
        x1, y1, x2, y2 = roi_coords
        frame_completo[y1:y2, x1:x2] = roi_frame
        
        return frame_completo
    
    def crear_frame_visualizacion_completo(self, frame_original, roi_coords, contornos_info):
        """Crea frame de visualización con todas las anotaciones"""
        if len(frame_original.shape) == 2:
            vis_frame = cv2.cvtColor(frame_original, cv2.COLOR_GRAY2RGB)
        else:
            vis_frame = frame_original.copy()
        
        # Dibujar ROI
        x1, y1, x2, y2 = roi_coords
        cv2.rectangle(vis_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.putText(vis_frame, "ROI", (x1+5, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Dibujar contornos detectados
        if contornos_info:
            for info in contornos_info:
                # Ajustar coordenadas del contorno a frame completo
                contour_adjusted = info['contour'].copy()
                contour_adjusted[:, :, 1] += y1  # Ajustar coordenada Y
                
                # Dibujar contorno
                cv2.drawContours(vis_frame, [contour_adjusted], -1, (255, 0, 255), 3)
                
                # Dibujar centro
                cx, cy = info['centro']
                cy_adjusted = cy + y1
                cv2.circle(vis_frame, (cx, cy_adjusted), 10, (255, 0, 0), -1)
                cv2.circle(vis_frame, (cx, cy_adjusted), 15, (0, 0, 255), 3)
                
                # Mostrar información del contorno
                cv2.putText(vis_frame, f"A:{int(info['area'])}", 
                           (cx-30, cy_adjusted-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Información en pantalla con fondo
        info_bg = np.zeros((120, 250, 3), dtype=np.uint8)
        info_bg[:] = (0, 0, 0)  # Fondo negro
        
        # Agregar información
        info_y = 25
        cv2.putText(info_bg, f"FPS: {self.fps_actual}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        info_y += 30
        cv2.putText(info_bg, f"Pos: {self.ultima_deteccion['posicion_linea']:.3f}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        info_y += 30
        cv2.putText(info_bg, f"Conf: {self.ultima_deteccion['confianza']:.2f}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Superponer información en el frame
        vis_frame[10:130, 10:260] = info_bg
        
        return vis_frame
    
    def obtener_frame_pipeline(self, etapa):
        """Obtiene frame de una etapa específica del pipeline"""
        return self.pipeline_frames.get(etapa, np.zeros((240, 320, 3), dtype=np.uint8))
    
    def calcular_comando_movimiento(self):
        """Calcula comando de movimiento usando PID avanzado"""
        if not self.ultima_deteccion['linea_detectada']:
            return "stop", 0
        
        error = self.ultima_deteccion['posicion_linea']
        confianza = self.ultima_deteccion['confianza']
        
        # Zona muerta
        if abs(error) < self.parametros['zona_muerta_centro']:
            return "forward", int(self.parametros['velocidad_base'])
        
        # Parámetros PID ajustados por confianza
        kp = self.parametros['pid_kp'] * confianza
        ki = self.parametros['pid_ki'] * confianza  
        kd = self.parametros['pid_kd'] * confianza
        
        # Cálculo PID
        self.integral_error += error * ki
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        
        derivativo = kd * (error - self.error_anterior)
        self.error_anterior = error
        
        # Factor de reducción adaptativo
        factor_reduccion = 1.0 - min(abs(error) * 0.5, 0.7)
        
        velocidad_base = int(self.parametros['velocidad_base'] * factor_reduccion)
        velocidad_giro = int(self.parametros['velocidad_giro'] * (0.6 + 0.4 * factor_reduccion))
        
        # Decidir movimiento
        if abs(error) < 0.15:
            return "forward", velocidad_base
        elif error < 0:
            return "left", velocidad_giro
        else:
            return "right", velocidad_giro

class RobotControllerAdvanced:
    """Controlador avanzado del robot"""
    
    def __init__(self):
        self.conectado = False
        self.modo_automatico = False
        self.vision_processor = VisionProcessorAdvanced()
        
        # Session HTTP optimizada
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'RobotControllerAdvanced/2.0',
            'Connection': 'keep-alive'
        })
    
    def enviar_comando_http(self, url, datos=None, timeout=None):
        """Envío HTTP optimizado"""
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
            return None
    
    def enviar_movimiento(self, direccion, velocidad=None):
        """Envía comando de movimiento"""
        if self.modo_automatico and direccion != "stop":
            return False
        
        if velocidad is None:
            if direccion in ['left', 'right']:
                velocidad = int(self.vision_processor.parametros['velocidad_giro'])
            else:
                velocidad = int(self.vision_processor.parametros['velocidad_base'])
        
        datos = {"direction": direccion, "speed": velocidad}
        resultado = self.enviar_comando_http(MOVE_URL, datos)
        
        return resultado is not None
    
    def alternar_modo_automatico(self):
        """Alternar modo automático"""
        nuevo_modo = not self.modo_automatico
        datos = {"auto": nuevo_modo}
        
        resultado = self.enviar_comando_http(AUTO_MODE_URL, datos)
        
        if resultado:
            self.modo_automatico = nuevo_modo
            return True
        return False
    
    def parar_robot(self):
        """Parada de emergencia"""
        resultado = self.enviar_comando_http(STOP_URL, {})
        if resultado:
            self.modo_automatico = False
            return True
        return False
    
    def control_automatico(self):
        """Lógica de control automático"""
        if not self.modo_automatico:
            return
        
        comando, velocidad = self.vision_processor.calcular_comando_movimiento()
        
        if comando and comando != "stop":
            self.enviar_movimiento(comando, velocidad)
        elif not self.vision_processor.ultima_deteccion['linea_detectada']:
            self.enviar_movimiento("stop")

# Variables globales
root = None
robot_controller = RobotControllerAdvanced()
stop_stream_flag = threading.Event()
parametros_actuales = PARAMETROS_DEFAULT.copy()

# Variables de la interfaz
estado_label = None
modo_label = None 
linea_label = None
auto_button = None
sliders = {}

# Variables para visualización múltiple
pipeline_labels = {}

def hilo_stream_video():
    """Hilo optimizado para stream con procesamiento completo"""
    logger.info("🎥 Iniciando stream con pipeline completo...")
    
    errores_consecutivos = 0
    max_errores = 5
    
    while not stop_stream_flag.is_set():
        try:
            with requests.get(STREAM_URL, stream=True, timeout=TIMEOUT_STREAM) as r:
                r.raise_for_status()
                robot_controller.conectado = True
                errores_consecutivos = 0
                
                bytes_data = b''
                
                for chunk in r.iter_content(chunk_size=8192):
                    if stop_stream_flag.is_set():
                        break
                    
                    bytes_data += chunk
                    
                    inicio = bytes_data.find(b'\xff\xd8')
                    fin = bytes_data.find(b'\xff\xd9')
                    
                    if inicio != -1 and fin != -1 and fin > inicio:
                        jpg = bytes_data[inicio:fin+2]
                        bytes_data = bytes_data[fin+2:]
                        
                        try:
                            # Procesar imagen
                            img = Image.open(io.BytesIO(jpg))
                            frame_np = np.array(img)
                            
                            # Actualizar parámetros
                            robot_controller.vision_processor.actualizar_parametros(parametros_actuales)
                            
                            # Procesamiento completo
                            if robot_controller.vision_processor.procesar_frame_completo(frame_np):
                                # Mostrar pipeline completo
                                mostrar_pipeline_completo()
                                
                                # Control automático
                                if robot_controller.modo_automatico:
                                    robot_controller.control_automatico()
                                
                                # Actualizar interfaz
                                actualizar_estado_deteccion()
                            
                        except Exception as e:
                            logger.error(f"❌ Error procesando frame: {e}")
                
        except Exception as e:
            errores_consecutivos += 1
            robot_controller.conectado = False
            
            if errores_consecutivos >= max_errores:
                logger.error(f"❌ Stream perdido tras {errores_consecutivos} intentos")
                break
            
            time.sleep(min(2 ** errores_consecutivos, 30))
    
    logger.info("🔚 Hilo de stream terminado")

def mostrar_pipeline_completo():
    """Muestra el pipeline completo de procesamiento en grid 3x2"""
    try:
        # Lista de etapas a mostrar
        etapas = ['original', 'binary', 'opening', 'closing', 'edges', 'final']
        
        for etapa in etapas:
            if etapa in pipeline_labels and pipeline_labels[etapa].winfo_exists():
                frame = robot_controller.vision_processor.obtener_frame_pipeline(etapa)
                
                if frame is not None and frame.size > 0:
                    # Redimensionar a tamaño grande y visible
                    h, w = frame.shape[:2]
                    ancho_objetivo = 380  # Tamaño más grande
                    alto_objetivo = int(h * (ancho_objetivo / w))
                    
                    frame_resized = cv2.resize(frame, (ancho_objetivo, alto_objetivo))
                    
                    # Convertir a RGB si es necesario
                    if len(frame_resized.shape) == 2:
                        frame_resized = cv2.cvtColor(frame_resized, cv2.COLOR_GRAY2RGB)
                    
                    # Convertir a PIL y mostrar
                    img_pil = Image.fromarray(frame_resized)
                    foto = ImageTk.PhotoImage(image=img_pil)
                    
                    pipeline_labels[etapa].config(image=foto)
                    pipeline_labels[etapa].image = foto
        
    except Exception as e:
        logger.error(f"❌ Error mostrando pipeline: {e}")

def actualizar_estado_deteccion():
    """Actualizar estado de detección"""
    try:
        if linea_label and linea_label.winfo_exists():
            det = robot_controller.vision_processor.ultima_deteccion
            fps = robot_controller.vision_processor.fps_actual
            
            if det['linea_detectada']:
                texto = f"✅ Línea Detectada | Pos: {det['posicion_linea']:.3f} | Conf: {det['confianza']:.2f} | FPS: {fps} | Contornos: {det['num_contornos']}"
                color = "#2ecc71"  # Verde
            else:
                texto = f"❌ Sin Línea | Conf: {det['confianza']:.2f} | FPS: {fps} | Contornos: {det['num_contornos']}"
                color = "#e74c3c"  # Rojo
            
            linea_label.config(text=texto, background=color)
    except:
        pass

def callback_slider_parametros(param_name):
    """Callback para sliders"""
    def callback(valor):
        try:
            # Actualizar parámetro
            if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro',
                             'canny_low', 'canny_high', 'canny_aperture', 'morph_kernel_size',
                             'opening_iterations', 'closing_iterations', 'erosion_iterations',
                             'dilation_iterations', 'blur_kernel', 'brightness_beta',
                             'hough_threshold', 'min_line_length', 'max_line_gap',
                             'area_min_threshold', 'area_max_threshold']:
                parametros_actuales[param_name] = int(float(valor))
            else:
                parametros_actuales[param_name] = float(valor)
            
            # Actualizar etiqueta
            if param_name in sliders:
                slider_info = sliders[param_name]
                if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro',
                                 'canny_low', 'canny_high', 'canny_aperture', 'morph_kernel_size',
                                 'opening_iterations', 'closing_iterations', 'erosion_iterations',
                                 'dilation_iterations', 'blur_kernel', 'brightness_beta',
                                 'hough_threshold', 'min_line_length', 'max_line_gap',
                                 'area_min_threshold', 'area_max_threshold']:
                    slider_info['valor_label'].config(text=str(int(float(valor))))
                else:
                    slider_info['valor_label'].config(text=f"{float(valor):.3f}")
            
        except Exception as e:
            logger.error(f"❌ Error callback slider {param_name}: {e}")
    
    return callback

def resetear_parametros():
    """Resetea parámetros a valores por defecto"""
    global parametros_actuales
    
    parametros_actuales = PARAMETROS_DEFAULT.copy()
    
    for param_name, slider_info in sliders.items():
        if param_name in parametros_actuales:
            slider_info['slider'].set(parametros_actuales[param_name])
            
            if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro',
                             'canny_low', 'canny_high', 'canny_aperture', 'morph_kernel_size',
                             'opening_iterations', 'closing_iterations', 'erosion_iterations',
                             'dilation_iterations', 'blur_kernel', 'brightness_beta',
                             'hough_threshold', 'min_line_length', 'max_line_gap',
                             'area_min_threshold', 'area_max_threshold']:
                slider_info['valor_label'].config(text=str(parametros_actuales[param_name]))
            else:
                slider_info['valor_label'].config(text=f"{parametros_actuales[param_name]:.3f}")
    
    messagebox.showinfo("Reset Completo", "✅ Todos los parámetros han sido reseteados")

def crear_slider_compacto(parent, param_name, config):
    """Crea un slider compacto horizontal"""
    # Frame principal del slider
    slider_frame = tk.Frame(parent, bg='#34495e', relief='raised', bd=1)
    slider_frame.pack(fill="x", padx=2, pady=1)
    
    # Etiqueta del parámetro (más compacta)
    label = tk.Label(slider_frame, text=config['label'], 
                    font=("Arial", 8, "bold"), bg='#34495e', fg='white', anchor='w')
    label.pack(side="top", fill="x", padx=5, pady=1)
    
    # Frame para slider y valor
    control_frame = tk.Frame(slider_frame, bg='#34495e')
    control_frame.pack(fill="x", padx=5, pady=2)
    
    # Slider horizontal compacto
    slider = tk.Scale(control_frame, from_=config['min'], to=config['max'], 
                     resolution=config['step'], orient="horizontal", 
                     command=callback_slider_parametros(param_name),
                     bg='#2c3e50', fg='white', highlightthickness=0, 
                     troughcolor='#7f8c8d', activebackground='#3498db',
                     length=200, width=15)
    slider.set(parametros_actuales[param_name])
    slider.pack(side="left", fill="x", expand=True)
    
    # Etiqueta de valor actual
    if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro',
                     'canny_low', 'canny_high', 'canny_aperture', 'morph_kernel_size',
                     'opening_iterations', 'closing_iterations', 'erosion_iterations',
                     'dilation_iterations', 'blur_kernel', 'brightness_beta',
                     'hough_threshold', 'min_line_length', 'max_line_gap',
                     'area_min_threshold', 'area_max_threshold']:
        valor_texto = str(parametros_actuales[param_name])
    else:
        valor_texto = f"{parametros_actuales[param_name]:.3f}"
        
    valor_label = tk.Label(control_frame, text=valor_texto, 
                          font=("Arial", 9, "bold"), bg='#e74c3c', fg='white', 
                          width=8, relief='raised', bd=1)
    valor_label.pack(side="right", padx=(5, 0))
    
    # Guardar referencia
    sliders[param_name] = {'slider': slider, 'valor_label': valor_label}

def crear_interfaz_visual_profesional():
    """Crear interfaz gráfica visual profesional que ocupa toda la pantalla"""
    global root, estado_label, modo_label, linea_label, auto_button, pipeline_labels
    
    root = tk.Tk()
    root.title("🤖 Robot Seguidor - Visualización Profesional del Pipeline")
    root.state('zoomed')  # Maximizar ventana en Windows
    # Para Linux/Mac: root.attributes('-zoomed', True)
    root.configure(bg='#1a1a1a')
    
    # Configurar grid weights para redimensionamiento
    root.grid_rowconfigure(1, weight=1)
    root.grid_columnconfigure(0, weight=1)
    
    # ==========================================
    # BARRA SUPERIOR DE ESTADO
    # ==========================================
    header_frame = tk.Frame(root, bg='#2c3e50', height=80, relief='raised', bd=2)
    header_frame.pack(fill="x", padx=5, pady=5)
    header_frame.pack_propagate(False)
    
    # Título principal
    titulo_label = tk.Label(header_frame, 
                           text="🤖 ROBOT SEGUIDOR DE LÍNEA - PIPELINE PROFESIONAL OpenCV", 
                           font=("Arial", 16, "bold"), bg='#2c3e50', fg='#ecf0f1')
    titulo_label.pack(pady=5)
    
    # Estado del sistema
    estado_frame = tk.Frame(header_frame, bg='#2c3e50')
    estado_frame.pack(fill="x", padx=20)
    
    estado_label = tk.Label(estado_frame, text="🔴 Estado: Conectando...", 
                           font=("Arial", 12, "bold"), bg='#34495e', fg='white', 
                           relief='raised', bd=2, padx=10, pady=5)
    estado_label.pack(side="left", padx=5)
    
    modo_label = tk.Label(estado_frame, text="📱 Modo: Manual", 
                         font=("Arial", 12, "bold"), bg='#34495e', fg='white', 
                         relief='raised', bd=2, padx=10, pady=5)
    modo_label.pack(side="left", padx=5)
    
    linea_label = tk.Label(estado_frame, text="🎯 Iniciando detección...", 
                          font=("Arial", 12, "bold"), bg='#f39c12', fg='white', 
                          relief='raised', bd=2, padx=10, pady=5)
    linea_label.pack(side="right", padx=5)
    
    # ==========================================
    # CONTENEDOR PRINCIPAL DIVIDIDO
    # ==========================================
    main_container = tk.Frame(root, bg='#1a1a1a')
    main_container.pack(fill="both", expand=True, padx=5)
    
    # ==========================================
    # LADO IZQUIERDO: GRID DE IMÁGENES 3x2
    # ==========================================
    images_frame = tk.Frame(main_container, bg='#2c3e50', relief='raised', bd=3)
    images_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
    
    # Título del pipeline
    pipeline_titulo = tk.Label(images_frame, 
                              text="🔍 PIPELINE DE PROCESAMIENTO EN TIEMPO REAL", 
                              font=("Arial", 14, "bold"), bg='#2c3e50', fg='#ecf0f1')
    pipeline_titulo.pack(pady=10)
    
    # Configurar grid 3x2 para las imágenes
    grid_frame = tk.Frame(images_frame, bg='#2c3e50')
    grid_frame.pack(fill="both", expand=True, padx=10, pady=10)
    
    # Configurar pesos del grid
    for i in range(2):  # 2 filas
        grid_frame.grid_rowconfigure(i, weight=1)
    for j in range(3):  # 3 columnas
        grid_frame.grid_columnconfigure(j, weight=1)
    
    # Definir etapas del pipeline con colores
    etapas_pipeline = [
        ('original', '📹 ORIGINAL', '#3498db'),
        ('binary', '⚫ BINARIZACIÓN', '#9b59b6'),
        ('opening', '🔵 APERTURA', '#2ecc71'),
        ('closing', '🔴 CIERRE', '#e74c3c'),
        ('edges', '⚡ BORDES CANNY', '#f39c12'),
        ('final', '🎯 RESULTADO FINAL', '#1abc9c')
    ]
    
    # Crear grid de imágenes 3x2
    for idx, (etapa, titulo, color) in enumerate(etapas_pipeline):
        fila = idx // 3
        columna = idx % 3
        
        # Frame para cada etapa
        etapa_frame = tk.Frame(grid_frame, bg=color, relief='raised', bd=3)
        etapa_frame.grid(row=fila, column=columna, padx=5, pady=5, sticky="nsew")
        
        # Título de la etapa
        etapa_titulo = tk.Label(etapa_frame, text=titulo, 
                               font=("Arial", 11, "bold"), bg=color, fg='white')
        etapa_titulo.pack(pady=5)
        
        # Label para la imagen
        etapa_label = tk.Label(etapa_frame, bg="black", fg="white", 
                              text=f"{titulo}\n\nProcesando...\nAjusta los sliders\npara ver cambios", 
                              font=("Arial", 10), relief='sunken', bd=2)
        etapa_label.pack(fill="both", expand=True, padx=5, pady=(0, 5))
        
        pipeline_labels[etapa] = etapa_label
    
    # ==========================================
    # LADO DERECHO: CONTROLES Y SLIDERS
    # ==========================================
    control_frame = tk.Frame(main_container, bg='#34495e', width=350, relief='raised', bd=3)
    control_frame.pack(side="right", fill="y", padx=(5, 0))
    control_frame.pack_propagate(False)
    
    # Título de controles
    control_titulo = tk.Label(control_frame, 
                             text="🎛️ CONTROLES Y PARÁMETROS", 
                             font=("Arial", 12, "bold"), bg='#34495e', fg='white')
    control_titulo.pack(pady=10)
    
    # ==========================================
    # BOTONES DE CONTROL PRINCIPAL
    # ==========================================
    botones_frame = tk.Frame(control_frame, bg='#34495e')
    botones_frame.pack(fill="x", padx=10, pady=10)
    
    auto_button = tk.Button(botones_frame, text="🟢 ACTIVAR IA", 
                           command=robot_controller.alternar_modo_automatico,
                           font=("Arial", 11, "bold"), bg="#27ae60", fg="white",
                           relief='raised', bd=3, height=2)
    auto_button.pack(fill="x", pady=2)
    
    stop_button = tk.Button(botones_frame, text="🔴 PARADA EMERGENCIA", 
                           command=robot_controller.parar_robot,
                           font=("Arial", 11, "bold"), bg="#e74c3c", fg="white",
                           relief='raised', bd=3, height=2)
    stop_button.pack(fill="x", pady=2)
    
    reset_button = tk.Button(botones_frame, text="🔄 RESET PARÁMETROS", 
                            command=resetear_parametros,
                            font=("Arial", 11, "bold"), bg="#f39c12", fg="white",
                            relief='raised', bd=3, height=2)
    reset_button.pack(fill="x", pady=2)
    
    # ==========================================
    # NOTEBOOK CON PESTAÑAS DE PARÁMETROS
    # ==========================================
    notebook = ttk.Notebook(control_frame)
    notebook.pack(fill="both", expand=True, padx=10, pady=10)
    
    # Configurar estilo del notebook
    style = ttk.Style()
    style.configure('TNotebook.Tab', padding=[8, 4])
    
    # PESTAÑA 1: Parámetros Básicos
    tab1 = tk.Frame(notebook, bg='#34495e')
    notebook.add(tab1, text="🔧 Básicos")
    
    params_basicos = {
        'umbral_binario': {'label': 'Umbral Binarización (0-255)', 'min': 0, 'max': 255, 'step': 1},
        'altura_roi': {'label': 'Altura ROI (20-120px)', 'min': 20, 'max': 120, 'step': 5},
        'zona_muerta_centro': {'label': 'Zona Muerta (0.02-0.3)', 'min': 0.02, 'max': 0.3, 'step': 0.01},
        'velocidad_base': {'label': 'Velocidad Base (100-255)', 'min': 100, 'max': 255, 'step': 5},
        'velocidad_giro': {'label': 'Velocidad Giro (80-200)', 'min': 80, 'max': 200, 'step': 5}
    }
    
    for param_name, config in params_basicos.items():
        crear_slider_compacto(tab1, param_name, config)
    
    # PESTAÑA 2: Canny y Bordes
    tab2 = tk.Frame(notebook, bg='#34495e')
    notebook.add(tab2, text="⚡ Canny")
    
    params_canny = {
        'canny_low': {'label': 'Canny Umbral Bajo (10-150)', 'min': 10, 'max': 150, 'step': 5},
        'canny_high': {'label': 'Canny Umbral Alto (50-300)', 'min': 50, 'max': 300, 'step': 5},
        'canny_aperture': {'label': 'Canny Apertura (3-7)', 'min': 3, 'max': 7, 'step': 2}
    }
    
    for param_name, config in params_canny.items():
        crear_slider_compacto(tab2, param_name, config)
    
    # PESTAÑA 3: Morfología
    tab3 = tk.Frame(notebook, bg='#34495e')
    notebook.add(tab3, text="🔵🔴 Morfología")
    
    params_morph = {
        'morph_kernel_size': {'label': 'Tamaño Kernel (3-15)', 'min': 3, 'max': 15, 'step': 2},
        'erosion_iterations': {'label': 'Erosión (0-5)', 'min': 0, 'max': 5, 'step': 1},
        'dilation_iterations': {'label': 'Dilatación (0-5)', 'min': 0, 'max': 5, 'step': 1},
        'opening_iterations': {'label': 'Apertura (0-5)', 'min': 0, 'max': 5, 'step': 1},
        'closing_iterations': {'label': 'Cierre (0-5)', 'min': 0, 'max': 5, 'step': 1}
    }
    
    for param_name, config in params_morph.items():
        crear_slider_compacto(tab3, param_name, config)
    
    # PESTAÑA 4: Preprocesamiento
    tab4 = tk.Frame(notebook, bg='#34495e')
    notebook.add(tab4, text="🎨 Preproc.")
    
    params_preproc = {
        'blur_kernel': {'label': 'Desenfoque (1-15)', 'min': 1, 'max': 15, 'step': 2},
        'contrast_alpha': {'label': 'Contraste (0.5-3.0)', 'min': 0.5, 'max': 3.0, 'step': 0.1},
        'brightness_beta': {'label': 'Brillo (-50 a 50)', 'min': -50, 'max': 50, 'step': 5}
    }
    
    for param_name, config in params_preproc.items():
        crear_slider_compacto(tab4, param_name, config)
    
    # PESTAÑA 5: Filtros Avanzados
    tab5 = tk.Frame(notebook, bg='#34495e')
    notebook.add(tab5, text="📐 Filtros")
    
    params_filtros = {
        'hough_threshold': {'label': 'Umbral Hough (20-100)', 'min': 20, 'max': 100, 'step': 5},
        'min_line_length': {'label': 'Long. Mín (10-80)', 'min': 10, 'max': 80, 'step': 5},
        'area_min_threshold': {'label': 'Área Mín (50-500)', 'min': 50, 'max': 500, 'step': 10},
        'area_max_threshold': {'label': 'Área Máx (1000-10000)', 'min': 1000, 'max': 10000, 'step': 100},
        'confidence_threshold': {'label': 'Confianza (0.1-0.8)', 'min': 0.1, 'max': 0.8, 'step': 0.05}
    }
    
    for param_name, config in params_filtros.items():
        crear_slider_compacto(tab5, param_name, config)
    
    # PESTAÑA 6: Control PID
    tab6 = tk.Frame(notebook, bg='#34495e')
    notebook.add(tab6, text="🎯 PID")
    
    params_pid = {
        'pid_kp': {'label': 'Proporcional Kp (0.1-2.0)', 'min': 0.1, 'max': 2.0, 'step': 0.1},
        'pid_ki': {'label': 'Integral Ki (0.0-0.5)', 'min': 0.0, 'max': 0.5, 'step': 0.01},
        'pid_kd': {'label': 'Derivativo Kd (0.0-1.0)', 'min': 0.0, 'max': 1.0, 'step': 0.05}
    }
    
    for param_name, config in params_pid.items():
        crear_slider_compacto(tab6, param_name, config)
    
    # ==========================================
    # INFORMACIÓN TÉCNICA EN LA PARTE INFERIOR
    # ==========================================
    info_frame = tk.Frame(control_frame, bg='#2c3e50', relief='raised', bd=2)
    info_frame.pack(fill="x", side="bottom", padx=10, pady=10)
    
    info_text = f"""💻 TECNOLOGÍAS: Python 3, OpenCV, Pillow, Tkinter, NumPy
📡 ESP32-CAM: {ESP32_CAM_IP}
🎯 Pipeline: 6 etapas visualizables
🔧 Parámetros: 24 ajustables en tiempo real"""
    
    info_label = tk.Label(info_frame, text=info_text, 
                         font=("Arial", 9), bg='#2c3e50', fg='#bdc3c7',
                         justify='left', padx=10, pady=10)
    info_label.pack()
    
    # ==========================================
    # CONFIGURACIÓN FINAL
    # ==========================================
    
    # Protocolo de cierre
    def al_cerrar():
        logger.info("🔚 Cerrando aplicación...")
        stop_stream_flag.set()
        robot_controller.parar_robot()
        time.sleep(1)
        root.quit()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", al_cerrar)
    
    # Iniciar stream
    video_thread = threading.Thread(target=hilo_stream_video, daemon=True)
    video_thread.start()
    
    logger.info("=" * 100)
    logger.info("🤖 INTERFAZ VISUAL PROFESIONAL INICIADA")
    logger.info("=" * 100)
    logger.info(f"📡 ESP32-CAM: {ESP32_CAM_IP}")
    logger.info("🎥 Grid 3x2 con imágenes grandes y visibles")
    logger.info("🎛️ 24 sliders organizados en 6 pestañas")
    logger.info("💻 Interfaz maximizada ocupando toda la pantalla")
    logger.info("⚡ Actualización en tiempo real del pipeline")
    logger.info("=" * 100)
    
    root.mainloop()

if __name__ == "__main__":
    try:
        crear_interfaz_visual_profesional()
    except KeyboardInterrupt:
        logger.info("⚠️ Aplicación terminada por usuario")
    except Exception as e:
        logger.error(f"❌ Error crítico: {e}")
        messagebox.showerror("Error Crítico", f"Error: {e}")
    finally:
        logger.info("👋 Aplicación terminada")