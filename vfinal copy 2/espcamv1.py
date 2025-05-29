"""
ROBOT SEGUIDOR DE LÍNEA ESP32-CAM - VERSIÓN CORREGIDA
====================================================

CORRECCIONES IMPLEMENTADAS:
- Rotación de imagen 180 grados para corregir orientación
- Filtrado avanzado para eliminar objetos blancos no deseados
- Detección de línea más robusta con filtros morfológicos
- Selección inteligente del contorno de línea principal
- Eliminación de ruido y objetos pequeños
"""

import cv2
import numpy as np
import requests
import time
from PIL import Image, ImageTk
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import io
import json
from datetime import datetime

# ==========================================
# CONFIGURACIÓN DE CONEXIÓN
# ==========================================

# CAMBIAR ESTA IP POR LA DE TU ESP32-CAM
ESP32_CAM_IP = "192.168.196.182"

# URLs de la API
BASE_URL = f"http://{ESP32_CAM_IP}"
STREAM_URL = f"{BASE_URL}/stream"
AUTO_MODE_URL = f"{BASE_URL}/auto"
STOP_URL = f"{BASE_URL}/stop"
MOVE_URL = f"{BASE_URL}/move"
PARAMS_URL = f"{BASE_URL}/params"

# Timeouts optimizados
TIMEOUT_COMANDO = 6
TIMEOUT_STREAM = 10
TIMEOUT_PARAMS = 4

# Parámetros de visión
PARAMETROS_DEFAULT = {
    'umbral_binario': 80,
    'altura_roi': 60,
    'zona_muerta_centro': 0.08,
    'velocidad_base': 160,
    'velocidad_giro': 120,
}

# ==========================================
# VARIABLES GLOBALES
# ==========================================

# Referencias a widgets
root = None
estado_label = None
modo_label = None
linea_label = None
video_original_label = None
video_binario_label = None
auto_button = None
control_buttons = []
sliders_frame = None
sliders = {}

# Control de hilos y estado
stop_stream_flag = threading.Event()
parametros_actuales = PARAMETROS_DEFAULT.copy()

# Variables de detección mejoradas
ultima_deteccion = {
    'linea_detectada': False,
    'posicion_linea': 0.0,
    'centro_x': 0,
    'centro_y': 0,
    'pixels_detectados': 0,
    'area_contorno': 0,
    'confianza': 0.0
}

class RobotController:
    """Controlador del robot con comunicación optimizada"""
    
    def __init__(self):
        self.conectado = False
        self.modo_automatico = False
        self.ultimo_comando = time.time()
        self.intentos_reconexion = 0
        
    def enviar_comando_http(self, url, datos=None, timeout=TIMEOUT_COMANDO):
        """Envío HTTP con reintentos"""
        for intento in range(3):
            try:
                if datos:
                    headers = {'Content-Type': 'application/json'}
                    response = requests.post(url, json=datos, headers=headers, timeout=timeout)
                else:
                    response = requests.get(url, timeout=timeout)
                
                response.raise_for_status()
                self.conectado = True
                
                try:
                    return response.json()
                except:
                    return {"status": "ok", "data": response.text}
                    
            except requests.exceptions.ConnectionError:
                self.conectado = False
                if intento == 2:  # Último intento
                    self.mostrar_error_conexion()
                else:
                    time.sleep(1)
                    
            except requests.exceptions.Timeout:
                if intento == 2:
                    messagebox.showwarning("Timeout", f"ESP32-CAM no responde en {timeout}s")
                    
            except Exception as e:
                print(f"[ERROR] {url}: {e}")
                break
        
        return None
    
    def mostrar_error_conexion(self):
        """Muestra error de conexión detallado"""
        mensaje = f"""Error de Conexión con ESP32-CAM

IP configurada: {ESP32_CAM_IP}

Verificar:
✓ ESP32-CAM encendido y conectado
✓ IP correcta (Monitor Serie Arduino)
✓ Misma red WiFi
✓ Puerto 80 no bloqueado

Reinicia el ESP32-CAM si persiste el error."""
        
        messagebox.showerror("Error de Conexión", mensaje)
    
    def enviar_movimiento(self, direccion, velocidad=None):
        """Envía comando de movimiento"""
        if self.modo_automatico and direccion != "stop":
            messagebox.showinfo("Modo Automático", "Robot en modo automático.\nUse STOP o desactive modo automático.")
            return False
        
        if velocidad is None:
            if direccion in ['left', 'right', 'rotate_left', 'rotate_right']:
                velocidad = parametros_actuales['velocidad_giro']
            else:
                velocidad = parametros_actuales['velocidad_base']
        
        datos = {"direction": direccion, "speed": velocidad}
        resultado = self.enviar_comando_http(MOVE_URL, datos)
        
        if resultado:
            self.ultimo_comando = time.time()
            actualizar_estado_labels(f"Comando: {direccion}", 
                                   "Manual" if not self.modo_automatico else "Automático")
            return True
        return False
    
    def alternar_modo_automatico(self):
        """Cambia entre modo manual y automático"""
        nuevo_modo = not self.modo_automatico
        datos = {"auto": nuevo_modo}
        
        resultado = self.enviar_comando_http(AUTO_MODE_URL, datos)
        
        if resultado:
            self.modo_automatico = nuevo_modo
            actualizar_interfaz_modo()
            
            modo_texto = "Automático" if self.modo_automatico else "Manual"
            actualizar_estado_labels("Modo cambiado", modo_texto)
            
            print(f"[MODO] {modo_texto}")
            return True
        return False
    
    def parar_robot(self):
        """Parada de emergencia"""
        resultado = self.enviar_comando_http(STOP_URL, {})
        
        if resultado:
            self.modo_automatico = False
            actualizar_interfaz_modo()
            actualizar_estado_labels("DETENIDO", "Manual")
            print("[STOP] Robot detenido")
            return True
        return False
    
    def enviar_parametros(self, params):
        """Envía parámetros de visión al ESP32"""
        datos = {
            "umbral": int(params['umbral_binario']),
            "altura_roi": int(params['altura_roi']),
            "zona_muerta": float(params['zona_muerta_centro']),
            "velocidad_base": int(params['velocidad_base']),
            "velocidad_giro": int(params['velocidad_giro'])
        }
        
        resultado = self.enviar_comando_http(PARAMS_URL, datos, TIMEOUT_PARAMS)
        
        if resultado:
            print(f"[PARAMS] Actualizados: {datos}")
            return True
        return False

# Instancia global del controlador
robot = RobotController()

def aplicar_filtros_morfologicos(imagen_binaria):
    """Aplica filtros morfológicos para limpiar la imagen"""
    # Crear kernels morfológicos
    kernel_small = np.ones((3,3), np.uint8)
    kernel_medium = np.ones((5,5), np.uint8)
    
    # 1. Operación de apertura (erosión + dilatación) para eliminar ruido pequeño
    imagen_limpia = cv2.morphologyEx(imagen_binaria, cv2.MORPH_OPEN, kernel_small)
    
    # 2. Operación de cierre (dilatación + erosión) para cerrar huecos en líneas
    imagen_limpia = cv2.morphologyEx(imagen_limpia, cv2.MORPH_CLOSE, kernel_medium)
    
    # 3. Eliminación de componentes pequeños
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(imagen_limpia, connectivity=8)
    
    # Crear imagen de salida
    imagen_filtrada = np.zeros_like(imagen_limpia)
    
    for i in range(1, num_labels):  # Saltar el fondo (label 0)
        area = stats[i, cv2.CC_STAT_AREA]
        # Solo mantener componentes con área mínima (filtrar ruido)
        if area > 100:  # Área mínima para ser considerado línea
            imagen_filtrada[labels == i] = 255
    
    return imagen_filtrada

def seleccionar_contorno_linea_principal(contornos, ancho_imagen):
    """Selecciona el contorno más probable de ser la línea principal"""
    if not contornos:
        return None
    
    mejor_contorno = None
    mejor_puntuacion = -1
    
    centro_imagen = ancho_imagen // 2
    
    for contorno in contornos:
        area = cv2.contourArea(contorno)
        
        # Filtrar contornos muy pequeños o muy grandes
        if area < 150 or area > 5000:
            continue
        
        # Calcular rectángulo delimitador
        x, y, w, h = cv2.boundingRect(contorno)
        
        # Calcular relación de aspecto (líneas tienden a ser más largas que anchas)
        aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0
        
        # Calcular centroide
        M = cv2.moments(contorno)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Calcular distancia al centro de la imagen
            dist_centro = abs(cx - centro_imagen) / centro_imagen
            
            # Puntuación basada en múltiples factores
            puntuacion = 0
            
            # Factor 1: Área (preferir tamaños medios)
            if 200 <= area <= 2000:
                puntuacion += 30
            elif 150 <= area <= 3000:
                puntuacion += 20
            
            # Factor 2: Relación de aspecto (líneas son alargadas)
            if aspect_ratio > 2:
                puntuacion += 25
            elif aspect_ratio > 1.5:
                puntuacion += 15
            
            # Factor 3: Proximidad al centro (líneas suelen estar centradas)
            if dist_centro < 0.3:
                puntuacion += 25
            elif dist_centro < 0.5:
                puntuacion += 15
            
            # Factor 4: Posición en Y (líneas suelen estar en la parte inferior del ROI)
            if y > h * 0.3:  # En la mitad inferior del ROI
                puntuacion += 20
            
            if puntuacion > mejor_puntuacion:
                mejor_puntuacion = puntuacion
                mejor_contorno = contorno
    
    return mejor_contorno

def procesar_frame_corregido(frame):
    """Procesa frame con corrección de rotación y filtrado avanzado"""
    global ultima_deteccion
    
    try:
        # CORRECCIÓN 1: Rotar imagen 180 grados
        frame_rotado = cv2.rotate(frame, cv2.ROTATE_180)
        
        # Convertir a escala de grises
        if len(frame_rotado.shape) == 3:
            gris = cv2.cvtColor(frame_rotado, cv2.COLOR_RGB2GRAY)
        else:
            gris = frame_rotado.copy()
        
        h, w = gris.shape
        
        # Parámetros actuales
        umbral = parametros_actuales['umbral_binario']
        altura_roi = min(parametros_actuales['altura_roi'], h//2)
        
        # Definir ROI (región de interés)
        roi_y = h - altura_roi
        roi = gris[roi_y:h, 0:w]
        
        # CORRECCIÓN 2: Aplicar filtro Gaussiano antes de binarizar
        roi_suavizada = cv2.GaussianBlur(roi, (5, 5), 0)
        
        # Binarización mejorada
        _, roi_binaria = cv2.threshold(roi_suavizada, umbral, 255, cv2.THRESH_BINARY_INV)
        
        # CORRECCIÓN 3: Aplicar filtros morfológicos para eliminar ruido
        roi_filtrada = aplicar_filtros_morfologicos(roi_binaria)
        
        # Frame de visualización original
        frame_original = cv2.cvtColor(gris, cv2.COLOR_GRAY2RGB)
        
        # Dibujar ROI en verde
        cv2.rectangle(frame_original, (0, roi_y), (w-1, h-1), (0, 255, 0), 2)
        cv2.putText(frame_original, f"ROI", (10, roi_y-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Mostrar parámetros
        cv2.putText(frame_original, f"T:{umbral}", (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # CORRECCIÓN 4: Detección inteligente de línea principal
        linea_detectada = False
        centro_x, centro_y = 0, 0
        posicion_linea = 0.0
        area_contorno = 0
        confianza = 0.0
        
        # Encontrar contornos
        contornos, _ = cv2.findContours(roi_filtrada, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Seleccionar el mejor contorno (línea principal)
        mejor_contorno = seleccionar_contorno_linea_principal(contornos, w)
        
        if mejor_contorno is not None:
            area_contorno = cv2.contourArea(mejor_contorno)
            
            # Calcular centroide
            M = cv2.moments(mejor_contorno)
            if M["m00"] != 0:
                centro_x = int(M["m10"] / M["m00"])
                centro_y = int(M["m01"] / M["m00"]) + roi_y
                
                # Calcular posición normalizada (-1 a 1)
                posicion_linea = (centro_x - w/2) / (w/2)
                linea_detectada = True
                
                # Calcular confianza basada en área y posición
                confianza = min(1.0, area_contorno / 1000.0)
                
                # Dibujar el contorno seleccionado en azul (para debug)
                cv2.drawContours(frame_original, [mejor_contorno], -1, (0, 0, 255), 2, offset=(0, roi_y))
                
                # Dibujar centroide en rojo
                cv2.circle(frame_original, (centro_x, centro_y), 8, (255, 0, 0), -1)
                
                # Mostrar información del centroide
                cv2.putText(frame_original, f"Centro: ({centro_x},{centro_y})", 
                           (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(frame_original, f"Pos: {posicion_linea:.3f}", 
                           (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(frame_original, f"Area: {int(area_contorno)}", 
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Actualizar detección global
        ultima_deteccion.update({
            'linea_detectada': linea_detectada,
            'posicion_linea': posicion_linea,
            'centro_x': centro_x,
            'centro_y': centro_y,
            'pixels_detectados': int(area_contorno),
            'area_contorno': area_contorno,
            'confianza': confianza
        })
        
        # Crear frame binario completo para visualización
        frame_binario_completo = np.zeros_like(gris)
        frame_binario_completo[roi_y:h, 0:w] = roi_filtrada  # Usar imagen filtrada
        frame_binario_rgb = cv2.cvtColor(frame_binario_completo, cv2.COLOR_GRAY2RGB)
        
        # Actualizar etiqueta de estado de línea
        if linea_label:
            if linea_detectada:
                texto_linea = f"Línea: ✓ Detectada | Posición: {posicion_linea:.3f} | Área: {int(area_contorno)} | Conf: {confianza:.2f}"
            else:
                texto_linea = "Línea: ✗ No detectada | Posición: N/A"
            linea_label.config(text=texto_linea)
        
        return frame_original, frame_binario_rgb
        
    except Exception as e:
        print(f"[ERROR] Procesando frame: {e}")
        # Frames vacíos en caso de error
        frame_vacio = np.zeros((240, 320, 3), dtype=np.uint8)
        return frame_vacio, frame_vacio

def hilo_stream_video():
    """Hilo para stream de video optimizado"""
    print("[STREAM] Iniciando stream de video corregido...")
    
    errores_consecutivos = 0
    max_errores = 5
    
    while not stop_stream_flag.is_set():
        try:
            with requests.get(STREAM_URL, stream=True, timeout=TIMEOUT_STREAM) as r:
                r.raise_for_status()
                robot.conectado = True
                errores_consecutivos = 0
                
                bytes_data = b''
                
                for chunk in r.iter_content(chunk_size=2048):
                    if stop_stream_flag.is_set():
                        break
                    
                    bytes_data += chunk
                    
                    # Buscar frame JPEG completo
                    inicio = bytes_data.find(b'\xff\xd8')
                    fin = bytes_data.find(b'\xff\xd9')
                    
                    if inicio != -1 and fin != -1 and fin > inicio:
                        jpg = bytes_data[inicio:fin+2]
                        bytes_data = bytes_data[fin+2:]
                        
                        try:
                            # Decodificar y procesar imagen
                            img = Image.open(io.BytesIO(jpg))
                            frame_np = np.array(img)
                            
                            # Procesar con correcciones (rotación + filtrado)
                            frame_original, frame_binario = procesar_frame_corregido(frame_np)
                            
                            # Mostrar en interfaz
                            mostrar_frames_duales(frame_original, frame_binario)
                            
                        except Exception as e:
                            print(f"[ERROR] Frame: {e}")
                
        except requests.exceptions.ConnectionError:
            errores_consecutivos += 1
            robot.conectado = False
            
            if errores_consecutivos == 1:
                print(f"[STREAM] Conexión perdida con {ESP32_CAM_IP}")
            
            if errores_consecutivos >= max_errores:
                messagebox.showerror("Error de Stream", 
                                   f"Stream perdido tras {errores_consecutivos} intentos.\n"
                                   f"Verifica la conexión con el ESP32-CAM.")
                break
            
            time.sleep(2 + errores_consecutivos)
            
        except Exception as e:
            errores_consecutivos += 1
            print(f"[STREAM] Error: {e}")
            time.sleep(3)
    
    print("[STREAM] Hilo de stream terminado")

def mostrar_frames_duales(frame_original, frame_binario):
    """Muestra ambos frames en la interfaz dual"""
    try:
        if video_original_label and video_original_label.winfo_exists():
            # Redimensionar frames manteniendo proporción
            h_orig, w_orig = frame_original.shape[:2]
            
            # Tamaño objetivo para cada frame
            ancho_objetivo = 320
            alto_objetivo = int(h_orig * (ancho_objetivo / w_orig))
            
            # Redimensionar ambos frames
            frame_orig_resize = cv2.resize(frame_original, (ancho_objetivo, alto_objetivo))
            frame_bin_resize = cv2.resize(frame_binario, (ancho_objetivo, alto_objetivo))
            
            # Convertir a PIL
            img_orig_pil = Image.fromarray(frame_orig_resize)
            img_bin_pil = Image.fromarray(frame_bin_resize)
            
            # Crear PhotoImage
            foto_original = ImageTk.PhotoImage(image=img_orig_pil)
            foto_binaria = ImageTk.PhotoImage(image=img_bin_pil)
            
            # Actualizar labels
            video_original_label.config(image=foto_original)
            video_original_label.image = foto_original
            
            video_binario_label.config(image=foto_binaria)
            video_binario_label.image = foto_binaria
            
    except Exception as e:
        print(f"[ERROR] Mostrando frames: {e}")

def actualizar_estado_labels(estado, modo):
    """Actualiza las etiquetas de estado"""
    try:
        if estado_label and estado_label.winfo_exists():
            estado_label.config(text=f"Estado: {estado}")
        if modo_label and modo_label.winfo_exists():
            modo_label.config(text=f"Modo: {modo}")
    except:
        pass

def actualizar_interfaz_modo():
    """Actualiza interfaz según modo automático"""
    try:
        if auto_button and auto_button.winfo_exists():
            if robot.modo_automatico:
                auto_button.config(text="🔴 Desactivar IA", style="Red.TButton")
                # Deshabilitar controles manuales
                for btn in control_buttons:
                    if btn.winfo_exists() and "STOP" not in btn['text']:
                        btn.config(state="disabled")
            else:
                auto_button.config(text="🟢 Activar IA", style="Green.TButton")
                # Habilitar controles manuales
                for btn in control_buttons:
                    if btn.winfo_exists():
                        btn.config(state="normal")
    except:
        pass

def callback_slider_parametros(param_name):
    """Callback para sliders de parámetros"""
    def callback(valor):
        try:
            # Actualizar parámetro local
            if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro']:
                parametros_actuales[param_name] = int(float(valor))
            else:
                parametros_actuales[param_name] = float(valor)
            
            # Actualizar etiqueta del slider
            if param_name in sliders:
                slider_info = sliders[param_name]
                if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro']:
                    slider_info['valor_label'].config(text=str(int(float(valor))))
                else:
                    slider_info['valor_label'].config(text=f"{float(valor):.3f}")
            
            # Debounce - enviar cada 0.8 segundos
            if not hasattr(callback_slider_parametros, 'ultimo_envio'):
                callback_slider_parametros.ultimo_envio = {}
            
            tiempo_actual = time.time()
            ultimo = callback_slider_parametros.ultimo_envio.get(param_name, 0)
            
            if tiempo_actual - ultimo > 0.8:
                robot.enviar_parametros(parametros_actuales)
                callback_slider_parametros.ultimo_envio[param_name] = tiempo_actual
            
        except Exception as e:
            print(f"[ERROR] Callback slider {param_name}: {e}")
    
    return callback

def crear_interfaz_completa():
    """Crea la interfaz completa con correcciones implementadas"""
    global root, estado_label, modo_label, linea_label
    global video_original_label, video_binario_label, auto_button, control_buttons, sliders
    
    root = tk.Tk()
    root.title("🤖 Robot Seguidor de Línea - ESP32-CAM (Corregido)")
    root.geometry("1350x900")
    root.configure(bg='#f0f0f0')
    root.resizable(True, True)
    
    # Configurar estilos
    style = ttk.Style()
    style.theme_use('clam')
    
    # Estilos personalizados
    style.configure("Title.TLabel", font=("Arial", 16, "bold"), background='#f0f0f0')
    style.configure("Status.TLabel", font=("Arial", 11), background='#e8f4fd', relief="solid", padding=8)
    style.configure("Green.TButton", background="#28a745", foreground="white", font=("Arial", 11, "bold"))
    style.configure("Red.TButton", background="#dc3545", foreground="white", font=("Arial", 11, "bold"))
    style.configure("Blue.TButton", background="#007bff", foreground="white", font=("Arial", 10, "bold"))
    style.configure("Orange.TButton", background="#fd7e14", foreground="white", font=("Arial", 10, "bold"))
    
    # Frame principal
    main_frame = ttk.Frame(root, padding="15")
    main_frame.pack(fill="both", expand=True)
    
    # ==========================================
    # TÍTULO PRINCIPAL
    # ==========================================
    titulo_frame = ttk.Frame(main_frame)
    titulo_frame.pack(fill="x", pady=(0, 15))
    
    titulo_label = ttk.Label(titulo_frame, text="🤖 Robot Seguidor de Línea ESP32-CAM (Versión Corregida)", 
                            style="Title.TLabel")
    titulo_label.pack()
    
    # Subtítulo con correcciones
    subtitulo_label = ttk.Label(titulo_frame, text="✅ Rotación 180° corregida | ✅ Filtrado anti-ruido | ✅ Detección inteligente", 
                               font=("Arial", 10), background='#f0f0f0', foreground='#28a745')
    subtitulo_label.pack(pady=(5, 0))
    
    # ==========================================
    # ESTADO DEL SISTEMA
    # ==========================================
    estado_frame = ttk.LabelFrame(main_frame, text="📊 Estado del Sistema", padding="10")
    estado_frame.pack(fill="x", pady=(0, 15))
    
    # Fila de estados
    estados_container = ttk.Frame(estado_frame)
    estados_container.pack(fill="x")
    
    # Estados izquierda
    estados_izq = ttk.Frame(estados_container)
    estados_izq.pack(side="left", fill="x", expand=True)
    
    estado_label = ttk.Label(estados_izq, text="Estado: Iniciando...", style="Status.TLabel")
    estado_label.pack(side="left", padx=(0, 15))
    
    modo_label = ttk.Label(estados_izq, text="Modo: Manual", style="Status.TLabel")
    modo_label.pack(side="left")
    
    # Estado de línea (derecha)
    linea_label = ttk.Label(estados_container, text="Línea: N/A | Posición: N/A", 
                           style="Status.TLabel")
    linea_label.pack(side="right")
    
    # ==========================================
    # VISUALIZACIÓN DUAL DE CÁMARA CORREGIDA
    # ==========================================
    video_container = ttk.Frame(main_frame)
    video_container.pack(fill="both", expand=True, pady=(0, 15))
    
    # Frame izquierdo - Cámara original (ROTADA)
    video_original_frame = ttk.LabelFrame(video_container, text="📹 Cámara con Detección (Rotación Corregida)", padding="10")
    video_original_frame.pack(side="left", fill="both", expand=True, padx=(0, 8))
    
    video_original_label = tk.Label(video_original_frame, bg="black", 
                                   text="🔄 Conectando a ESP32-CAM...\n✅ Aplicando rotación 180°\n🎯 Activando detección inteligente", 
                                   fg="white", font=("Arial", 11), justify="center")
    video_original_label.pack(fill="both", expand=True)
    
    # Frame derecho - Imagen binarizada (FILTRADA)
    video_binario_frame = ttk.LabelFrame(video_container, text="🔍 Imagen Binarizada (Filtros Anti-Ruido)", padding="10")
    video_binario_frame.pack(side="right", fill="both", expand=True, padx=(8, 0))
    
    video_binario_label = tk.Label(video_binario_frame, bg="black", 
                                  text="🧹 Filtros morfológicos activos\n🎯 Eliminación de objetos no deseados\n✨ Detección de línea principal", 
                                  fg="white", font=("Arial", 11), justify="center")
    video_binario_label.pack(fill="both", expand=True)
    
    # ==========================================
    # CONTROL PRINCIPAL
    # ==========================================
    control_principal_frame = ttk.LabelFrame(main_frame, text="🎮 Control Principal", padding="15")
    control_principal_frame.pack(fill="x", pady=(0, 15))
    
    botones_principales = ttk.Frame(control_principal_frame)
    botones_principales.pack()
    
    auto_button = ttk.Button(botones_principales, text="🟢 Activar IA", 
                            command=robot.alternar_modo_automatico, style="Green.TButton", width=20)
    auto_button.pack(side="left", padx=(0, 20))
    
    stop_button = ttk.Button(botones_principales, text="🔴 DETENER TODO", 
                            command=robot.parar_robot, style="Red.TButton", width=20)
    stop_button.pack(side="left")
    
    # ==========================================
    # AJUSTE DE PARÁMETROS CON SLIDERS
    # ==========================================
    parametros_frame = ttk.LabelFrame(main_frame, text="🔧 Ajuste de Parámetros de Visión (Optimizados)", padding="15")
    parametros_frame.pack(fill="x", pady=(0, 15))
    
    # Configuración de parámetros optimizada
    params_config = {
        'umbral_binario': {'label': 'Umbral de Binarización (0-255)', 'min': 40, 'max': 150, 'step': 1},
        'altura_roi': {'label': 'Altura de ROI (píxeles)', 'min': 40, 'max': 100, 'step': 5},
        'zona_muerta_centro': {'label': 'Zona Muerta Central (0.0-0.3)', 'min': 0.02, 'max': 0.2, 'step': 0.01},
        'velocidad_base': {'label': 'Velocidad Base (100-200)', 'min': 100, 'max': 200, 'step': 5},
        'velocidad_giro': {'label': 'Velocidad de Giro (80-160)', 'min': 80, 'max': 160, 'step': 5}
    }
    
    # Organizar en dos columnas
    columna_izq = ttk.Frame(parametros_frame)
    columna_izq.pack(side="left", fill="both", expand=True, padx=(0, 15))
    
    columna_der = ttk.Frame(parametros_frame)
    columna_der.pack(side="right", fill="both", expand=True)
    
    contador = 0
    for param_name, config in params_config.items():
        # Alternar entre columnas
        parent = columna_izq if contador < 3 else columna_der
        contador += 1
        
        # Frame para cada parámetro
        param_frame = ttk.Frame(parent)
        param_frame.pack(fill="x", pady=5)
        
        # Etiqueta
        label = ttk.Label(param_frame, text=config['label'], font=("Arial", 10, "bold"))
        label.pack(anchor="w")
        
        # Frame para slider y valor
        slider_frame = ttk.Frame(param_frame)
        slider_frame.pack(fill="x", pady=(5, 0))
        
        # Slider
        slider = tk.Scale(slider_frame, from_=config['min'], to=config['max'], 
                         resolution=config['step'], orient="horizontal", length=250,
                         command=callback_slider_parametros(param_name))
        slider.set(parametros_actuales[param_name])
        slider.pack(side="left", fill="x", expand=True)
        
        # Etiqueta de valor
        if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro']:
            valor_texto = str(parametros_actuales[param_name])
        else:
            valor_texto = f"{parametros_actuales[param_name]:.3f}"
            
        valor_label = ttk.Label(slider_frame, text=valor_texto, font=("Arial", 10, "bold"), width=8)
        valor_label.pack(side="right", padx=(10, 0))
        
        # Guardar referencia
        sliders[param_name] = {'slider': slider, 'valor_label': valor_label}
    
    # Botón de reset
    reset_button = ttk.Button(parametros_frame, text="🔄 Resetear Valores Optimizados", 
                             command=resetear_parametros, style="Orange.TButton")
    reset_button.pack(pady=(15, 0))
    
    # ==========================================
    # CONTROL MANUAL
    # ==========================================
    control_manual_frame = ttk.LabelFrame(main_frame, text="🕹️ Control Manual", padding="15")
    control_manual_frame.pack(fill="x")
    
    # Instrucciones
    instrucciones = ttk.Label(control_manual_frame, 
                             text="Teclado: W(adelante) A(izq) S(stop) D(der) Q(rotar-izq) E(rotar-der) X(atrás)", 
                             font=("Arial", 10), background='#f0f0f0')
    instrucciones.pack(pady=(0, 15))
    
    # Grid de botones
    botones_grid = ttk.Frame(control_manual_frame)
    botones_grid.pack()
    
    # Configuración de botones
    botones_config = [
        ("↺ Rotar Izq (Q)", "rotate_left", 0, 0),
        ("⬆ Adelante (W)", "forward", 0, 1),
        ("↻ Rotar Der (E)", "rotate_right", 0, 2),
        ("⬅ Izquierda (A)", "left", 1, 0),
        ("⏹ STOP (S)", "stop", 1, 1),
        ("➡ Derecha (D)", "right", 1, 2),
        ("", "", 2, 0),  # Espacio vacío
        ("⬇ Atrás (X)", "backward", 2, 1),
        ("", "", 2, 2),  # Espacio vacío
    ]
    
    control_buttons = []
    for texto, comando, fila, columna in botones_config:
        if texto:  # Solo crear botón si hay texto
            estilo = "Red.TButton" if "STOP" in texto else "Blue.TButton"
            btn = ttk.Button(botones_grid, text=texto, 
                           command=lambda cmd=comando: robot.enviar_movimiento(cmd),
                           style=estilo, width=16)
            btn.grid(row=fila, column=columna, padx=5, pady=5)
            control_buttons.append(btn)
    
    # ==========================================
    # EVENTOS Y CONFIGURACIÓN FINAL
    # ==========================================
    
    # Eventos de teclado
    def manejar_tecla(event):
        teclas = {
            'w': 'forward', 'a': 'left', 's': 'stop', 'd': 'right',
            'q': 'rotate_left', 'e': 'rotate_right', 'x': 'backward'
        }
        if event.char.lower() in teclas:
            robot.enviar_movimiento(teclas[event.char.lower()])
    
    root.bind('<KeyPress>', manejar_tecla)
    root.focus_set()
    
    # Información de IP
    ip_info = ttk.Label(main_frame, text=f"🌐 ESP32-CAM: {ESP32_CAM_IP} | 🔄 Rotación: Corregida | 🧹 Filtros: Activos", 
                       font=("Arial", 10), background='#f0f0f0')
    ip_info.pack(pady=(10, 0))
    
    # Protocolo de cierre
    def al_cerrar():
        print("[CIERRE] Cerrando aplicación...")
        stop_stream_flag.set()
        robot.parar_robot()
        time.sleep(0.7)
        root.quit()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", al_cerrar)
    
    # Iniciar hilo de video
    video_thread = threading.Thread(target=hilo_stream_video, daemon=True)
    video_thread.start()
    
    # Enviar parámetros iniciales
    root.after(3000, lambda: robot.enviar_parametros(parametros_actuales))
    
    # Estado inicial
    actualizar_estado_labels("Conectando...", "Manual")
    
    print("=" * 80)
    print("🤖 ROBOT SEGUIDOR DE LÍNEA - VERSIÓN CORREGIDA INICIADA")
    print("=" * 80)
    print(f"📡 Conectando a ESP32-CAM: {ESP32_CAM_IP}")
    print("🔄 CORRECCIÓN 1: Rotación de imagen 180° aplicada")
    print("🧹 CORRECCIÓN 2: Filtros morfológicos anti-ruido activos")
    print("🎯 CORRECCIÓN 3: Detección inteligente de línea principal")
    print("📹 Stream dual: Original + Binarizada")
    print("🎛️ Sliders de parámetros optimizados")
    print("⌨️ Controles de teclado habilitados")
    print("=" * 80)
    
    root.mainloop()

def resetear_parametros():
    """Resetea todos los parámetros a valores optimizados"""
    global parametros_actuales
    
    # Valores optimizados para evitar problemas
    parametros_optimizados = {
        'umbral_binario': 75,        # Umbral medio-bajo
        'altura_roi': 50,            # ROI más pequeña
        'zona_muerta_centro': 0.10,  # Zona muerta mayor para estabilidad
        'velocidad_base': 140,       # Velocidad reducida
        'velocidad_giro': 100,       # Giro más lento
    }
    
    parametros_actuales = parametros_optimizados.copy()
    
    # Actualizar todos los sliders
    for param_name, slider_info in sliders.items():
        slider_info['slider'].set(parametros_actuales[param_name])
        
        # Actualizar etiqueta de valor
        if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro']:
            slider_info['valor_label'].config(text=str(parametros_actuales[param_name]))
        else:
            slider_info['valor_label'].config(text=f"{parametros_actuales[param_name]:.3f}")
    
    # Enviar al ESP32
    robot.enviar_parametros(parametros_actuales)
    
    print("[RESET] Parámetros reseteados a valores optimizados anti-ruido")
    messagebox.showinfo("Parámetros Optimizados", 
                       "Parámetros reseteados a valores optimizados:\n"
                       "• Umbral: 75 (medio-bajo)\n"
                       "• ROI: 50px (más pequeña)\n"
                       "• Zona muerta: 0.10 (mayor estabilidad)\n"
                       "• Velocidades reducidas para mayor control")

if __name__ == "__main__":
    print("=" * 90)
    print("🤖 ROBOT SEGUIDOR DE LÍNEA ESP32-CAM - VERSIÓN CORREGIDA")
    print("=" * 90)
    print(f"📡 IP ESP32-CAM configurada: {ESP32_CAM_IP}")
    print("⚠️  IMPORTANTE: Verifica que la IP sea correcta antes de continuar")
    print("📝 Si la IP es incorrecta, cambia ESP32_CAM_IP en la línea 25")
    print()
    print("🔧 CORRECCIONES IMPLEMENTADAS:")
    print("• 🔄 Rotación de imagen 180° para corregir orientación")
    print("• 🧹 Filtros morfológicos para eliminar ruido y objetos no deseados")
    print("• 🎯 Detección inteligente que selecciona solo la línea principal")
    print("• 📏 Filtrado por área, forma y posición para mayor precisión")
    print("• ⚡ Suavizado Gaussiano antes de binarización")
    print("• 🎛️ Parámetros optimizados para evitar falsas detecciones")
    print()
    print("✨ CARACTERÍSTICAS MEJORADAS:")
    print("• 📹 Visualización dual con filtros aplicados")
    print("• 🎯 Centroide más preciso y estable")
    print("• 🛡️ Resistente a objetos blancos en el fondo")
    print("• 📊 Información detallada de confianza y área")
    print("• 🎮 Control manual y automático")
    print()
    print("🚀 Iniciando interfaz corregida...")
    print("=" * 90)
    
    try:
        crear_interfaz_completa()
    except KeyboardInterrupt:
        print("\n⚠️ Aplicación terminada por el usuario (Ctrl+C)")
    except Exception as e:
        print(f"\n❌ Error crítico: {e}")
        messagebox.showerror("Error Crítico", f"Error en la aplicación: {e}")
    finally:
        print("👋 Aplicación terminada correctamente")