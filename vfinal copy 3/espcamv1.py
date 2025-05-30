"""
ROBOT SEGUIDOR DE LÍNEA ESP32-CAM - INTERFAZ COMPLETA
====================================================

Interfaz que replica exactamente la imagen mostrada con:
- Visualización dual (original + binarizada)
- Detección de centroide con ROI
- Sliders para ajuste de parámetros
- Control manual y automático
- Seguimiento preciso de línea negra
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
    'velocidad_base': 180,
    'velocidad_giro': 140,
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

# Variables de detección
ultima_deteccion = {
    'linea_detectada': False,
    'posicion_linea': 0.0,
    'centro_x': 0,
    'centro_y': 0,
    'pixels_detectados': 0
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

def procesar_frame_completo(frame):
    """Procesa frame para mostrar detección exacta como en la imagen"""
    global ultima_deteccion
    
    try:
        # Convertir a escala de grises
        if len(frame.shape) == 3:
            gris = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        else:
            gris = frame.copy()
        
        h, w = gris.shape
        
        # Parámetros actuales
        umbral = parametros_actuales['umbral_binario']
        altura_roi = min(parametros_actuales['altura_roi'], h//2)
        
        # Definir ROI (región de interés)
        roi_y = h - altura_roi
        roi = gris[roi_y:h, 0:w]
        
        # Binarización - línea negra sobre fondo claro
        _, roi_binaria = cv2.threshold(roi, umbral, 255, cv2.THRESH_BINARY_INV)
        
        # Frame de visualización original
        frame_original = cv2.cvtColor(gris, cv2.COLOR_GRAY2RGB)
        
        # Dibujar ROI en verde
        cv2.rectangle(frame_original, (0, roi_y), (w-1, h-1), (0, 255, 0), 2)
        cv2.putText(frame_original, f"ROI", (10, roi_y-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Mostrar parámetros
        cv2.putText(frame_original, f"T:{umbral}", (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Detectar centroide de la línea
        linea_detectada = False
        centro_x, centro_y = 0, 0
        posicion_linea = 0.0
        pixels_detectados = 0
        
        # Encontrar contornos
        contornos, _ = cv2.findContours(roi_binaria, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contornos:
            # Encontrar el contorno más grande
            contorno_mayor = max(contornos, key=cv2.contourArea)
            area = cv2.contourArea(contorno_mayor)
            
            if area > 200:  # Área mínima para considerar una línea válida
                # Calcular momentos para encontrar centroide
                M = cv2.moments(contorno_mayor)
                if M["m00"] != 0:
                    centro_x = int(M["m10"] / M["m00"])
                    centro_y = int(M["m01"] / M["m00"]) + roi_y
                    
                    # Calcular posición normalizada (-1 a 1)
                    posicion_linea = (centro_x - w/2) / (w/2)
                    linea_detectada = True
                    pixels_detectados = int(area)
                    
                    # Dibujar centroide en rojo
                    cv2.circle(frame_original, (centro_x, centro_y), 8, (255, 0, 0), -1)
                    
                    # Mostrar información del centroide
                    cv2.putText(frame_original, f"Centro: ({centro_x},{centro_y})", 
                               (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    cv2.putText(frame_original, f"Pos: {posicion_linea:.3f}", 
                               (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Actualizar detección global
        ultima_deteccion.update({
            'linea_detectada': linea_detectada,
            'posicion_linea': posicion_linea,
            'centro_x': centro_x,
            'centro_y': centro_y,
            'pixels_detectados': pixels_detectados
        })
        
        # Crear frame binario completo para visualización
        frame_binario_completo = np.zeros_like(gris)
        frame_binario_completo[roi_y:h, 0:w] = roi_binaria
        frame_binario_rgb = cv2.cvtColor(frame_binario_completo, cv2.COLOR_GRAY2RGB)
        
        # Actualizar etiqueta de estado de línea
        if linea_label:
            if linea_detectada:
                texto_linea = f"Línea: ✓ Detectada | Posición: {posicion_linea:.3f}"
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
    print("[STREAM] Iniciando stream de video...")
    
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
                            
                            # Procesar para obtener ambos frames
                            frame_original, frame_binario = procesar_frame_completo(frame_np)
                            
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
    """Crea la interfaz completa replicando la imagen mostrada"""
    global root, estado_label, modo_label, linea_label
    global video_original_label, video_binario_label, auto_button, control_buttons, sliders
    
    root = tk.Tk()
    root.title("🤖 Robot Seguidor de Línea - ESP32-CAM")
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
    
    titulo_label = ttk.Label(titulo_frame, text="🤖 Robot Seguidor de Línea ESP32-CAM", 
                            style="Title.TLabel")
    titulo_label.pack()
    
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
    # VISUALIZACIÓN DUAL DE CÁMARA
    # ==========================================
    video_container = ttk.Frame(main_frame)
    video_container.pack(fill="both", expand=True, pady=(0, 15))
    
    # Frame izquierdo - Cámara original
    video_original_frame = ttk.LabelFrame(video_container, text="📹 Cámara con Detección", padding="10")
    video_original_frame.pack(side="left", fill="both", expand=True, padx=(0, 8))
    
    video_original_label = tk.Label(video_original_frame, bg="black", 
                                   text="Conectando a ESP32-CAM...\nEsperando stream de video...", 
                                   fg="white", font=("Arial", 12), justify="center")
    video_original_label.pack(fill="both", expand=True)
    
    # Frame derecho - Imagen binarizada
    video_binario_frame = ttk.LabelFrame(video_container, text="🔍 Imagen Binarizada", padding="10")
    video_binario_frame.pack(side="right", fill="both", expand=True, padx=(8, 0))
    
    video_binario_label = tk.Label(video_binario_frame, bg="black", 
                                  text="Procesamiento de imagen...\nUmbralización en tiempo real", 
                                  fg="white", font=("Arial", 12), justify="center")
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
    parametros_frame = ttk.LabelFrame(main_frame, text="🔧 Ajuste de Parámetros de Visión", padding="15")
    parametros_frame.pack(fill="x", pady=(0, 15))
    
    # Configuración de parámetros
    params_config = {
        'umbral_binario': {'label': 'Umbral de Binarización (0-255)', 'min': 0, 'max': 255, 'step': 1},
        'altura_roi': {'label': 'Altura de ROI (píxeles)', 'min': 30, 'max': 120, 'step': 5},
        'zona_muerta_centro': {'label': 'Zona Muerta Central (0.0-0.3)', 'min': 0.02, 'max': 0.3, 'step': 0.01},
        'velocidad_base': {'label': 'Velocidad Base (100-255)', 'min': 100, 'max': 255, 'step': 5},
        'velocidad_giro': {'label': 'Velocidad de Giro (80-200)', 'min': 80, 'max': 200, 'step': 5}
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
    reset_button = ttk.Button(parametros_frame, text="🔄 Resetear Valores por Defecto", 
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
    ip_info = ttk.Label(main_frame, text=f"🌐 Conectado a ESP32-CAM: {ESP32_CAM_IP}", 
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
    
    print("=" * 70)
    print("🤖 ROBOT SEGUIDOR DE LÍNEA - INTERFAZ COMPLETA INICIADA")
    print("=" * 70)
    print(f"📡 Conectando a ESP32-CAM: {ESP32_CAM_IP}")
    print("📹 Stream dual: Original + Binarizada")
    print("🎛️ Sliders de parámetros en tiempo real")
    print("🎮 Control manual y automático")
    print("⌨️ Controles de teclado habilitados")
    print("=" * 70)
    
    root.mainloop()

def resetear_parametros():
    """Resetea todos los parámetros a valores por defecto"""
    global parametros_actuales
    
    parametros_actuales = PARAMETROS_DEFAULT.copy()
    
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
    
    print("[RESET] Parámetros reseteados a valores por defecto")
    messagebox.showinfo("Parámetros Reseteados", 
                       "Todos los parámetros han sido reseteados a sus valores por defecto.")

if __name__ == "__main__":
    print("=" * 80)
    print("🤖 ROBOT SEGUIDOR DE LÍNEA ESP32-CAM - INTERFAZ COMPLETA")
    print("=" * 80)
    print(f"📡 IP ESP32-CAM configurada: {ESP32_CAM_IP}")
    print("⚠️  IMPORTANTE: Verifica que la IP sea correcta antes de continuar")
    print("📝 Si la IP es incorrecta, cambia ESP32_CAM_IP en la línea 25")
    print()
    print("✨ CARACTERÍSTICAS DE LA INTERFAZ:")
    print("• 📹 Visualización dual (Original + Binarizada)")
    print("• 🎯 Detección de centroide en tiempo real")
    print("• 🎛️ 5 sliders para ajuste de parámetros")
    print("• 🎮 Control manual y automático")
    print("• ⌨️ Controles de teclado (W-A-S-D-Q-E-X)")
    print("• 🔄 Botón de reset de parámetros")
    print("• 📊 Estado detallado del sistema")
    print()
    print("🚀 Iniciando interfaz completa...")
    print("=" * 80)
    
    try:
        crear_interfaz_completa()
    except KeyboardInterrupt:
        print("\n⚠️ Aplicación terminada por el usuario (Ctrl+C)")
    except Exception as e:
        print(f"\n❌ Error crítico: {e}")
        messagebox.showerror("Error Crítico", f"Error en la aplicación: {e}")
    finally:
        print("👋 Aplicación terminada correctamente")
        