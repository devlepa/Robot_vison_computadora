"""
CLIENTE PYTHON OPTIMIZADO PARA ESP32-CAM - VERSION LIGERA
=========================================================

OPTIMIZACIONES IMPLEMENTADAS:
- Comunicacion HTTP simplificada y mas robusta
- Timeouts aumentados para ESP32 sobrecargado
- Frecuencia de peticiones reducida
- Mejor manejo de errores de conexion
- Interfaz mas ligera para menor carga de procesamiento
- Sincronizacion optimizada con cores del ESP32

DISTRIBUCION DE TAREAS:
- ESP32 Core 0: Control de motores
- ESP32 Core 1: Comunicacion WiFi
- Python: Interfaz de usuario y parametros
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
# CONFIGURACION DE CONEXION OPTIMIZADA
# ==========================================

# CAMBIAR ESTA IP POR LA DE TU ESP32-CAM
ESP32_CAM_IP = "192.168.196.182"

# URLs de la API optimizada
BASE_URL = f"http://{ESP32_CAM_IP}"
STREAM_URL = f"{BASE_URL}/stream"
AUTO_MODE_URL = f"{BASE_URL}/auto"
STOP_URL = f"{BASE_URL}/stop"
MOVE_URL = f"{BASE_URL}/move"
PARAMS_URL = f"{BASE_URL}/params"

# Timeouts aumentados para ESP32 ocupado
TIMEOUT_COMANDO = 8      # 8 segundos para comandos
TIMEOUT_STREAM = 12      # 12 segundos para stream
TIMEOUT_PARAMS = 5       # 5 segundos para parametros

# Parametros por defecto sincronizados con ESP32
PARAMETROS_DEFAULT = {
    'umbral_binario': 80,
    'altura_roi': 50,            # ROI reducida
    'zona_muerta_centro': 0.12,  # Zona muerta aumentada
    'velocidad_base': 180,       # Velocidad reducida
    'velocidad_giro': 140,       # Velocidad giro reducida
}

# ==========================================
# VARIABLES GLOBALES SIMPLIFICADAS
# ==========================================

# Referencias a widgets principales
root = None
video_label = None
status_label = None
mode_label = None
line_label = None
connection_label = None

# Controles
auto_button = None
control_frame = None
sliders = {}

# Control de hilos
stop_stream_flag = threading.Event()
parametros_actuales = PARAMETROS_DEFAULT.copy()

class RobotControllerOptimizado:
    """Controlador optimizado para conexion estable con ESP32-CAM"""
    
    def __init__(self):
        self.conectado = False
        self.modo_auto = False
        self.ultimo_comando = time.time()
        self.intentos_reconexion = 0
        self.max_intentos = 3
        
        # Estadisticas simplificadas
        self.stats = {
            'comandos_ok': 0,
            'comandos_error': 0,
            'ultima_conexion': None
        }
    
    def enviar_comando_http(self, url, datos=None, timeout=TIMEOUT_COMANDO):
        """Envio HTTP optimizado con reintentos"""
        
        for intento in range(self.max_intentos):
            try:
                if datos:
                    # POST con JSON
                    headers = {'Content-Type': 'application/json'}
                    response = requests.post(url, json=datos, headers=headers, timeout=timeout)
                else:
                    # GET simple
                    response = requests.get(url, timeout=timeout)
                
                response.raise_for_status()
                
                # Conexion exitosa
                self.conectado = True
                self.intentos_reconexion = 0
                self.stats['comandos_ok'] += 1
                self.stats['ultima_conexion'] = datetime.now().strftime("%H:%M:%S")
                
                print(f"[OK] {url} - Intento {intento + 1}")
                
                # Intentar parsear JSON, sino devolver texto
                try:
                    return response.json()
                except:
                    return {"status": "ok", "data": response.text}
                    
            except requests.exceptions.ConnectionError:
                self.conectado = False
                self.stats['comandos_error'] += 1
                print(f"[ERROR] Conexion fallida - Intento {intento + 1}/{self.max_intentos}")
                
                if intento < self.max_intentos - 1:
                    time.sleep(1 + intento)  # Backoff progresivo
                else:
                    self.mostrar_error_conexion()
                    
            except requests.exceptions.Timeout:
                print(f"[TIMEOUT] {url} - Intento {intento + 1}")
                if intento == self.max_intentos - 1:
                    messagebox.showwarning("Timeout", 
                                         f"ESP32-CAM no responde en {timeout}s\n"
                                         f"El sistema puede estar sobrecargado")
                
            except Exception as e:
                print(f"[ERROR] {url}: {e}")
                break
        
        return None
    
    def mostrar_error_conexion(self):
        """Muestra error de conexion con diagnostico"""
        mensaje = f"""Error de Conexion con ESP32-CAM

IP configurada: {ESP32_CAM_IP}

Verificaciones:
✓ ESP32-CAM encendido y conectado al WiFi
✓ IP correcta (revisar Monitor Serie de Arduino)  
✓ Misma red WiFi (PC y ESP32)
✓ No hay firewall bloqueando puerto 80

Estadisticas:
• Comandos exitosos: {self.stats['comandos_ok']}
• Errores de conexion: {self.stats['comandos_error']}
• Ultima conexion: {self.stats['ultima_conexion'] or 'Nunca'}

El ESP32 puede estar sobrecargado.
Intenta reiniciar el ESP32-CAM."""
        
        messagebox.showerror("Error de Conexion", mensaje)
    
    def enviar_movimiento(self, direccion, velocidad=None):
        """Envia comando de movimiento"""
        if self.modo_auto and direccion != "stop":
            messagebox.showinfo("Modo Automatico", 
                              "Robot en modo automatico.\nUse STOP o desactive modo automatico.")
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
            actualizar_estado(f"Movimiento: {direccion}", 
                            "Manual" if not self.modo_auto else "Automatico")
            return True
        return False
    
    def alternar_modo_auto(self):
        """Cambia entre modo manual y automatico"""
        nuevo_modo = not self.modo_auto
        datos = {"auto": nuevo_modo}
        
        resultado = self.enviar_comando_http(AUTO_MODE_URL, datos)
        
        if resultado:
            self.modo_auto = nuevo_modo
            actualizar_interfaz_modo()
            
            modo_texto = "Automatico" if self.modo_auto else "Manual"
            actualizar_estado("Modo cambiado", modo_texto)
            
            print(f"[MODO] {modo_texto}")
            return True
        return False
    
    def parar_robot(self):
        """Parada de emergencia"""
        resultado = self.enviar_comando_http(STOP_URL, {})
        
        if resultado:
            self.modo_auto = False
            actualizar_interfaz_modo()
            actualizar_estado("DETENIDO", "Manual")
            print("[STOP] Robot detenido")
            return True
        return False
    
    def enviar_parametros(self, params):
        """Envia parametros de vision al ESP32"""
        datos = {
            "umbral": int(params['umbral_binario']),
            "altura_roi": int(params['altura_roi']),
            "zona_muerta": float(params['zona_muerta_centro']),
            "velocidad_base": int(params['velocidad_base']),
            "velocidad_giro": int(params['velocidad_giro'])
        }
        
        resultado = self.enviar_comando_http(PARAMS_URL, datos, TIMEOUT_PARAMS)
        
        if resultado:
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"[PARAMS] Actualizados a las {timestamp}")
            return True
        return False

# Instancia global del controlador
robot = RobotControllerOptimizado()

def procesar_frame_ligero(frame):
    """Procesamiento de frame simplificado para menor carga"""
    try:
        if len(frame.shape) == 3:
            gris = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        else:
            gris = frame.copy()
        
        h, w = gris.shape
        
        # ROI simplificada
        roi_h = min(parametros_actuales['altura_roi'], h//2)
        roi_y = h - roi_h
        roi = gris[roi_y:h, 0:w]
        
        # Binarizacion
        umbral = parametros_actuales['umbral_binario']
        _, roi_bin = cv2.threshold(roi, umbral, 255, cv2.THRESH_BINARY_INV)
        
        # Frame de display simple
        display = cv2.cvtColor(gris, cv2.COLOR_GRAY2RGB)
        
        # Dibujar ROI
        cv2.rectangle(display, (0, roi_y), (w-1, h-1), (0, 255, 0), 2)
        cv2.putText(display, f"ROI:{roi_h} T:{umbral}", (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Deteccion simple de centroide
        contornos, _ = cv2.findContours(roi_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        linea_detectada = False
        posicion = 0.0
        
        if contornos:
            contorno_mayor = max(contornos, key=cv2.contourArea)
            if cv2.contourArea(contorno_mayor) > 100:
                M = cv2.moments(contorno_mayor)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + roi_y
                    
                    posicion = (cx - w/2) / (w/2)
                    linea_detectada = True
                    
                    # Dibujar centroide
                    cv2.circle(display, (cx, cy), 6, (255, 0, 0), -1)
                    cv2.putText(display, f"Pos:{posicion:.2f}", 
                               (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Actualizar estado de linea
        if line_label:
            estado_linea = "Detectada" if linea_detectada else "No detectada"
            pos_texto = f"Pos: {posicion:.3f}" if linea_detectada else "Pos: N/A"
            line_label.config(text=f"Linea: {estado_linea} | {pos_texto}")
        
        return display
        
    except Exception as e:
        print(f"[ERROR] Procesando frame: {e}")
        return np.zeros((240, 320, 3), dtype=np.uint8)

def hilo_stream_optimizado():
    """Hilo de stream optimizado para conexion estable"""
    print("[STREAM] Iniciando stream optimizado...")
    
    errores_consecutivos = 0
    max_errores = 5
    
    while not stop_stream_flag.is_set():
        try:
            with requests.get(STREAM_URL, stream=True, timeout=TIMEOUT_STREAM) as r:
                r.raise_for_status()
                robot.conectado = True
                errores_consecutivos = 0
                
                if connection_label:
                    connection_label.config(text="Conexion: OK", fg="green")
                
                bytes_data = b''
                frames_procesados = 0
                
                for chunk in r.iter_content(chunk_size=2048):  # Chunks mas grandes
                    if stop_stream_flag.is_set():
                        break
                    
                    bytes_data += chunk
                    
                    # Buscar frame JPEG completo
                    inicio = bytes_data.find(b'\xff\xd8')
                    fin = bytes_data.find(b'\xff\xd9')
                    
                    if inicio != -1 and fin != -1 and fin > inicio:
                        jpg = bytes_data[inicio:fin+2]
                        bytes_data = bytes_data[fin+2:]
                        
                        # Procesar solo cada 3er frame para reducir carga
                        frames_procesados += 1
                        if frames_procesados % 3 == 0:
                            try:
                                img = Image.open(io.BytesIO(jpg))
                                frame_np = np.array(img)
                                frame_procesado = procesar_frame_ligero(frame_np)
                                mostrar_frame(frame_procesado)
                            except Exception as e:
                                print(f"[ERROR] Frame individual: {e}")
                
        except requests.exceptions.ConnectionError:
            errores_consecutivos += 1
            robot.conectado = False
            
            if connection_label:
                connection_label.config(text="Conexion: ERROR", fg="red")
            
            if errores_consecutivos == 1:
                print(f"[STREAM] Conexion perdida con {ESP32_CAM_IP}")
            
            if errores_consecutivos >= max_errores:
                print(f"[STREAM] Demasiados errores ({errores_consecutivos})")
                messagebox.showerror("Error de Stream", 
                                   f"Stream perdido tras {errores_consecutivos} intentos.\n"
                                   f"Verifica la conexion con el ESP32-CAM.")
                break
            
            time.sleep(2 + errores_consecutivos)  # Backoff progresivo
            
        except Exception as e:
            errores_consecutivos += 1
            print(f"[STREAM] Error: {e}")
            time.sleep(3)
    
    print("[STREAM] Hilo terminado")

def mostrar_frame(frame):
    """Muestra frame en la interfaz"""
    try:
        if video_label and video_label.winfo_exists():
            # Redimensionar para visualizacion
            h, w = frame.shape[:2]
            nuevo_w = 400
            nuevo_h = int(h * (nuevo_w / w))
            
            frame_resize = cv2.resize(frame, (nuevo_w, nuevo_h))
            img_pil = Image.fromarray(frame_resize)
            foto = ImageTk.PhotoImage(image=img_pil)
            
            video_label.config(image=foto)
            video_label.image = foto
            
    except Exception as e:
        print(f"[ERROR] Mostrando frame: {e}")

def actualizar_estado(estado, modo):
    """Actualiza etiquetas de estado"""
    try:
        if status_label and status_label.winfo_exists():
            status_label.config(text=f"Estado: {estado}")
        if mode_label and mode_label.winfo_exists():
            mode_label.config(text=f"Modo: {modo}")
    except:
        pass

def actualizar_interfaz_modo():
    """Actualiza interfaz segun modo automatico"""
    try:
        if auto_button and auto_button.winfo_exists():
            if robot.modo_auto:
                auto_button.config(text="Desactivar IA", style="Red.TButton")
                # Deshabilitar controles manuales
                if control_frame:
                    for widget in control_frame.winfo_children():
                        if isinstance(widget, ttk.Button) and widget != auto_button:
                            widget.config(state="disabled")
            else:
                auto_button.config(text="Activar IA", style="Green.TButton")
                # Habilitar controles manuales
                if control_frame:
                    for widget in control_frame.winfo_children():
                        if isinstance(widget, ttk.Button):
                            widget.config(state="normal")
    except:
        pass

def callback_slider_optimizado(param_name):
    """Callback optimizado para sliders con debounce"""
    def callback(valor):
        try:
            # Actualizar parametro local
            if param_name in ['umbral_binario', 'altura_roi', 'velocidad_base', 'velocidad_giro']:
                parametros_actuales[param_name] = int(float(valor))
            else:
                parametros_actuales[param_name] = float(valor)
            
            # Debounce - enviar solo cada 1 segundo
            if not hasattr(callback_slider_optimizado, 'ultimo_envio'):
                callback_slider_optimizado.ultimo_envio = {}
            
            tiempo_actual = time.time()
            ultimo = callback_slider_optimizado.ultimo_envio.get(param_name, 0)
            
            if tiempo_actual - ultimo > 1.0:  # 1 segundo de debounce
                robot.enviar_parametros(parametros_actuales)
                callback_slider_optimizado.ultimo_envio[param_name] = tiempo_actual
            
        except Exception as e:
            print(f"[ERROR] Callback slider {param_name}: {e}")
    
    return callback

def crear_interfaz_optimizada():
    """Crea interfaz optimizada para mejor rendimiento"""
    global root, video_label, status_label, mode_label, line_label, connection_label
    global auto_button, control_frame, sliders
    
    root = tk.Tk()
    root.title("Robot ESP32-CAM - Control Optimizado")
    root.geometry("1000x800")
    root.configure(bg='#f5f5f5')
    
    # Estilos simplificados
    style = ttk.Style()
    style.theme_use('clam')
    style.configure("Green.TButton", background="#28a745", foreground="white")
    style.configure("Red.TButton", background="#dc3545", foreground="white")
    style.configure("Blue.TButton", background="#007bff", foreground="white")
    
    # Frame principal
    main_frame = ttk.Frame(root, padding="10")
    main_frame.pack(fill="both", expand=True)
    
    # Titulo
    titulo = ttk.Label(main_frame, text="Robot Seguidor de Linea - ESP32-CAM Optimizado", 
                      font=("Arial", 14, "bold"))
    titulo.pack(pady=(0, 10))
    
    # Estado del sistema
    estado_frame = ttk.LabelFrame(main_frame, text="Estado del Sistema", padding="10")
    estado_frame.pack(fill="x", pady=(0, 10))
    
    # Estados en fila
    estados_row = ttk.Frame(estado_frame)
    estados_row.pack(fill="x")
    
    status_label = ttk.Label(estados_row, text="Estado: Iniciando...", 
                           background="#e3f2fd", relief="solid", padding=5)
    status_label.pack(side="left", padx=(0, 10))
    
    mode_label = ttk.Label(estados_row, text="Modo: Manual", 
                          background="#e8f5e8", relief="solid", padding=5)
    mode_label.pack(side="left", padx=(0, 10))
    
    connection_label = ttk.Label(estados_row, text="Conexion: Conectando...", 
                                background="#fff3cd", relief="solid", padding=5)
    connection_label.pack(side="right")
    
    line_label = ttk.Label(estado_frame, text="Linea: N/A", 
                          font=("Arial", 9), background="#f8f9fa", relief="solid", padding=5)
    line_label.pack(fill="x", pady=(10, 0))
    
    # Video
    video_frame = ttk.LabelFrame(main_frame, text="Camara con Deteccion", padding="10")
    video_frame.pack(fill="both", expand=True, pady=(0, 10))
    
    video_label = tk.Label(video_frame, bg="black", text="Conectando a ESP32-CAM...", 
                          fg="white", font=("Arial", 12))
    video_label.pack(fill="both", expand=True)
    
    # Controles principales
    controles_principales = ttk.LabelFrame(main_frame, text="Control Principal", padding="10")
    controles_principales.pack(fill="x", pady=(0, 10))
    
    botones_principales = ttk.Frame(controles_principales)
    botones_principales.pack()
    
    auto_button = ttk.Button(botones_principales, text="Activar IA", 
                            command=robot.alternar_modo_auto, style="Green.TButton", width=15)
    auto_button.pack(side="left", padx=(0, 10))
    
    stop_button = ttk.Button(botones_principales, text="PARAR", 
                           command=robot.parar_robot, style="Red.TButton", width=15)
    stop_button.pack(side="left")
    
    # Parametros simplificados
    params_frame = ttk.LabelFrame(main_frame, text="Parametros de Vision", padding="10")
    params_frame.pack(fill="x", pady=(0, 10))
    
    # Solo 3 parametros principales para reducir complejidad
    params_principales = {
        'umbral_binario': {'label': 'Umbral (0-255)', 'min': 0, 'max': 255, 'step': 5},
        'altura_roi': {'label': 'Altura ROI (px)', 'min': 20, 'max': 100, 'step': 5},
        'zona_muerta_centro': {'label': 'Zona Muerta', 'min': 0.05, 'max': 0.25, 'step': 0.01}
    }
    
    for i, (param, config) in enumerate(params_principales.items()):
        frame_param = ttk.Frame(params_frame)
        frame_param.pack(fill="x", pady=2)
        
        label = ttk.Label(frame_param, text=config['label'], width=15)
        label.pack(side="left")
        
        slider = tk.Scale(frame_param, from_=config['min'], to=config['max'], 
                         resolution=config['step'], orient="horizontal",
                         command=callback_slider_optimizado(param))
        slider.set(parametros_actuales[param])
        slider.pack(side="left", fill="x", expand=True, padx=(10, 10))
        
        valor_label = ttk.Label(frame_param, text=str(parametros_actuales[param]), width=8)
        valor_label.pack(side="right")
        
        sliders[param] = {'slider': slider, 'label': valor_label}
    
    # Controles manuales simplificados
    control_frame = ttk.LabelFrame(main_frame, text="Control Manual", padding="10")
    control_frame.pack(fill="x")
    
    # Instrucciones
    ttk.Label(control_frame, text="Teclado: W(adelante) A(izq) S(stop) D(der) Q(rotar-izq) E(rotar-der)", 
             font=("Arial", 9)).pack(pady=(0, 10))
    
    # Botones en grid simple
    botones_frame = ttk.Frame(control_frame)
    botones_frame.pack()
    
    botones = [
        ("Q Rotar Izq", "rotate_left", 0, 0),
        ("W Adelante", "forward", 0, 1),
        ("E Rotar Der", "rotate_right", 0, 2),
        ("A Izquierda", "left", 1, 0),
        ("S STOP", "stop", 1, 1),
        ("D Derecha", "right", 1, 2)
    ]
    
    for texto, cmd, fila, col in botones:
        estilo = "Red.TButton" if "STOP" in texto else "Blue.TButton"
        btn = ttk.Button(botones_frame, text=texto, 
                        command=lambda c=cmd: robot.enviar_movimiento(c),
                        style=estilo, width=12)
        btn.grid(row=fila, column=col, padx=2, pady=2)
    
    # Eventos de teclado
    def tecla_presionada(event):
        teclas = {
            'w': 'forward', 'a': 'left', 's': 'stop', 'd': 'right',
            'q': 'rotate_left', 'e': 'rotate_right'
        }
        if event.char.lower() in teclas:
            robot.enviar_movimiento(teclas[event.char.lower()])
    
    root.bind('<KeyPress>', tecla_presionada)
    root.focus_set()
    
    # Cierre de aplicacion
    def al_cerrar():
        print("[CIERRE] Cerrando aplicacion...")
        stop_stream_flag.set()
        robot.parar_robot()
        time.sleep(0.5)
        root.quit()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", al_cerrar)
    
    # Iniciar stream
    stream_thread = threading.Thread(target=hilo_stream_optimizado, daemon=True)
    stream_thread.start()
    
    # Enviar parametros iniciales tras conectar
    root.after(3000, lambda: robot.enviar_parametros(parametros_actuales))
    
    # Estado inicial
    actualizar_estado("Conectando...", "Manual")
    
    print("=" * 50)
    print("INTERFAZ OPTIMIZADA INICIADA")
    print(f"Conectando a ESP32-CAM: {ESP32_CAM_IP}")
    print("Configuracion optimizada para conexion estable")
    print("Timeouts aumentados para ESP32 ocupado")
    print("=" * 50)
    
    root.mainloop()

if __name__ == "__main__":
    print("=" * 60)
    print("CLIENTE PYTHON OPTIMIZADO - ROBOT ESP32-CAM")
    print("=" * 60)
    print(f"IP ESP32-CAM: {ESP32_CAM_IP}")
    print("IMPORTANTE: Verifica que la IP sea correcta")
    print("Si no conecta, cambia la IP en la linea 25")
    print()
    print("OPTIMIZACIONES:")
    print("• Timeouts aumentados para ESP32 ocupado")
    print("• Comunicacion HTTP con reintentos")
    print("• Procesamiento de imagen simplificado")
    print("• Interfaz ligera para mejor rendimiento")
    print("• Sincronizacion optimizada con cores ESP32")
    print("=" * 60)
    
    try:
        crear_interfaz_optimizada()
    except KeyboardInterrupt:
        print("\n[CTRL+C] Aplicacion terminada por usuario")
    except Exception as e:
        print(f"\n[ERROR] Error critico: {e}")
        messagebox.showerror("Error Critico", f"Error en la aplicacion: {e}")
    finally:
        print("[FIN] Aplicacion terminada")