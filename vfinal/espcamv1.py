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

# --- Configuración del Robot ---
ESP32_CAM_IP = "192.168.196.182"  # ¡¡¡CAMBIA ESTO POR LA IP DE TU ESP32-CAM!!!
BASE_URL = f"http://{ESP32_CAM_IP}"
STREAM_URL = f"{BASE_URL}/stream"
AUTO_MODE_URL = f"{BASE_URL}/auto"
STOP_URL = f"{BASE_URL}/stop"
MOVE_URL = f"{BASE_URL}/move"

# --- Variables Globales ---
root = None
video_label = None
binary_label = None
status_label = None
mode_status_label = None
line_status_label = None
auto_mode_var = False
stop_stream_flag = threading.Event()
auto_mode_button = None
control_frame = None

# --- Parámetros de Procesamiento ---
THRESHOLD_VAL = 80
ROI_HEIGHT = 60

class RobotController:
    def __init__(self):
        self.connected = False
        self.auto_mode = False
        self.last_command_time = time.time()
        
    def send_json_command(self, url, data):
        """Envía comando JSON al ESP32"""
        try:
            headers = {'Content-Type': 'application/json'}
            response = requests.post(url, json=data, headers=headers, timeout=5)
            response.raise_for_status()
            print(f"✅ Comando enviado a {url}: {response.text}")
            self.connected = True
            return response.json() if response.text else {"status": "ok"}
        except requests.exceptions.ConnectionError:
            self.connected = False
            self.show_connection_error()
            return None
        except requests.exceptions.Timeout:
            messagebox.showerror("Timeout", "Tiempo de espera agotado")
            return None
        except Exception as e:
            messagebox.showerror("Error", f"Error enviando comando: {e}")
            return None
    
    def show_connection_error(self):
        messagebox.showerror("Error de Conexión", 
                           f"No se pudo conectar al ESP32-CAM en {ESP32_CAM_IP}\n"
                           f"Verifica:\n"
                           f"• La IP en el código Python\n"
                           f"• Que el ESP32 esté encendido\n"
                           f"• La conexión Wi-Fi")
    
    def send_move_command(self, direction, speed=150):
        """Envía comando de movimiento"""
        if self.auto_mode and direction != "stop":
            messagebox.showinfo("Modo Automático", 
                              "Robot en modo automático. Use STOP para detener o desactive el modo automático.")
            return False
        
        data = {"direction": direction, "speed": speed}
        result = self.send_json_command(MOVE_URL, data)
        
        if result:
            self.last_command_time = time.time()
            update_status_labels(f"Movimiento: {direction.replace('_', ' ').title()}", 
                                "Manual" if not self.auto_mode else "Automático")
            return True
        return False
    
    def toggle_auto_mode(self):
        """Alterna modo automático"""
        new_mode = not self.auto_mode
        data = {"auto": new_mode}
        result = self.send_json_command(AUTO_MODE_URL, data)
        
        if result:
            self.auto_mode = new_mode
            update_auto_mode_ui()
            mode_text = "Automático" if self.auto_mode else "Manual"
            update_status_labels("Modo cambiado", mode_text)
            print(f"🤖 Modo automático: {'Activado' if self.auto_mode else 'Desactivado'}")
            return True
        return False
    
    def send_stop_command(self):
        """Envía comando de parada"""
        result = self.send_json_command(STOP_URL, {})
        if result:
            self.auto_mode = False  # Stop siempre desactiva modo automático
            update_auto_mode_ui()
            update_status_labels("Detenido", "Manual")
            print("🛑 Robot detenido")
            return True
        return False

# Instancia global del controlador
robot = RobotController()

def process_frame_for_display(frame):
    """Procesa frame para mostrar detección de línea"""
    try:
        # Convertir a escala de grises si no lo está
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        else:
            gray = frame.copy()
        
        h, w = gray.shape
        
        # Definir ROI
        roi_start_y = max(0, h - ROI_HEIGHT)
        roi = gray[roi_start_y:h, 0:w]
        
        # Binarizar ROI
        _, binary_roi = cv2.threshold(roi, THRESHOLD_VAL, 255, cv2.THRESH_BINARY)
        
        # Crear frame display
        display_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        
        # Dibujar ROI
        cv2.rectangle(display_frame, (0, roi_start_y), (w-1, h-1), (0, 255, 0), 2)
        cv2.putText(display_frame, "ROI", (10, roi_start_y-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Detectar línea
        line_position = 0.0
        line_detected = False
        
        # Encontrar contornos
        contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Contorno más grande
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 50:  # Mínimo área
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + roi_start_y
                    
                    # Calcular posición normalizada
                    line_position = (cx - w/2) / (w/2)
                    line_detected = True
                    
                    # Dibujar centroide
                    cv2.circle(display_frame, (cx, cy), 8, (255, 0, 0), -1)
                    cv2.putText(display_frame, f"Centro: ({cx},{cy})", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    cv2.putText(display_frame, f"Pos: {line_position:.3f}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Crear frame binario completo para display
        binary_full = np.zeros_like(gray)
        binary_full[roi_start_y:h, 0:w] = binary_roi
        binary_display = cv2.cvtColor(binary_full, cv2.COLOR_GRAY2RGB)
        
        # Actualizar estado de línea
        if line_status_label:
            status = "✅ Detectada" if line_detected else "❌ No detectada"
            pos_text = f"Posición: {line_position:.3f}" if line_detected else "Posición: N/A"
            line_status_label.config(text=f"Línea: {status} | {pos_text}")
        
        return display_frame, binary_display
        
    except Exception as e:
        print(f"Error procesando frame: {e}")
        # Retornar frames vacíos en caso de error
        empty = np.zeros((240, 320, 3), dtype=np.uint8)
        return empty, empty

def stream_video():
    """Hilo para recibir y mostrar video"""
    print("🎥 Iniciando stream de video...")
    consecutive_errors = 0
    
    while not stop_stream_flag.is_set():
        try:
            with requests.get(STREAM_URL, stream=True, timeout=10) as r:
                r.raise_for_status()
                robot.connected = True
                consecutive_errors = 0
                
                bytes_data = b''
                for chunk in r.iter_content(chunk_size=1024):
                    if stop_stream_flag.is_set():
                        break
                        
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')  # JPEG start
                    b = bytes_data.find(b'\xff\xd9')  # JPEG end
                    
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        
                        try:
                            # Decodificar imagen
                            img = Image.open(io.BytesIO(jpg))
                            img_np = np.array(img)
                            
                            # Procesar frame
                            processed_original, processed_binary = process_frame_for_display(img_np)
                            
                            # Mostrar en GUI
                            display_frames(processed_original, processed_binary)
                            
                        except Exception as e:
                            print(f"Error procesando frame: {e}")
                            
        except requests.exceptions.ConnectionError:
            consecutive_errors += 1
            robot.connected = False
            if consecutive_errors == 1:  # Solo mostrar el primer error
                print(f"❌ Conexión perdida con {ESP32_CAM_IP}")
            time.sleep(3)
            
        except Exception as e:
            consecutive_errors += 1
            print(f"Error en stream: {e}")
            time.sleep(2)
    
    print("🎥 Stream de video detenido")

def display_frames(original, binary):
    """Muestra frames en la GUI"""
    try:
        if video_label and video_label.winfo_exists():
            # Redimensionar manteniendo aspecto
            target_width = 400
            h, w = original.shape[:2]
            ratio = target_width / w
            new_h = int(h * ratio)
            
            # Redimensionar imágenes
            img_orig_resized = cv2.resize(original, (target_width, new_h))
            img_bin_resized = cv2.resize(binary, (target_width, new_h))
            
            # Convertir a PIL y mostrar
            img_orig_pil = Image.fromarray(img_orig_resized)
            img_bin_pil = Image.fromarray(img_bin_resized)
            
            photo_orig = ImageTk.PhotoImage(image=img_orig_pil)
            photo_bin = ImageTk.PhotoImage(image=img_bin_pil)
            
            video_label.config(image=photo_orig)
            video_label.image = photo_orig
            
            binary_label.config(image=photo_bin)
            binary_label.image = photo_bin
            
    except Exception as e:
        print(f"Error mostrando frames: {e}")

def update_status_labels(robot_status, mode_status):
    """Actualiza etiquetas de estado"""
    try:
        if status_label and status_label.winfo_exists():
            status_label.config(text=f"Estado: {robot_status}")
        if mode_status_label and mode_status_label.winfo_exists():
            mode_status_label.config(text=f"Modo: {mode_status}")
    except:
        pass

def update_auto_mode_ui():
    """Actualiza interfaz según modo automático"""
    try:
        if auto_mode_button and auto_mode_button.winfo_exists():
            if robot.auto_mode:
                auto_mode_button.config(text="🛑 Desactivar IA", style="Red.TButton")
                if control_frame:
                    for widget in control_frame.winfo_children():
                        if isinstance(widget, ttk.Button) and widget != auto_mode_button:
                            widget.config(state="disabled")
            else:
                auto_mode_button.config(text="🤖 Activar IA", style="Green.TButton")
                if control_frame:
                    for widget in control_frame.winfo_children():
                        if isinstance(widget, ttk.Button):
                            widget.config(state="normal")
    except:
        pass

def create_gui():
    """Crea la interfaz gráfica"""
    global root, video_label, binary_label, status_label, mode_status_label
    global line_status_label, auto_mode_button, control_frame
    
    root = tk.Tk()
    root.title("🤖 Robot Seguidor de Línea - ESP32-CAM")
    root.geometry("1200x900")
    root.configure(bg='#f0f0f0')
    
    # Estilos
    style = ttk.Style()
    style.theme_use('clam')
    
    # Estilos personalizados
    style.configure("Title.TLabel", font=("Arial", 16, "bold"), background='#f0f0f0')
    style.configure("Status.TLabel", font=("Arial", 11), background='#e3f2fd', relief="raised", padding=5)
    style.configure("Green.TButton", background="#28a745", foreground="white", font=("Arial", 11, "bold"))
    style.configure("Red.TButton", background="#dc3545", foreground="white", font=("Arial", 11, "bold"))
    style.configure("Blue.TButton", background="#007bff", foreground="white", font=("Arial", 10))
    
    # Frame principal
    main_frame = ttk.Frame(root, padding="15")
    main_frame.pack(fill="both", expand=True)
    
    # Título
    title_label = ttk.Label(main_frame, text="🤖 Robot Seguidor de Línea ESP32-CAM", style="Title.TLabel")
    title_label.pack(pady=(0, 15))
    
    # Frame de estado
    status_frame = ttk.LabelFrame(main_frame, text="📊 Estado del Sistema", padding="10")
    status_frame.pack(fill="x", pady=(0, 15))
    
    status_info_frame = ttk.Frame(status_frame)
    status_info_frame.pack(fill="x")
    
    status_label = ttk.Label(status_info_frame, text="Estado: Iniciando...", style="Status.TLabel")
    status_label.pack(side="left", padx=(0, 10))
    
    mode_status_label = ttk.Label(status_info_frame, text="Modo: Manual", style="Status.TLabel")
    mode_status_label.pack(side="left", padx=(0, 10))
    
    line_status_label = ttk.Label(status_info_frame, text="Línea: N/A", style="Status.TLabel")
    line_status_label.pack(side="right")
    
    # Frame para videos
    video_container = ttk.Frame(main_frame)
    video_container.pack(fill="both", expand=True, pady=(0, 15))
    
    # Video original
    original_frame = ttk.LabelFrame(video_container, text="📹 Cámara con Detección", padding="10")
    original_frame.pack(side="left", fill="both", expand=True, padx=(0, 7))
    
    video_label = tk.Label(original_frame, bg="black", text="Conectando a la cámara...", 
                          fg="white", font=("Arial", 12))
    video_label.pack(fill="both", expand=True)
    
    # Video binario
    binary_frame = ttk.LabelFrame(video_container, text="🔍 Imagen Binarizada", padding="10")
    binary_frame.pack(side="right", fill="both", expand=True, padx=(7, 0))
    
    binary_label = tk.Label(binary_frame, bg="black", text="Procesamiento de imagen...", 
                           fg="white", font=("Arial", 12))
    binary_label.pack(fill="both", expand=True)
    
    # Controles principales
    main_controls = ttk.LabelFrame(main_frame, text="🎮 Control Principal", padding="15")
    main_controls.pack(fill="x", pady=(0, 10))
    
    main_buttons_frame = ttk.Frame(main_controls)
    main_buttons_frame.pack()
    
    auto_mode_button = ttk.Button(main_buttons_frame, text="🤖 Activar IA", 
                                 command=robot.toggle_auto_mode, style="Green.TButton", width=20)
    auto_mode_button.pack(side="left", padx=(0, 15))
    
    stop_button = ttk.Button(main_buttons_frame, text="🛑 DETENER TODO", 
                            command=robot.send_stop_command, style="Red.TButton", width=20)
    stop_button.pack(side="left")
    
    # Controles manuales
    control_frame = ttk.LabelFrame(main_frame, text="🕹️ Control Manual", padding="15")
    control_frame.pack(fill="x")
    
    # Grid de botones
    grid_frame = ttk.Frame(control_frame)
    grid_frame.pack()
    
    # Definir comandos y sus etiquetas
    buttons_config = [
        ("⤺ Rotar Izq (Q)", "rotate_left", 0, 0),
        ("⬆️ Adelante (W)", "forward", 0, 1),
        ("⤻ Rotar Der (E)", "rotate_right", 0, 2),
        ("⬅️ Izquierda (A)", "left", 1, 0),
        ("⏸️ STOP (S)", "stop", 1, 1),
        ("➡️ Derecha (D)", "right", 1, 2),
        ("", "", 2, 0),  # Espacio vacío
        ("⬇️ Atrás (X)", "backward", 2, 1),
        ("", "", 2, 2),  # Espacio vacío
    ]
    
    for text, command, row, col in buttons_config:
        if text:  # Solo crear botón si hay texto
            style_name = "Red.TButton" if "STOP" in text else "Blue.TButton"
            btn = ttk.Button(grid_frame, text=text, 
                           command=lambda cmd=command: robot.send_move_command(cmd),
                           style=style_name, width=15)
            btn.grid(row=row, column=col, padx=5, pady=5)
    
    # Bindings de teclado
    def on_key_press(event):
        key_commands = {
            'w': 'forward', 'a': 'left', 's': 'stop', 'd': 'right',
            'q': 'rotate_left', 'e': 'rotate_right', 'x': 'backward'
        }
        if event.char.lower() in key_commands:
            robot.send_move_command(key_commands[event.char.lower()])
    
    root.bind('<KeyPress>', on_key_press)
    root.focus_set()  # Para recibir eventos de teclado
    
    # Información de IP
    ip_label = ttk.Label(main_frame, text=f"🌐 IP del Robot: {ESP32_CAM_IP}", 
                        font=("Arial", 10), background='#f0f0f0')
    ip_label.pack(pady=(10, 0))
    
    # Protocolo de cierre
    def on_closing():
        print("🔄 Cerrando aplicación...")
        stop_stream_flag.set()
        robot.send_stop_command()
        time.sleep(0.5)
        root.quit()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Iniciar thread de video
    video_thread = threading.Thread(target=stream_video, daemon=True)
    video_thread.start()
    
    # Estado inicial
    update_status_labels("Conectando...", "Manual")
    
    print("🚀 Interfaz iniciada")
    print(f"🌐 Conectando a: {ESP32_CAM_IP}")
    print("⌨️  Controles de teclado: W(adelante) A(izq) S(stop) D(der) Q(rotar izq) E(rotar der) X(atrás)")
    
    root.mainloop()

if __name__ == "__main__":
    print("=" * 60)
    print("🤖 ROBOT SEGUIDOR DE LÍNEA - CLIENTE PYTHON")
    print("=" * 60)
    print(f"🌐 ESP32-CAM IP: {ESP32_CAM_IP}")
    print("🔧 Si no conecta, verifica la IP en la línea 11 del código")
    print("=" * 60)
    
    create_gui()