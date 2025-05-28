import cv2
import numpy as np
import requests
import time
from PIL import Image, ImageTk
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import io

# --- Configuración del Robot ---
ESP32_CAM_IP = "192.168.196.182"  # ¡¡¡CAMBIA ESTO POR LA IP DE TU ESP32-CAM!!!
STREAM_URL = f"http://{ESP32_CAM_IP}/stream"
AUTO_MODE_URL = f"http://{ESP32_CAM_IP}/auto"
STOP_URL = f"http://{ESP32_CAM_IP}/stop"
MOVE_URL = f"http://{ESP32_CAM_IP}/move" # Mantener para comandos manuales, aunque el robot decide en auto

# --- Variables Globales para la Interfaz y Control ---
root = None
video_label = None
binary_label = None
status_label = None
mode_status_label = None
auto_mode_var = False
stop_stream_flag = threading.Event() # Evento para detener el hilo del stream
auto_mode_button = None # Añadimos esta declaración global
stop_button = None # Añadimos esta declaración global

# --- Parámetros de Procesamiento de Visión en PC (solo para visualización) ---
THRESHOLD_VAL = 80 # Debe coincidir con el del ESP32 para una representación precisa
ROI_HEIGHT = 60    # Debe coincidir con el del ESP32
ROWS_TO_ANALYZE = 15 # Debe coincidir con el del ESP32

# --- Funciones de Comunicación con el Robot ---

def send_auto_mode(enable):
    """Envía el comando para activar/desactivar el modo automático."""
    global auto_mode_var
    try:
        data = {"auto": enable}
        response = requests.post(AUTO_MODE_URL, json=data, timeout=2)
        if response.status_code == 200:
            auto_mode_var = enable
            update_status("Modo automático: " + ("ACTIVADO" if enable else "DESACTIVADO"))
            update_mode_status()
        else:
            update_status(f"Error al cambiar modo auto: {response.status_code}")
    except requests.exceptions.RequestException as e:
        update_status(f"Error de conexión al cambiar modo: {e}")

def send_stop_command():
    """Envía el comando de detener al robot."""
    try:
        response = requests.post(STOP_URL, timeout=2)
        if response.status_code == 200:
            update_status("Comando de STOP enviado.")
            # Asegurarse de que el botón de auto refleje el estado real
            send_auto_mode(False) 
        else:
            update_status(f"Error al enviar STOP: {response.status_code}")
    except requests.exceptions.RequestException as e:
        update_status(f"Error de conexión al enviar STOP: {e}")

def send_manual_move_command(direction, speed=150):
    """Envía un comando de movimiento manual. (Sólo si el robot no está en modo auto)"""
    if auto_mode_var:
        update_status("Robot en modo automático. No se puede mover manualmente.")
        return
    try:
        data = {"direction": direction, "speed": speed}
        response = requests.post(MOVE_URL, json=data, timeout=1)
        if response.status_code == 200:
            update_status(f"Movimiento manual: {direction} (velocidad: {speed})")
        else:
            update_status(f"Error al enviar movimiento: {response.status_code}")
    except requests.exceptions.RequestException as e:
        update_status(f"Error de conexión al enviar movimiento: {e}")

# --- Funciones de Procesamiento de Visión en PC para Visualización ---

def process_frame_for_display(frame):
    """
    Procesa un frame gris para binarización y detección de línea (solo para visualización).
    Dibuja la línea detectada en el frame binario.
    """
    if frame is None:
        return None, None

    # El stream de la ESP32-CAM ahora es en escala de grises
    gray_frame = frame 
    height, width = gray_frame.shape[:2]

    # Binarización (debe coincidir con el umbral del ESP32)
    _, binary_frame = cv2.threshold(gray_frame, THRESHOLD_VAL, 255, cv2.THRESH_BINARY)
    
    # Crear una copia para dibujar sin modificar el original
    binary_display = cv2.cvtColor(binary_frame, cv2.COLOR_GRAY2BGR) # Convertir a BGR para dibujar colores

    # Definir ROI (debe coincidir con el del ESP32)
    roi_start_y = height - ROI_HEIGHT
    if roi_start_y < 0: roi_start_y = 0

    # Extraer la ROI principal
    roi_main = binary_frame[roi_start_y:height, 0:width]

    line_pixels_x_sum = 0
    line_pixels_count = 0

    # Analiza las últimas filas de la ROI (más cercanas al robot)
    current_roi_height = roi_main.shape[0]
    for y_offset in range(current_roi_height - min(ROWS_TO_ANALYZE, current_roi_height), current_roi_height):
        for x in range(width):
            if roi_main[y_offset, x] > 0: # Si es un píxel blanco (línea)
                line_pixels_x_sum += x
                line_pixels_count += 1

    line_center_x = -1 # Valor por defecto si no se detecta línea
    if line_pixels_count > (width * min(ROWS_TO_ANALYZE, current_roi_height) * 0.02): # Mínimo 2% de píxeles
        line_center_x = int(line_pixels_x_sum / line_pixels_count)
        # Dibujar el centro de la línea en la imagen binarizada para depuración
        center_y_draw = int(roi_start_y + current_roi_height / 2) # Dibuja en el centro de la ROI
        cv2.line(binary_display, (line_center_x, roi_start_y), (line_center_x, height), (0, 255, 0), 2) # Línea verde

    # Dibujar el rectángulo de la ROI principal
    cv2.rectangle(binary_display, (0, roi_start_y), (width - 1, height - 1), (0, 0, 255), 1) # Línea roja para ROI

    return gray_frame, binary_display

# --- Funciones de la Interfaz Gráfica (Tkinter) ---

def update_status(message):
    """Actualiza el mensaje de estado en la interfaz."""
    if status_label:
        status_label.config(text=f"Estado: {message}")

def update_mode_status():
    """Actualiza el estado del modo (automático/manual) en la interfaz."""
    if mode_status_label:
        mode_status_label.config(text=f"Modo: {'Automático (IA)' if auto_mode_var else 'Manual'}")
    
    # Actualizar el estilo del botón de modo automático
    if auto_mode_button:
        if auto_mode_var:
            auto_mode_button.config(text="Desactivar IA", style="Red.TButton")
        else:
            auto_mode_button.config(text="Activar IA", style="Green.TButton")


def video_stream_thread():
    """Hilo para capturar y procesar el stream de video del ESP32-CAM."""
    global video_label, binary_label

    try:
        # Aumentar timeout para la conexión inicial si es necesario
        response = requests.get(STREAM_URL, stream=True, timeout=10) 
        if response.status_code != 200:
            update_status(f"Error al iniciar stream: {response.status_code}")
            return

        bytes_buffer = b''
        for chunk in response.iter_content(chunk_size=1024):
            if stop_stream_flag.is_set():
                break

            bytes_buffer += chunk
            a = bytes_buffer.find(b'\xff\xd8') # JPEG start
            b = bytes_buffer.find(b'\xff\xd9') # JPEG end
            if a != -1 and b != -1:
                jpg = bytes_buffer[a:b+2]
                bytes_buffer = bytes_buffer[b+2:]
                
                try:
                    # Convertir bytes a imagen OpenCV
                    img_np = np.frombuffer(jpg, dtype=np.uint8)
                    frame = cv2.imdecode(img_np, cv2.IMREAD_GRAYSCALE) # Leer directamente como GRIS

                    if frame is not None:
                        # Procesar para visualización
                        gray_frame_display, binary_frame_display = process_frame_for_display(frame)

                        # Mostrar frame original (gris)
                        img_rgb = Image.fromarray(gray_frame_display)
                        img_tk = ImageTk.PhotoImage(image=img_rgb)
                        video_label.imgtk = img_tk
                        video_label.config(image=img_tk)

                        # Mostrar frame binarizado con detección de línea
                        img_binary_rgb = Image.fromarray(binary_frame_display)
                        img_binary_tk = ImageTk.PhotoImage(image=img_binary_rgb)
                        binary_label.imgtk = img_binary_tk
                        binary_label.config(image=img_binary_tk)

                except Exception as e:
                    print(f"Error al procesar frame: {e}")

    except requests.exceptions.RequestException as e:
        update_status(f"Error de conexión con el stream: {e}")
    finally:
        update_status("Stream detenido.")
        print("Hilo de stream terminado.")

def start_stream():
    """Inicia el hilo del stream de video."""
    stop_stream_flag.clear()
    threading.Thread(target=video_stream_thread, daemon=True).start()
    update_status("Iniciando stream...")

def stop_all():
    """Detiene el stream y envía el comando de STOP al robot."""
    stop_stream_flag.set()
    send_stop_command()

def on_closing():
    """Maneja el cierre de la ventana de la interfaz."""
    if messagebox.askokcancel("Salir", "¿Estás seguro de que quieres salir y detener todo?"):
        stop_all() # Detener el stream y el robot
        root.destroy()

# --- Configuración de la Interfaz Gráfica ---

def setup_gui():
    """Configura la ventana principal de la interfaz gráfica."""
    global root, video_label, binary_label, status_label, mode_status_label, auto_mode_button, stop_button

    root = tk.Tk()
    root.title("Control Robot Omnidireccional - ESP32-CAM")
    root.geometry("1000x700") # Aumentar el tamaño para más espacio

    # *** CONFIGURE STYLES FOR TTK BUTTONS ***
    style = ttk.Style()
    
    # Configure a style for the green button
    style.configure("Green.TButton", background="green", foreground="white", font=("Arial", 10, "bold"))
    style.map("Green.TButton",
              background=[('active', 'lightgreen'), ('!disabled', 'green')],
              foreground=[('active', 'black'), ('!disabled', 'white')])

    # Configure a style for the red button
    style.configure("Red.TButton", background="red", foreground="white", font=("Arial", 10, "bold"))
    style.map("Red.TButton",
              background=[('active', 'lightcoral'), ('!disabled', 'red')],
              foreground=[('active', 'black'), ('!disabled', 'white')])
    
    # Configure a style for general manual control buttons (optional, but good practice)
    style.configure("Manual.TButton", font=("Arial", 10))
    # ****************************************

    # Frame principal
    main_frame = ttk.Frame(root, padding="10")
    main_frame.pack(fill="both", expand=True)

    # Título
    ttk.Label(main_frame, text="Control y Visión del Robot ESP32-CAM", font=("Arial", 16, "bold")).pack(pady=10)

    # Estado y modo
    status_frame = ttk.LabelFrame(main_frame, text="Estado del Sistema", padding="10")
    status_frame.pack(pady=10, fill="x")
    status_label = ttk.Label(status_frame, text="Estado: Conectando...", font=("Arial", 10))
    status_label.pack(side="left", padx=5)
    mode_status_label = ttk.Label(status_frame, text="Modo: Manual", font=("Arial", 10, "bold"))
    mode_status_label.pack(side="right", padx=5)
    
    # Controles de Modo
    mode_control_frame = ttk.LabelFrame(main_frame, text="Control de Modo", padding="10")
    mode_control_frame.pack(pady=10, fill="x")

    auto_mode_button = ttk.Button(mode_control_frame, text="Activar IA", command=lambda: send_auto_mode(not auto_mode_var), style="Green.TButton")
    auto_mode_button.pack(side="left", padx=5, expand=True, fill="x")
    
    stop_button = ttk.Button(mode_control_frame, text="DETENER TODO", command=stop_all, style="Red.TButton")
    stop_button.pack(side="left", padx=5, expand=True, fill="x")

    # Controles Manuales (deshabilitados si está en modo automático)
    manual_control_frame = ttk.LabelFrame(main_frame, text="Control Manual (Solo en modo Manual)", padding="10")
    manual_control_frame.pack(pady=10, fill="x")

    # Uso de grid para organizar mejor los botones manuales
    manual_control_frame.columnconfigure(0, weight=1)
    manual_control_frame.columnconfigure(1, weight=1)
    manual_control_frame.columnconfigure(2, weight=1)

    ttk.Button(manual_control_frame, text="↑ Adelante (W)", command=lambda: send_manual_move_command("forward"), style="Manual.TButton").grid(row=0, column=1, padx=5, pady=5, sticky="ew")
    ttk.Button(manual_control_frame, text="← Izquierda (A)", command=lambda: send_manual_move_command("left"), style="Manual.TButton").grid(row=1, column=0, padx=5, pady=5, sticky="ew")
    ttk.Button(manual_control_frame, text="→ Derecha (D)", command=lambda: send_manual_move_command("right"), style="Manual.TButton").grid(row=1, column=2, padx=5, pady=5, sticky="ew")
    ttk.Button(manual_control_frame, text="↓ Atrás (S)", command=lambda: send_manual_move_command("backward"), style="Manual.TButton").grid(row=2, column=1, padx=5, pady=5, sticky="ew")
    ttk.Button(manual_control_frame, text="Girar Izq (Q)", command=lambda: send_manual_move_command("rotate_left"), style="Manual.TButton").grid(row=0, column=0, padx=5, pady=5, sticky="ew")
    ttk.Button(manual_control_frame, text="Girar Der (E)", command=lambda: send_manual_move_command("rotate_right"), style="Manual.TButton").grid(row=0, column=2, padx=5, pady=5, sticky="ew")
    ttk.Button(manual_control_frame, text="STOP Manual (Space)", command=lambda: send_manual_move_command("stop", 0), style="Manual.TButton").grid(row=1, column=1, padx=5, pady=5, sticky="ew")

    # Frame para los videos
    video_frames_container = ttk.Frame(main_frame)
    video_frames_container.pack(pady=10, fill="both", expand=True)

    # Frame para el video original
    original_video_frame = ttk.LabelFrame(video_frames_container, text="Stream Original (Gris)", padding="10")
    original_video_frame.pack(side="left", fill="both", expand=True, padx=5)
    video_label = tk.Label(original_video_frame)
    video_label.pack(fill="both", expand=True)

    # Frame para el video binarizado
    binary_video_frame = ttk.LabelFrame(video_frames_container, text="Imagen Binarizada + Detección", padding="10")
    binary_video_frame.pack(side="right", fill="both", expand=True, padx=5)
    binary_label = tk.Label(binary_video_frame)
    binary_label.pack(fill="both", expand=True)
    
    # Manejo de teclas para control manual
    root.bind("<w>", lambda event: send_manual_move_command("forward"))
    root.bind("<a>", lambda event: send_manual_move_command("left"))
    root.bind("<s>", lambda event: send_manual_move_command("backward"))
    root.bind("<d>", lambda event: send_manual_move_command("right"))
    root.bind("<q>", lambda event: send_manual_move_command("rotate_left"))
    root.bind("<e>", lambda event: send_manual_move_command("rotate_right"))
    root.bind("<space>", lambda event: send_manual_move_command("stop", 0))

    root.protocol("WM_DELETE_WINDOW", on_closing)

    start_stream()
    update_mode_status() # Estado inicial del modo

    root.mainloop()

if __name__ == "__main__":
    setup_gui()