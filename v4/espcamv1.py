import cv2
import numpy as np
import requests
import time
from PIL import Image, ImageTk
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import io

# --- Configuracion del Robot ---
ESP32_CAM_IP = "192.168.196.182"  # ¡¡¡CAMBIA ESTO POR LA IP DE TU ESP32-CAM!!!
STREAM_URL = f"http://{ESP32_CAM_IP}/stream"
AUTO_MODE_URL = f"http://{ESP32_CAM_IP}/auto"
STOP_URL = f"http://{ESP32_CAM_IP}/stop"
MOVE_URL = f"http://{ESP32_CAM_IP}/move"

# --- Variables Globales para la Interfaz y Control ---
root = None
video_label = None
binary_label = None
status_label = None
mode_status_label = None
auto_mode_var = False
stop_stream_flag = threading.Event() # Evento para detener el hilo del stream
auto_mode_button = None
stop_button = None

# --- Parametros de Procesamiento de Vision en PC (solo para visualizacion) ---
THRESHOLD_VAL = 80 # Debe coincidir con el del ESP32 para una representacion precisa
ROI_HEIGHT = 60    # Debe coincidir con el del ESP32 para una representacion precisa

# --- Funciones de Control del Robot ---

def send_command(url, params=None):
    """Envio generico de comandos HTTP GET al ESP32."""
    try:
        response = requests.get(url, params=params, timeout=5)
        response.raise_for_status() # Lanza un error para codigos de estado HTTP malos
        print(f"Comando enviado a {url} con parametros {params}: {response.text}")
        return response.text
    except requests.exceptions.ConnectionError:
        messagebox.showerror("Error de Conexión", f"No se pudo conectar al ESP32-CAM en {ESP32_CAM_IP}. Verifica la IP y la conexion Wi-Fi.")
        print(f"ERROR: No se pudo conectar a {ESP32_CAM_IP}")
        return None
    except requests.exceptions.Timeout:
        messagebox.showerror("Error de Tiempo de Espera", "La solicitud al ESP32-CAM ha excedido el tiempo de espera.")
        print(f"ERROR: Tiempo de espera excedido al conectar a {ESP32_CAM_IP}")
        return None
    except requests.exceptions.RequestException as e:
        messagebox.showerror("Error de Solicitud", f"Error al enviar el comando al ESP32-CAM: {e}")
        print(f"ERROR: Error de solicitud HTTP: {e}")
        return None

def send_manual_move_command(command):
    """Envia un comando de movimiento manual al ESP32."""
    global auto_mode_var
    if auto_mode_var and command != "stop":
        messagebox.showinfo("Modo Automatico Activo", "No se pueden enviar comandos manuales mientras el robot esta en modo automatico. Desactiva el modo automatico primero o usa el boton STOP.")
        return

    response_text = send_command(MOVE_URL, params={"command": command})
    if response_text:
        # Actualizar el estado del robot en la GUI
        update_status_labels(f"Movimiento: {command.replace('_', ' ').capitalize()}", "Manual" if not auto_mode_var else "Automatico")
        if command == "stop":
             update_status_labels("Detenido", "Manual" if not auto_mode_var else "Automatico")


def toggle_auto_mode():
    """Alterna el modo automatico/manual del robot."""
    global auto_mode_var
    new_state = not auto_mode_var
    enable_param = "true" if new_state else "false"
    response_text = send_command(AUTO_MODE_URL, params={"enable": enable_param})

    if response_text:
        auto_mode_var = new_state
        if auto_mode_var:
            auto_mode_button.config(text="Desactivar Modo Automatico", style="Red.TButton")
            update_status_labels("Movimiento Automatico", "Automatico")
        else:
            auto_mode_button.config(text="Activar Modo Automatico", style="Green.TButton")
            update_status_labels("Detenido", "Manual")
        print(f"Modo Automatico: {'Activado' if auto_mode_var else 'Desactivado'}")

def send_stop_command():
    """Envia el comando de detener al robot."""
    global auto_mode_var
    response_text = send_command(STOP_URL)
    if response_text:
        auto_mode_var = False # Al detener, siempre vuelve a manual
        auto_mode_button.config(text="Activar Modo Automatico", style="Green.TButton")
        update_status_labels("Detenido", "Manual")
        print("Comando STOP enviado.")

# --- Funciones de Procesamiento de Video ---

def process_frame(frame):
    """
    Procesa un frame para deteccion de lineas (ejemplo simplificado).
    Returns: frame binarizado y frame con deteccion.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Definir la Region de Interes (ROI) en la parte inferior de la imagen
    h, w = gray.shape
    roi_start_y = h - ROI_HEIGHT
    roi = gray[roi_start_y:h, 0:w]

    # Binarizar la ROI
    _, binary_roi = cv2.threshold(roi, THRESHOLD_VAL, 255, cv2.THRESH_BINARY)

    # Encontrar contornos en la ROI binarizada (simplificado para deteccion visual)
    contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    binary_full_frame = np.zeros_like(gray)
    binary_full_frame[roi_start_y:h, 0:w] = binary_roi # Poner la ROI binarizada en un frame completo

    display_frame = frame.copy() # Copia para dibujar sobre ella

    # Dibujar la ROI en el frame original para visualizacion
    cv2.rectangle(display_frame, (0, roi_start_y), (w-1, h-1), (0, 255, 0), 2)

    if contours:
        # Encontrar el contorno mas grande (asumiendo que es la linea principal)
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + roi_start_y # Ajustar Y a coordenadas del frame completo

            # Dibujar el centroide
            cv2.circle(display_frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(display_frame, f"Centro: ({cx},{cy})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

    # Convertir los frames a formato RGB para PIL
    display_frame_rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB) # Asegurar que es RGB
    binary_full_frame_rgb = cv2.cvtColor(binary_full_frame, cv2.COLOR_GRAY2RGB)

    return display_frame_rgb, binary_full_frame_rgb

def stream_video():
    """Hilo para recibir y mostrar el stream de video."""
    print("Iniciando stream de video...")
    while not stop_stream_flag.is_set():
        try:
            # Usar stream=True para mantener la conexion abierta y leer chunks
            with requests.get(STREAM_URL, stream=True, timeout=10) as r:
                r.raise_for_status()
                bytes_data = b''
                for chunk in r.iter_content(chunk_size=1024):
                    if stop_stream_flag.is_set():
                        break
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8') # JPEG start
                    b = bytes_data.find(b'\xff\xd9') # JPEG end
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        try:
                            # Decodificar JPEG
                            img = Image.open(io.BytesIO(jpg))
                            img_np = np.array(img) # Convertir a NumPy array

                            # Procesar el frame
                            processed_original, processed_binary = process_frame(img_np)

                            # Redimensionar para la visualizacion
                            # Asumiendo que video_label y binary_label tienen un tamano
                            if video_label.winfo_width() > 0:
                                max_width = video_label.winfo_width()
                                max_height = video_label.winfo_height()
                                if max_width > 0 and max_height > 0:
                                    # Para mantener la proporcion de aspecto
                                    h, w, _ = processed_original.shape
                                    ratio = min(max_width / w, max_height / h)
                                    new_w = int(w * ratio)
                                    new_h = int(h * ratio)

                                    img_original_resized = Image.fromarray(processed_original).resize((new_w, new_h), Image.LANCZOS)
                                    img_binary_resized = Image.fromarray(processed_binary).resize((new_w, new_h), Image.LANCZOS)

                                    photo_original = ImageTk.PhotoImage(image=img_original_resized)
                                    photo_binary = ImageTk.PhotoImage(image=img_binary_resized)

                                    video_label.config(image=photo_original)
                                    video_label.image = photo_original
                                    binary_label.config(image=photo_binary)
                                    binary_label.image = photo_binary
                        except Exception as e:
                            print(f"Error procesando/mostrando frame: {e}")
        except requests.exceptions.ConnectionError:
            print(f"Conexion perdida con {ESP32_CAM_IP}. Reintentando en 3 segundos...")
            time.sleep(3)
        except requests.exceptions.RequestException as e:
            print(f"Error de stream: {e}. Reintentando en 3 segundos...")
            time.sleep(3)
        except Exception as e:
            print(f"Error inesperado en stream: {e}")
            time.sleep(1) # Pequeña pausa para evitar bucle apretado en caso de errores rapidos
    print("Stream de video detenido.")


def on_closing():
    """Funcion que se llama al cerrar la ventana de Tkinter."""
    print("Cerrando aplicacion...")
    stop_stream_flag.set() # Establece la bandera para detener el hilo del stream
    send_stop_command() # Asegurarse de que el robot se detenga al cerrar la app
    root.quit()
    root.destroy()

def update_status_labels(robot_status, mode_status):
    """Actualiza las etiquetas de estado del robot y del modo."""
    status_label.config(text=f"Estado del Robot: {robot_status}")
    mode_status_label.config(text=f"Modo: {mode_status}")


# --- Interfaz de Usuario (Tkinter) ---
def create_gui():
    """Crea y configura la interfaz grafica."""
    global root, video_label, binary_label, status_label, mode_status_label, auto_mode_button, stop_button

    root = tk.Tk()
    root.title("Control Robot ESP32-CAM")
    root.geometry("1000x800") # Aumentar un poco el tamano inicial

    # Estilos para los botones
    style = ttk.Style()
    style.configure("TButton", padding=10, font=("Arial", 12))
    style.configure("Green.TButton", background="#28a745", foreground="white")
    style.map("Green.TButton", background=[("active", "#218838")])
    style.configure("Red.TButton", background="#dc3545", foreground="white")
    style.map("Red.TButton", background=[("active", "#c82333")])
    style.configure("Blue.TButton", background="#007bff", foreground="white")
    style.map("Blue.TButton", background=[("active", "#0056b3")])

    # Frame principal
    main_frame = ttk.Frame(root, padding="10")
    main_frame.pack(fill="both", expand=True)

    # Frame para el estado
    status_container = ttk.LabelFrame(main_frame, text="Estado del Robot", padding="10")
    status_container.pack(pady=10, fill="x")

    status_label = ttk.Label(status_container, text="Estado del Robot: Desconocido", font=("Arial", 12, "bold"))
    status_label.pack(side="left", padx=10)

    mode_status_label = ttk.Label(status_container, text="Modo: Desconocido", font=("Arial", 12, "bold"))
    mode_status_label.pack(side="right", padx=10)

    # Contenedor para los frames de video
    video_frames_container = ttk.Frame(main_frame)
    video_frames_container.pack(pady=10, fill="both", expand=True)

    # Frame para el video original
    original_video_frame = ttk.LabelFrame(video_frames_container, text="Stream Original (Procesado)", padding="10")
    original_video_frame.pack(side="left", fill="both", expand=True, padx=5)
    video_label = tk.Label(original_video_frame)
    video_label.pack(fill="both", expand=True)

    # Frame para el video binarizado
    binary_video_frame = ttk.LabelFrame(video_frames_container, text="Imagen Binarizada + Deteccion", padding="10")
    binary_video_frame.pack(side="right", fill="both", expand=True, padx=5)
    binary_label = tk.Label(binary_video_frame)
    binary_label.pack(fill="both", expand=True)

    # Frame para los controles
    controls_frame = ttk.LabelFrame(main_frame, text="Control Manual del Robot", padding="10")
    controls_frame.pack(pady=10, fill="x")

    # Botones de control manual
    button_font = ("Arial", 14)
    button_width = 10

    # Grid para botones de movimiento
    grid_frame = ttk.Frame(controls_frame)
    grid_frame.pack(pady=10)

    # Fila superior (Rotar Izq, Adelante, Rotar Der)
    ttk.Button(grid_frame, text="Rotar Izq (Q)", command=lambda: send_manual_move_command("rotate_left"), style="Blue.TButton", width=button_width).grid(row=0, column=0, padx=5, pady=5)
    ttk.Button(grid_frame, text="Adelante (W)", command=lambda: send_manual_move_command("forward"), style="Blue.TButton", width=button_width).grid(row=0, column=1, padx=5, pady=5)
    ttk.Button(grid_frame, text="Rotar Der (E)", command=lambda: send_manual_move_command("rotate_right"), style="Blue.TButton", width=button_width).grid(row=0, column=2, padx=5, pady=5)

    # Fila central (Izquierda, STOP, Derecha)
    ttk.Button(grid_frame, text="Izquierda (A)", command=lambda: send_manual_move_command("left"), style="Blue.TButton", width=button_width).grid(row=1, column=0, padx=5, pady=5)
    stop_button = ttk.Button(grid_frame, text="STOP (S)", command=send_stop_command, style="Red.TButton", width=button_width)
    stop_button.grid(row=1, column=1, padx=5, pady=5)
    ttk.Button(grid_frame, text="Derecha (D)", command=lambda: send_manual_move_command("right"), style="Blue.TButton", width=button_width).grid(row=1, column=2, padx=5, pady=5)

    # Fila inferior (Atras)
    ttk.Button(grid_frame, text="Atras (S)", command=lambda: send_manual_move_command("backward"), style="Blue.TButton", width=button_width).grid(row=2, column=1, padx=5, pady=5)

    # Boton de modo automatico
    auto_mode_button = ttk.Button(controls_frame, text="Activar Modo Automatico", command=toggle_auto_mode, style="Green.TButton")
    auto_mode_button.pack(pady=10)

    # Manejo de teclas para control manual
    root.bind("<w>", lambda event: send_manual_move_command("forward"))
    root.bind("<a>", lambda event: send_manual_move_command("left"))
    root.bind("<s>", lambda event: send_manual_move_command("stop")) # 's' para detener
    root.bind("<d>", lambda event: send_manual_move_command("right"))
    root.bind("<q>", lambda event: send_manual_move_command("rotate_left"))
    root.bind("<e>", lambda event: send_manual_move_command("rotate_right"))

    # Configurar el cierre de la ventana
    root.protocol("WM_DELETE_WINDOW", on_closing)

    # Iniciar el hilo de video
    video_thread = threading.Thread(target=stream_video, daemon=True)
    video_thread.start()

    # Estado inicial
    update_status_labels("Detenido", "Manual")

    root.mainloop()

if __name__ == "__main__":
    create_gui()