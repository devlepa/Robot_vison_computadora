#!/usr/bin/env python3
"""
Robot ESP32-CAM Controller con Visión por Computadora
Controlador Python avanzado para robot ESP32-CAM con IA y visión
Autor: DevLepa
Versión: 2.0
"""

import tkinter as tk
from tkinter import ttk, messagebox, Frame, Label, Button, Scale, StringVar, BooleanVar
import cv2
import numpy as np
import requests
import threading
import time
from PIL import Image, ImageTk
import json
from datetime import datetime
import os
import sys

class RobotController:
    def __init__(self):
        # Configuración inicial
        self.robot_ip = "192.168.1.100"  # Cambiar por la IP de tu ESP32-CAM
        self.robot_url = f"http://{self.robot_ip}"
        self.stream_url = f"{self.robot_url}/stream"
        
        # Variables de control
        self.connected = False
        self.manual_mode = True
        self.ai_active = False
        self.recording = False
        self.frame_count = 0
        self.fps = 0
        self.last_fps_time = time.time()
        
        # Velocidades de motores
        self.speed_a = 200
        self.speed_b = 150
        self.speed_c = 200
        
        # Variables de visión por computadora
        self.detect_objects = False
        self.follow_object = False
        self.object_color_lower = np.array([0, 50, 50])
        self.object_color_upper = np.array([10, 255, 255])
        
        # Threading
        self.stream_thread = None
        self.ai_thread = None
        self.running = True
        
        # OpenCV
        self.current_frame = None
        self.processed_frame = None
        
        # Crear interfaz
        self.create_gui()
        self.start_systems()
        
    def create_gui(self):
        """Crear interfaz gráfica moderna"""
        self.root = tk.Tk()
        self.root.title("🤖 Robot ESP32-CAM Controller v2.0")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2c3e50')
        
        # Configurar estilo
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Modern.TFrame', background='#34495e')
        style.configure('Title.TLabel', background='#34495e', foreground='white', font=('Arial', 16, 'bold'))
        style.configure('Status.TLabel', background='#34495e', foreground='#2ecc71', font=('Arial', 10))
        
        # Frame principal
        main_frame = ttk.Frame(self.root, style='Modern.TFrame')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Crear paneles
        self.create_header(main_frame)
        self.create_video_panel(main_frame)
        self.create_control_panel(main_frame)
        self.create_ai_panel(main_frame)
        self.create_status_panel(main_frame)
        
        # Configurar eventos
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.setup_keyboard_controls()
        
    def create_header(self, parent):
        """Crear cabecera"""
        header_frame = ttk.Frame(parent, style='Modern.TFrame')
        header_frame.pack(fill=tk.X, pady=(0, 10))
        
        title_label = ttk.Label(header_frame, text="🤖 Robot ESP32-CAM Controller", style='Title.TLabel')
        title_label.pack(side=tk.LEFT)
        
        # Configuración de conexión
        connection_frame = ttk.Frame(header_frame, style='Modern.TFrame')
        connection_frame.pack(side=tk.RIGHT)
        
        tk.Label(connection_frame, text="IP Robot:", bg='#34495e', fg='white').pack(side=tk.LEFT, padx=5)
        self.ip_entry = tk.Entry(connection_frame, width=15)
        self.ip_entry.insert(0, self.robot_ip)
        self.ip_entry.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = tk.Button(connection_frame, text="🔗 Conectar", 
                                   command=self.connect_robot, bg='#3498db', fg='white')
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.connection_status = ttk.Label(connection_frame, text="❌ Desconectado", style='Status.TLabel')
        self.connection_status.pack(side=tk.LEFT, padx=10)
        
    def create_video_panel(self, parent):
        """Crear panel de video"""
        video_frame = ttk.LabelFrame(parent, text="📹 Video Stream & Visión por Computadora", style='Modern.TFrame')
        video_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Panel de video
        video_container = ttk.Frame(video_frame, style='Modern.TFrame')
        video_container.pack(fill=tk.BOTH, expand=True)
        
        # Video original
        left_panel = ttk.Frame(video_container, style='Modern.TFrame')
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        ttk.Label(left_panel, text="📸 Cámara Original", style='Title.TLabel').pack()
        self.video_label = tk.Label(left_panel, bg='black', width=40, height=20)
        self.video_label.pack(pady=5)
        
        # Video procesado
        right_panel = ttk.Frame(video_container, style='Modern.TFrame')
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        ttk.Label(right_panel, text="🧠 Procesamiento IA", style='Title.TLabel').pack()
        self.processed_label = tk.Label(right_panel, bg='black', width=40, height=20)
        self.processed_label.pack(pady=5)
        
        # Controles de video
        video_controls = ttk.Frame(video_frame, style='Modern.TFrame')
        video_controls.pack(fill=tk.X, pady=5)
        
        self.fps_label = ttk.Label(video_controls, text="FPS: 0", style='Status.TLabel')
        self.fps_label.pack(side=tk.LEFT, padx=10)
        
        self.record_btn = tk.Button(video_controls, text="🔴 Grabar", 
                                  command=self.toggle_recording, bg='#e74c3c', fg='white')
        self.record_btn.pack(side=tk.LEFT, padx=5)
        
        self.snapshot_btn = tk.Button(video_controls, text="📷 Captura", 
                                    command=self.take_snapshot, bg='#9b59b6', fg='white')
        self.snapshot_btn.pack(side=tk.LEFT, padx=5)
        
    def create_control_panel(self, parent):
        """Crear panel de controles"""
        control_frame = ttk.LabelFrame(parent, text="🎮 Control Manual", style='Modern.TFrame')
        control_frame.pack(fill=tk.X, pady=5)
        
        # Modo de control
        mode_frame = ttk.Frame(control_frame, style='Modern.TFrame')
        mode_frame.pack(fill=tk.X, pady=5)
        
        self.mode_var = tk.StringVar(value="MANUAL")
        self.mode_btn = tk.Button(mode_frame, text="Modo: MANUAL", font=('Arial', 12, 'bold'),
                                 command=self.toggle_mode, bg='#34495e', fg='white', width=15)
        self.mode_btn.pack(side=tk.LEFT, padx=10)
        
        # Controles direccionales
        direction_frame = ttk.Frame(control_frame, style='Modern.TFrame')
        direction_frame.pack(pady=10)
        
        # Fila 1: Adelante
        tk.Button(direction_frame, text="⬆️ Adelante", font=('Arial', 10, 'bold'),
                 bg='#2ecc71', fg='white', width=12, height=2,
                 command=lambda: self.send_command('forward')).grid(row=0, column=1, padx=5, pady=5)
        
        # Fila 2: Izquierda, Stop, Derecha
        tk.Button(direction_frame, text="⬅️ Izquierda", font=('Arial', 10, 'bold'),
                 bg='#3498db', fg='white', width=12, height=2,
                 command=lambda: self.send_command('left')).grid(row=1, column=0, padx=5, pady=5)
        
        tk.Button(direction_frame, text="⏹️ STOP", font=('Arial', 10, 'bold'),
                 bg='#e74c3c', fg='white', width=12, height=2,
                 command=lambda: self.send_command('stop')).grid(row=1, column=1, padx=5, pady=5)
        
        tk.Button(direction_frame, text="➡️ Derecha", font=('Arial', 10, 'bold'),
                 bg='#3498db', fg='white', width=12, height=2,
                 command=lambda: self.send_command('right')).grid(row=1, column=2, padx=5, pady=5)
        
        # Fila 3: Atrás
        tk.Button(direction_frame, text="⬇️ Atrás", font=('Arial', 10, 'bold'),
                 bg='#f39c12', fg='white', width=12, height=2,
                 command=lambda: self.send_command('backward')).grid(row=2, column=1, padx=5, pady=5)
        
        # Control individual de motores
        motors_frame = ttk.LabelFrame(control_frame, text="🔧 Control Individual Motores", style='Modern.TFrame')
        motors_frame.pack(fill=tk.X, pady=10)
        
        # Motor A
        motor_a_frame = ttk.Frame(motors_frame, style='Modern.TFrame')
        motor_a_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10)
        
        ttk.Label(motor_a_frame, text="🔴 Motor A", style='Title.TLabel').pack()
        
        motor_a_controls = ttk.Frame(motor_a_frame, style='Modern.TFrame')
        motor_a_controls.pack()
        
        tk.Button(motor_a_controls, text="⬆️", bg='#2ecc71', fg='white', width=5,
                 command=lambda: self.control_motor('A', 1)).pack(side=tk.LEFT, padx=2)
        tk.Button(motor_a_controls, text="⬇️", bg='#e74c3c', fg='white', width=5,
                 command=lambda: self.control_motor('A', -1)).pack(side=tk.LEFT, padx=2)
        tk.Button(motor_a_controls, text="⏹️", bg='#95a5a6', fg='white', width=5,
                 command=lambda: self.control_motor('A', 0)).pack(side=tk.LEFT, padx=2)
        
        self.speed_a_scale = tk.Scale(motor_a_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                                     label="Velocidad A", bg='#34495e', fg='white')
        self.speed_a_scale.set(self.speed_a)
        self.speed_a_scale.pack(fill=tk.X, pady=5)
        
        # Motor B
        motor_b_frame = ttk.Frame(motors_frame, style='Modern.TFrame')
        motor_b_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10)
        
        ttk.Label(motor_b_frame, text="🟡 Motor B", style='Title.TLabel').pack()
        
        motor_b_controls = ttk.Frame(motor_b_frame, style='Modern.TFrame')
        motor_b_controls.pack()
        
        tk.Button(motor_b_controls, text="⬆️", bg='#2ecc71', fg='white', width=5,
                 command=lambda: self.control_motor('B', 1)).pack(side=tk.LEFT, padx=2)
        tk.Button(motor_b_controls, text="⬇️", bg='#e74c3c', fg='white', width=5,
                 command=lambda: self.control_motor('B', -1)).pack(side=tk.LEFT, padx=2)
        tk.Button(motor_b_controls, text="⏹️", bg='#95a5a6', fg='white', width=5,
                 command=lambda: self.control_motor('B', 0)).pack(side=tk.LEFT, padx=2)
        
        self.speed_b_scale = tk.Scale(motor_b_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                                     label="Velocidad B", bg='#34495e', fg='white')
        self.speed_b_scale.set(self.speed_b)
        self.speed_b_scale.pack(fill=tk.X, pady=5)
        
        # Motor C
        motor_c_frame = ttk.Frame(motors_frame, style='Modern.TFrame')
        motor_c_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10)
        
        ttk.Label(motor_c_frame, text="🔵 Motor C", style='Title.TLabel').pack()
        
        motor_c_controls = ttk.Frame(motor_c_frame, style='Modern.TFrame')
        motor_c_controls.pack()
        
        tk.Button(motor_c_controls, text="⬆️", bg='#2ecc71', fg='white', width=5,
                 command=lambda: self.control_motor('C', 1)).pack(side=tk.LEFT, padx=2)
        tk.Button(motor_c_controls, text="⬇️", bg='#e74c3c', fg='white', width=5,
                 command=lambda: self.control_motor('C', -1)).pack(side=tk.LEFT, padx=2)
        tk.Button(motor_c_controls, text="⏹️", bg='#95a5a6', fg='white', width=5,
                 command=lambda: self.control_motor('C', 0)).pack(side=tk.LEFT, padx=2)
        
        self.speed_c_scale = tk.Scale(motor_c_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                                     label="Velocidad C", bg='#34495e', fg='white')
        self.speed_c_scale.set(self.speed_c)
        self.speed_c_scale.pack(fill=tk.X, pady=5)
        
    def create_ai_panel(self, parent):
        """Crear panel de IA"""
        ai_frame = ttk.LabelFrame(parent, text="🧠 Inteligencia Artificial & Visión", style='Modern.TFrame')
        ai_frame.pack(fill=tk.X, pady=5)
        
        # Controles de IA
        ai_controls = ttk.Frame(ai_frame, style='Modern.TFrame')
        ai_controls.pack(fill=tk.X, pady=5)
        
        self.ai_btn = tk.Button(ai_controls, text="▶️ Activar IA", font=('Arial', 12, 'bold'),
                               command=self.toggle_ai, bg='#9b59b6', fg='white', width=15)
        self.ai_btn.pack(side=tk.LEFT, padx=10)
        
        # Detección de objetos
        vision_frame = ttk.Frame(ai_frame, style='Modern.TFrame')
        vision_frame.pack(fill=tk.X, pady=5)
        
        self.detect_var = tk.BooleanVar()
        detect_check = tk.Checkbutton(vision_frame, text="🎯 Detección de Objetos", 
                                     variable=self.detect_var, command=self.toggle_detection,
                                     bg='#34495e', fg='white', selectcolor='#2ecc71')
        detect_check.pack(side=tk.LEFT, padx=10)
        
        self.follow_var = tk.BooleanVar()
        follow_check = tk.Checkbutton(vision_frame, text="🎯 Seguir Objeto", 
                                     variable=self.follow_var, command=self.toggle_following,
                                     bg='#34495e', fg='white', selectcolor='#e74c3c')
        follow_check.pack(side=tk.LEFT, padx=10)
        
        # Configuración de color para detección
        color_frame = ttk.Frame(ai_frame, style='Modern.TFrame')
        color_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(color_frame, text="🎨 Color a detectar:", bg='#34495e', fg='white').pack(side=tk.LEFT, padx=5)
        
        colors = [("🔴 Rojo", [0, 10]), ("🟢 Verde", [40, 80]), ("🔵 Azul", [100, 130]), ("🟡 Amarillo", [20, 40])]
        self.color_var = tk.StringVar(value="🔴 Rojo")
        
        for color_name, hue_range in colors:
            tk.Radiobutton(color_frame, text=color_name, variable=self.color_var, value=color_name,
                          command=lambda hr=hue_range: self.set_color_range(hr),
                          bg='#34495e', fg='white', selectcolor='#2ecc71').pack(side=tk.LEFT, padx=5)
        
    def create_status_panel(self, parent):
        """Crear panel de estado"""
        status_frame = ttk.LabelFrame(parent, text="📊 Estado del Sistema", style='Modern.TFrame')
        status_frame.pack(fill=tk.X, pady=5)
        
        status_info = ttk.Frame(status_frame, style='Modern.TFrame')
        status_info.pack(fill=tk.X, pady=5)
        
        self.status_text = tk.Text(status_info, height=4, bg='#2c3e50', fg='#2ecc71', 
                                  font=('Consolas', 9), wrap=tk.WORD)
        self.status_text.pack(fill=tk.X, padx=10)
        
        # Agregar scrollbar
        scrollbar = tk.Scrollbar(status_info, command=self.status_text.yview)
        self.status_text.config(yscrollcommand=scrollbar.set)
        
        self.log_message("🤖 Sistema inicializado - Listo para conectar")
        
    def setup_keyboard_controls(self):
        """Configurar controles de teclado"""
        self.root.bind('<KeyPress-w>', lambda e: self.send_command('forward'))
        self.root.bind('<KeyPress-s>', lambda e: self.send_command('backward'))
        self.root.bind('<KeyPress-a>', lambda e: self.send_command('left'))
        self.root.bind('<KeyPress-d>', lambda e: self.send_command('right'))
        self.root.bind('<KeyPress-space>', lambda e: self.send_command('stop'))
        self.root.bind('<KeyPress-q>', lambda e: self.toggle_mode())
        self.root.bind('<KeyPress-e>', lambda e: self.toggle_ai())
        
        self.root.focus_set()  # Para que funcionen las teclas
        
    def connect_robot(self):
        """Conectar al robot ESP32-CAM"""
        self.robot_ip = self.ip_entry.get()
        self.robot_url = f"http://{self.robot_ip}"
        self.stream_url = f"{self.robot_url}/stream"
        
        try:
            response = requests.get(f"{self.robot_url}/status", timeout=3)
            if response.status_code == 200:
                self.connected = True
                self.connection_status.config(text="✅ Conectado")
                self.connect_btn.config(text="🔓 Desconectar", bg='#e74c3c')
                self.log_message(f"✅ Conectado al robot en {self.robot_ip}")
                self.start_video_stream()
            else:
                raise Exception("Robot no responde")
        except Exception as e:
            self.connected = False
            self.connection_status.config(text="❌ Error conexión")
            self.log_message(f"❌ Error conectando: {str(e)}")
            messagebox.showerror("Error", f"No se pudo conectar al robot en {self.robot_ip}")
    
    def start_video_stream(self):
        """Iniciar stream de video"""
        if self.stream_thread is None or not self.stream_thread.is_alive():
            self.stream_thread = threading.Thread(target=self.stream_worker, daemon=True)
            self.stream_thread.start()
            self.log_message("📹 Stream de video iniciado")
    
    def stream_worker(self):
        """Worker para el stream de video"""
        while self.running and self.connected:
            try:
                response = requests.get(self.stream_url, stream=True, timeout=1)
                if response.status_code == 200:
                    # Convertir imagen
                    img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        self.current_frame = frame.copy()
                        self.update_video_display(frame)
                        
                        # Procesar con IA si está activada
                        if self.detect_objects or self.follow_object:
                            self.process_computer_vision(frame)
                        
                        # Actualizar FPS
                        self.update_fps()
                        
            except Exception as e:
                self.log_message(f"⚠️ Error en stream: {str(e)}")
                time.sleep(1)
    
    def update_video_display(self, frame):
        """Actualizar display de video"""
        try:
            # Redimensionar frame
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_resized = cv2.resize(frame_rgb, (320, 240))
            
            # Convertir a PhotoImage
            img_pil = Image.fromarray(frame_resized)
            img_tk = ImageTk.PhotoImage(img_pil)
            
            # Actualizar label en thread principal
            self.root.after(0, lambda: self.video_label.configure(image=img_tk))
            self.root.after(0, lambda: setattr(self.video_label, 'image', img_tk))
            
        except Exception as e:
            self.log_message(f"⚠️ Error actualizando video: {str(e)}")
    
    def process_computer_vision(self, frame):
        """Procesar visión por computadora"""
        try:
            # Convertir a HSV para mejor detección de color
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Crear máscara para el color seleccionado
            mask = cv2.inRange(hsv, self.object_color_lower, self.object_color_upper)
            
            # Filtrar ruido
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Encontrar contornos
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            processed_frame = frame.copy()
            
            if contours:
                # Encontrar el contorno más grande
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 500:  # Filtrar objetos muy pequeños
                    # Dibujar contorno
                    cv2.drawContours(processed_frame, [largest_contour], -1, (0, 255, 0), 2)
                    
                    # Calcular centro
                    moments = cv2.moments(largest_contour)
                    if moments["m00"] != 0:
                        cx = int(moments["m10"] / moments["m00"])
                        cy = int(moments["m01"] / moments["m00"])
                        
                        # Dibujar centro
                        cv2.circle(processed_frame, (cx, cy), 10, (255, 0, 0), -1)
                        cv2.putText(processed_frame, f"Objeto: ({cx},{cy})", (cx-50, cy-20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        
                        # Seguimiento automático si está activado
                        if self.follow_object and not self.manual_mode:
                            self.follow_object_logic(cx, cy, frame.shape[1], frame.shape[0])
            
            # Agregar información de estado
            cv2.putText(processed_frame, f"IA: {'ON' if not self.manual_mode else 'OFF'}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if not self.manual_mode else (0, 0, 255), 2)
            cv2.putText(processed_frame, f"Deteccion: {'ON' if self.detect_objects else 'OFF'}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if self.detect_objects else (0, 0, 255), 2)
            
            self.processed_frame = processed_frame
            self.update_processed_display(processed_frame)
            
        except Exception as e:
            self.log_message(f"⚠️ Error en visión por computadora: {str(e)}")
    
    def follow_object_logic(self, cx, cy, frame_width, frame_height):
        """Lógica para seguir objeto"""
        center_x = frame_width // 2
        center_y = frame_height // 2
        
        # Tolerancia para el centro
        tolerance_x = 50
        tolerance_y = 30
        
        # Determinar acción basada en posición del objeto
        if cx < center_x - tolerance_x:
            # Objeto a la izquierda - girar izquierda
            self.send_command('left')
            self.log_message("🎯 Siguiendo objeto: Girando izquierda")
        elif cx > center_x + tolerance_x:
            # Objeto a la derecha - girar derecha
            self.send_command('right')
            self.log_message("🎯 Siguiendo objeto: Girando derecha")
        elif cy > center_y + tolerance_y:
            # Objeto muy cerca - parar o retroceder
            self.send_command('stop')
            self.log_message("🎯 Objeto muy cerca: Deteniendo")
        else:
            # Objeto centrado - avanzar
            self.send_command('forward')
            self.log_message("🎯 Objeto centrado: Avanzando")
    
    def update_processed_display(self, frame):
        """Actualizar display de video procesado"""
        try:
            # Redimensionar frame
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_resized = cv2.resize(frame_rgb, (320, 240))
            
            # Convertir a PhotoImage
            img_pil = Image.fromarray(frame_resized)
            img_tk = ImageTk.PhotoImage(img_pil)
            
            # Actualizar label en thread principal
            self.root.after(0, lambda: self.processed_label.configure(image=img_tk))
            self.root.after(0, lambda: setattr(self.processed_label, 'image', img_tk))
            
        except Exception as e:
            self.log_message(f"⚠️ Error actualizando video procesado: {str(e)}")
    
    def update_fps(self):
        """Actualizar contador FPS"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
            
            # Actualizar label FPS
            self.root.after(0, lambda: self.fps_label.config(text=f"FPS: {self.fps:.1f}"))
    
    def send_command(self, command):
        """Enviar comando al robot"""
        if not self.connected:
            self.log_message("⚠️ Robot no conectado")
            return
        
        try:
            url = f"{self.robot_url}/control?cmd={command}"
            response = requests.get(url, timeout=1)
            if response.status_code == 200:
                self.log_message(f"✅ Comando enviado: {command}")
            else:
                self.log_message(f"⚠️ Error enviando comando: {command}")
        except Exception as e:
            self.log_message(f"❌ Error de comunicación: {str(e)}")
    
    def control_motor(self, motor, direction):
        """Controlar motor individual"""
        if not self.connected:
            self.log_message("⚠️ Robot no conectado")
            return
        
        # Obtener velocidad actual
        if motor == 'A':
            speed = self.speed_a_scale.get()
        elif motor == 'B':
            speed = self.speed_b_scale.get()
        else:  # motor == 'C'
            speed = self.speed_c_scale.get()
        
        try:
            url = f"{self.robot_url}/motor?motor={motor}&dir={direction}&speed={speed}"
            response = requests.get(url, timeout=1)
            if response.status_code == 200:
                action = "adelante" if direction == 1 else "atrás" if direction == -1 else "parar"
                self.log_message(f"✅ Motor {motor}: {action} (velocidad: {speed})")
            else:
                self.log_message(f"⚠️ Error controlando motor {motor}")
        except Exception as e:
            self.log_message(f"❌ Error de comunicación: {str(e)}")
    
    def toggle_mode(self):
        """Cambiar entre modo manual e IA"""
        self.manual_mode = not self.manual_mode
        
        if self.connected:
            try:
                url = f"{self.robot_url}/mode?manual={'1' if self.manual_mode else '0'}"
                requests.get(url, timeout=1)
            except:
                pass
        
        mode_text = "MANUAL" if self.manual_mode else "IA"
        self.mode_btn.config(text=f"Modo: {mode_text}")
        self.mode_btn.config(bg='#34495e' if self.manual_mode else '#2ecc71')
        
        self.log_message(f"🔄 Modo cambiado a: {mode_text}")
    
    def toggle_ai(self):
        """Activar/desactivar IA"""
        if not self.connected:
            self.log_message("⚠️ Robot no conectado")
            return
        
        try:
            response = requests.get(f"{self.robot_url}/ai_toggle", timeout=1)
            if response.status_code == 200:
                status = response.text
                self.ai_active = (status == "started")
                
                ai_text = "⏸️ Pausar IA" if self.ai_active else "▶️ Activar IA"
                self.ai_btn.config(text=ai_text)
                self.ai_btn.config(bg='#e74c3c' if self.ai_active else '#9b59b6')
                
                self.log_message(f"🧠 IA {'activada' if self.ai_active else 'desactivada'}")
        except Exception as e:
            self.log_message(f"❌ Error controlando IA: {str(e)}")
    
    def toggle_detection(self):
        """Activar/desactivar detección de objetos"""
        self.detect_objects = self.detect_var.get()
        status = "activada" if self.detect_objects else "desactivada"
        self.log_message(f"🎯 Detección de objetos {status}")
    
    def toggle_following(self):
        """Activar/desactivar seguimiento de objetos"""
        self.follow_object = self.follow_var.get()
        status = "activado" if self.follow_object else "desactivado"
        self.log_message(f"🎯 Seguimiento de objetos {status}")
    
    def set_color_range(self, hue_range):
        """Configurar rango de color para detección"""
        self.object_color_lower = np.array([hue_range[0], 50, 50])
        self.object_color_upper = np.array([hue_range[1], 255, 255])
        self.log_message(f"🎨 Color configurado: HSV {hue_range}")
    
    def toggle_recording(self):
        """Iniciar/detener grabación"""
        self.recording = not self.recording
        
        if self.recording:
            self.record_btn.config(text="⏹️ Parar", bg='#2ecc71')
            self.log_message("🔴 Grabación iniciada")
            # Aquí puedes agregar lógica para grabar video
        else:
            self.record_btn.config(text="🔴 Grabar", bg='#e74c3c')
            self.log_message("⏹️ Grabación detenida")
    
    def take_snapshot(self):
        """Tomar captura de pantalla"""
        if self.current_frame is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"snapshot_{timestamp}.jpg"
            cv2.imwrite(filename, self.current_frame)
            self.log_message(f"📷 Captura guardada: {filename}")
        else:
            self.log_message("⚠️ No hay imagen para capturar")
    
    def log_message(self, message):
        """Agregar mensaje al log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        full_message = f"[{timestamp}] {message}\n"
        
        # Ejecutar en thread principal
        self.root.after(0, lambda: self.status_text.insert(tk.END, full_message))
        self.root.after(0, lambda: self.status_text.see(tk.END))
        
        # Limitar líneas del log
        self.root.after(0, self.limit_log_lines)
    
    def limit_log_lines(self):
        """Limitar líneas del log para evitar memory leak"""
        lines = self.status_text.get("1.0", tk.END).split('\n')
        if len(lines) > 50:  # Mantener solo las últimas 50 líneas
            self.status_text.delete("1.0", f"{len(lines)-50}.0")
    
    def start_systems(self):
        """Iniciar sistemas del robot"""
        self.log_message("🚀 Sistemas iniciados")
        self.log_message("⌨️ Controles de teclado: W/A/S/D para movimiento, SPACE para parar")
        self.log_message("⌨️ Q para cambiar modo, E para activar/desactivar IA")
        
    def on_closing(self):
        """Manejar cierre de aplicación"""
        self.running = False
        
        if self.connected:
            try:
                requests.get(f"{self.robot_url}/control?cmd=stop", timeout=1)
            except:
                pass
        
        self.log_message("👋 Cerrando aplicación...")
        self.root.quit()
        self.root.destroy()
    
    def run(self):
        """Ejecutar aplicación"""
        self.root.mainloop()

def main():
    """Función principal"""
    print("🤖 Robot ESP32-CAM Controller v2.0")
    print("Iniciando aplicación...")
    
    try:
        app = RobotController()
        app.run()
    except KeyboardInterrupt:
        print("\n👋 Aplicación cerrada por el usuario")
    except Exception as e:
        print(f"❌ Error fatal: {str(e)}")

if __name__ == "__main__":
    main()