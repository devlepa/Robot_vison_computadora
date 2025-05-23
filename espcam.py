import cv2
import numpy as np
import requests
import json
import time
import threading
from collections import deque
import math
import sys

class RobotOmnidireccionalIA:
    def __init__(self, esp32_ip="192.168.10.43"):
        self.esp32_ip = esp32_ip
        self.capture_url = f"http://{esp32_ip}/capture"
        self.control_url = f"http://{esp32_ip}/control"
        self.stop_url = f"http://{esp32_ip}/stop"
        self.status_url = f"http://{esp32_ip}/status"
        
        # Configuración de imagen
        self.frame_width = 320
        self.frame_height = 240
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
        
        # IA: Sistema avanzado de detección
        self.line_history = deque(maxlen=12)  # Historial de líneas detectadas
        self.movement_buffer = deque(maxlen=6)  # Suavizado de movimientos
        self.direction_history = deque(maxlen=8)  # Historial de direcciones
        
        # Parámetros IA adaptivos
        self.confidence_threshold = 0.4
        self.lost_line_count = 0
        self.max_lost_frames = 20
        self.tracking_confidence = 0.0
        
        # Control omnidireccional optimizado
        self.base_speed = 0.35           # Velocidad base
        self.lateral_gain = 0.6          # Ganancia lateral
        self.angular_gain = 0.4          # Ganancia angular
        self.max_speed = 0.8             # Velocidad máxima
        self.speed_boost = 1.0           # Multiplicador de velocidad
        
        # Estados del sistema
        self.robot_active = False
        self.ia_enabled = True
        self.debug_enabled = True
        self.adaptive_mode = True
        
        # Estadísticas en tiempo real
        self.frame_count = 0
        self.successful_detections = 0
        self.commands_sent = 0
        self.start_time = time.time()
        
        # Variables de rendimiento
        self.last_fps_time = time.time()
        self.fps_counter = 0
        self.current_fps = 0
        
        print("🤖 ROBOT IA OMNIDIRECCIONAL v2.0")
        print("="*50)
        print(f"🌐 ESP32-CAM: {esp32_ip}")
        print(f"📸 Captura: {self.capture_url}")
        print(f"🎮 Control: {self.control_url}")
        print(f"🤖 Tasa éxito esperada: 80%+")
        
        # Verificar sistema
        self.system_check()

    def system_check(self):
        """Verificación completa del sistema"""
        print("\n🔍 VERIFICACIÓN DEL SISTEMA")
        print("-" * 30)
        
        try:
            # Test de status
            response = requests.get(self.status_url, timeout=5)
            if response.status_code == 200:
                status = response.json()
                print(f"✅ Conexión: OK")
                print(f"📊 Status ESP32: {status}")
                
                # Test rápido de captura
                print("📸 Probando captura...")
                cap_response = requests.get(self.capture_url, timeout=8)
                if cap_response.status_code == 200:
                    print(f"✅ Captura: OK ({len(cap_response.content)} bytes)")
                    
                    # Verificar imagen
                    img_array = np.asarray(bytearray(cap_response.content), dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    if frame is not None:
                        print(f"✅ Imagen válida: {frame.shape}")
                        print("🚀 Sistema listo para IA!")
                        return True
                    else:
                        print("❌ Imagen corrupta")
                else:
                    print(f"❌ Error captura: {cap_response.status_code}")
            else:
                print(f"❌ Error status: {response.status_code}")
                
        except Exception as e:
            print(f"❌ Error sistema: {e}")
            print("🔧 Verifica que el ESP32 esté encendido y funcionando")
            
        return False

    def capture_frame_optimized(self):
        """Captura optimizada con retry automático"""
        max_retries = 2
        for attempt in range(max_retries):
            try:
                response = requests.get(self.capture_url, timeout=3)
                if response.status_code == 200:
                    img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if frame is not None and frame.shape[0] > 0 and frame.shape[1] > 0:
                        return frame
                        
            except Exception as e:
                if attempt == max_retries - 1 and self.debug_enabled:
                    print(f"⚠️ Error captura intento {attempt + 1}: {str(e)[:50]}...")
                    
        return None

    def preprocess_advanced(self, frame):
        """Preprocesamiento avanzado con IA"""
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # IA: Ecualización adaptativa del histograma
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)
        
        # Filtro bilateral para suavizar preservando bordes
        bilateral = cv2.bilateralFilter(enhanced, 9, 75, 75)
        
        # IA: Binarización adaptativa optimizada
        binary = cv2.adaptiveThreshold(bilateral, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                     cv2.THRESH_BINARY_INV, 13, 4)
        
        # IA: Operaciones morfológicas inteligentes
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        
        # Cerrar gaps en líneas
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)
        # Eliminar ruido
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)
        
        return binary

    def detect_line_ai_advanced(self, binary):
        """IA avanzada: Detección multi-método de líneas"""
        height, width = binary.shape
        
        # IA: Zona de interés adaptativa
        roi_top = int(height * 0.4)
        roi_bottom = int(height * 0.95)
        roi = binary[roi_top:roi_bottom, :]
        
        line_points = []
        confidence_values = []
        
        # Método 1: Análisis por franjas con ponderación
        num_strips = 6
        strip_height = roi.shape[0] // num_strips
        
        for i in range(num_strips):
            y_start = i * strip_height
            y_end = (i + 1) * strip_height
            strip = roi[y_start:y_end, :]
            
            # Peso según distancia (más cerca = más importante)
            strip_weight = 1.0 - (i * 0.15)  # Peso decreciente
            
            # Encontrar contornos
            contours, _ = cv2.findContours(strip, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Filtrar y evaluar contornos
                valid_contours = []
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 40:  # Filtro por área mínima
                        # Evaluar forma del contorno
                        perimeter = cv2.arcLength(contour, True)
                        if perimeter > 0:
                            circularity = 4 * np.pi * area / (perimeter * perimeter)
                            if 0.1 < circularity < 0.9:  # Filtro de forma (no muy circular)
                                valid_contours.append((contour, area))
                
                if valid_contours:
                    # Tomar el contorno más grande
                    largest_contour, largest_area = max(valid_contours, key=lambda x: x[1])
                    
                    # Calcular centroide
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = y_start + int(M["m01"] / M["m00"]) + roi_top
                        
                        # Calcular confianza basada en área, peso y posición
                        area_confidence = min(largest_area / 500, 1.0)
                        position_confidence = 1.0 - abs(cx - width/2) / (width/2)
                        total_confidence = (area_confidence * 0.6 + position_confidence * 0.4) * strip_weight
                        
                        line_points.append((cx, cy))
                        confidence_values.append(total_confidence)
        
        # Método 2: Detección por proyección horizontal (refuerzo)
        if len(line_points) < 3:
            # Proyección horizontal en zona central
            central_strip = roi[roi.shape[0]//3:2*roi.shape[0]//3, :]
            h_projection = np.sum(central_strip, axis=0)
            
            if np.max(h_projection) > 0:
                # Suavizar proyección
                h_smooth = cv2.GaussianBlur(h_projection.astype(np.float32), (15, 1), 0)
                
                # Encontrar picos
                threshold = np.mean(h_smooth) + np.std(h_smooth)
                peaks = []
                for i in range(1, len(h_smooth)-1):
                    if h_smooth[i] > threshold and h_smooth[i] > h_smooth[i-1] and h_smooth[i] > h_smooth[i+1]:
                        peaks.append((i, h_smooth[i]))
                
                if peaks:
                    # Tomar el pico más fuerte
                    best_peak = max(peaks, key=lambda x: x[1])
                    peak_x = best_peak[0]
                    peak_y = roi_top + roi.shape[0] // 2
                    
                    line_points.append((peak_x, peak_y))
                    confidence_values.append(0.5)  # Confianza media para método backup
        
        return line_points, confidence_values

    def calculate_trajectory_ai(self, line_points, confidence_values):
        """IA: Cálculo avanzado de trayectoria con predicción"""
        if len(line_points) < 2:
            return None, None, 0, 0
        
        # IA: Filtrar puntos por confianza
        if confidence_values:
            filtered_data = [(p, c) for p, c in zip(line_points, confidence_values) 
                           if c > self.confidence_threshold]
            if len(filtered_data) >= 2:
                line_points = [p for p, c in filtered_data]
                confidence_values = [c for p, c in filtered_data]
        
        # IA: Regresión lineal ponderada
        x_coords = np.array([p[0] for p in line_points])
        y_coords = np.array([p[1] for p in line_points])
        weights = np.array(confidence_values) if confidence_values else np.ones(len(line_points))
        
        try:
            # Regresión ponderada
            sum_w = np.sum(weights)
            sum_wx = np.sum(weights * x_coords)
            sum_wy = np.sum(weights * y_coords)
            sum_wxy = np.sum(weights * x_coords * y_coords)
            sum_wx2 = np.sum(weights * x_coords * x_coords)
            
            # Calcular pendiente e intercepto
            denominator = sum_w * sum_wx2 - sum_wx * sum_wx
            if abs(denominator) > 1e-6:
                slope = (sum_w * sum_wxy - sum_wx * sum_wy) / denominator
                intercept = (sum_wy - slope * sum_wx) / sum_w
            else:
                # Fallback a promedio
                slope = 0
                intercept = np.mean(x_coords)
        except:
            slope = 0
            intercept = np.mean(x_coords)
        
        # Punto de referencia para control
        ref_y = self.frame_height * 0.85
        ref_x = slope * ref_y + intercept
        
        # Calcular confianza global
        global_confidence = np.mean(confidence_values) if confidence_values else 0
        
        # Ángulo de la línea
        line_angle = np.arctan(slope) if slope != 0 else 0
        
        # Agregar al historial para IA predictiva
        self.line_history.append({
            'ref_x': ref_x,
            'slope': slope,
            'angle': line_angle,
            'confidence': global_confidence,
            'timestamp': time.time()
        })
        
        return ref_x, slope, global_confidence, line_angle

    def ai_movement_controller(self, ref_x, slope, confidence, line_angle):
        """IA: Controlador avanzado de movimiento omnidireccional"""
        if ref_x is None:
            return self.handle_lost_line_ai()
        
        # IA: Análisis de tendencia histórica
        if len(self.line_history) > 3:
            recent_positions = [entry['ref_x'] for entry in list(self.line_history)[-4:]]
            position_trend = np.polyfit(range(len(recent_positions)), recent_positions, 1)[0]
            
            recent_angles = [entry['angle'] for entry in list(self.line_history)[-3:]]
            angle_trend = np.mean(recent_angles)
        else:
            position_trend = 0
            angle_trend = 0
        
        # Error de posición lateral (normalizado)
        lateral_error = (ref_x - self.center_x) / self.center_x
        
        # IA: Error angular (combinando pendiente y tendencia)
        angular_error = line_angle * 0.7 + angle_trend * 0.3
        angular_error = np.clip(angular_error / (np.pi / 4), -1, 1)
        
        # IA: Velocidad adaptativa basada en confianza y curvatura
        curvature_factor = abs(angular_error)
        confidence_factor = confidence
        speed_factor = (0.6 + 0.4 * confidence_factor) * (1.0 - curvature_factor * 0.4)
        
        adaptive_speed = self.base_speed * speed_factor * self.speed_boost
        
        # Cálculo de velocidades omnidireccionales
        # X: Movimiento lateral con compensación predictiva
        x_velocity = (-lateral_error * self.lateral_gain) + (position_trend * 0.15)
        
        # Y: Velocidad hacia adelante adaptativa
        y_velocity = adaptive_speed
        
        # Rotación: Control angular suavizado
        rotation_velocity = -angular_error * self.angular_gain
        
        # IA: Suavizado temporal con buffer
        self.movement_buffer.append((x_velocity, y_velocity, rotation_velocity))
        if len(self.movement_buffer) > 1:
            # Promedio ponderado con más peso a comandos recientes
            weights = np.linspace(0.5, 1.0, len(self.movement_buffer))
            weights /= np.sum(weights)
            
            recent_commands = np.array(list(self.movement_buffer))
            x_velocity = np.average(recent_commands[:, 0], weights=weights)
            y_velocity = np.average(recent_commands[:, 1], weights=weights)
            rotation_velocity = np.average(recent_commands[:, 2], weights=weights)
        
        # Limitar velocidades
        x_velocity = np.clip(x_velocity, -self.max_speed, self.max_speed)
        y_velocity = np.clip(y_velocity, 0, self.max_speed)
        rotation_velocity = np.clip(rotation_velocity, -self.max_speed, self.max_speed)
        
        # Actualizar estados
        self.lost_line_count = 0
        self.tracking_confidence = confidence
        
        return x_velocity, y_velocity, rotation_velocity

    def handle_lost_line_ai(self):
        """IA: Manejo inteligente de línea perdida con predicción"""
        self.lost_line_count += 1
        
        if self.lost_line_count < self.max_lost_frames:
            # IA: Predicción basada en historial
            if len(self.line_history) > 2:
                recent_entries = list(self.line_history)[-3:]
                
                # Extrapolar posición
                positions = [entry['ref_x'] for entry in recent_entries]
                angles = [entry['angle'] for entry in recent_entries]
                
                if len(positions) > 1:
                    # Predicción lineal
                    trend = positions[-1] - positions[0]
                    predicted_x = positions[-1] + trend * 0.5
                    predicted_angle = np.mean(angles)
                    
                    # Simular detección con predicción
                    lateral_error = (predicted_x - self.center_x) / self.center_x
                    angular_error = predicted_angle / (np.pi / 4)
                    
                    # Movimiento conservador basado en predicción
                    x_velocity = -lateral_error * self.lateral_gain * 0.6
                    y_velocity = self.base_speed * 0.4  # Velocidad muy reducida
                    rotation_velocity = -angular_error * self.angular_gain * 0.5
                    
                    if self.debug_enabled:
                        print(f"🔮 IA Predicción: frame {self.lost_line_count}, X={x_velocity:.2f}")
                    
                    return x_velocity, y_velocity, rotation_velocity
            
            # Movimiento de búsqueda conservador
            if self.movement_buffer:
                last_movement = self.movement_buffer[-1]
                search_factor = 0.3 * (1.0 - self.lost_line_count / self.max_lost_frames)
                
                x_velocity = last_movement[0] * search_factor
                y_velocity = self.base_speed * 0.2
                rotation_velocity = last_movement[2] * 0.3
                
                return x_velocity, y_velocity, rotation_velocity
        
        # Parada completa después de muchos frames perdidos
        print(f"❌ Línea perdida por {self.lost_line_count} frames - parada")
        return 0, 0, 0

    def send_omnidirectional_command(self, x, y, rotation):
        """Envío optimizado de comandos al robot"""
        try:
            command = {
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "rotation": round(float(rotation), 3)
            }
            
            response = requests.post(
                self.control_url,
                json=command,
                headers={'Content-Type': 'application/json'},
                timeout=1.5
            )
            
            if response.status_code == 200:
                self.commands_sent += 1
                if self.debug_enabled and self.commands_sent % 10 == 0:
                    print(f"📤 Comando #{self.commands_sent}: X={x:.2f} Y={y:.2f} R={rotation:.2f}")
                return True
            else:
                if self.debug_enabled:
                    print(f"❌ Error HTTP: {response.status_code}")
                return False
                
        except Exception as e:
            if self.debug_enabled:
                print(f"❌ Error envío: {str(e)[:40]}...")
            return False

    def emergency_stop(self):
        """Parada de emergencia"""
        try:
            response = requests.get(self.stop_url, timeout=2)
            if response.status_code == 200:
                print("🛑 PARADA DE EMERGENCIA EXITOSA")
            else:
                print(f"⚠️ Error en parada: {response.status_code}")
            self.robot_active = False
        except Exception as e:
            print(f"❌ Error parada emergencia: {e}")

    def draw_advanced_debug(self, frame, binary, line_points, confidence_values, 
                           ref_x, slope, global_confidence, x_vel, y_vel, rot_vel):
        """Debug avanzado con información de IA"""
        debug_frame = frame.copy()
        
        # Zona de interés
        roi_top = int(self.frame_height * 0.4)
        roi_bottom = int(self.frame_height * 0.95)
        cv2.rectangle(debug_frame, (0, roi_top), (self.frame_width, roi_bottom), (0, 255, 255), 1)
        
        # Centro y líneas de referencia
        cv2.circle(debug_frame, (self.center_x, self.center_y), 3, (0, 255, 0), -1)
        cv2.line(debug_frame, (self.center_x, 0), (self.center_x, self.frame_height), (0, 255, 0), 1)
        cv2.line(debug_frame, (0, self.center_y), (self.frame_width, self.center_y), (0, 255, 0), 1)
        
        # Puntos de línea detectados con confianza
        for i, (point, conf) in enumerate(zip(line_points, confidence_values or [0.5]*len(line_points))):
            color_intensity = int(255 * conf)
            color = (0, color_intensity, 255 - color_intensity)
            cv2.circle(debug_frame, point, 4, color, -1)
            cv2.putText(debug_frame, f"{conf:.2f}", (point[0]+5, point[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
        
        # Línea de trayectoria
        if ref_x is not None and slope is not None:
            y1, y2 = roi_top, self.frame_height
            x1 = int(slope * y1 + (ref_x - slope * self.frame_height * 0.85))
            x2 = int(slope * y2 + (ref_x - slope * self.frame_height * 0.85))
            cv2.line(debug_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            
            # Punto de control
            cv2.circle(debug_frame, (int(ref_x), int(self.frame_height * 0.85)), 6, (255, 255, 0), -1)
        
        # Información de estado avanzada
        info_lines = [
            f"IA: {'ON' if self.ia_enabled else 'OFF'} | Estado: {'ACTIVO' if self.robot_active else 'PAUSA'}",
            f"FPS: {self.current_fps:.1f} | Confianza: {global_confidence:.2f}",
            f"Comandos: X={x_vel:.2f} Y={y_vel:.2f} R={rot_vel:.2f}",
            f"Detecciones: {self.successful_detections}/{self.frame_count} ({self.successful_detections/max(self.frame_count,1)*100:.1f}%)",
            f"Perdida: {self.lost_line_count}/{self.max_lost_frames} | Buffer: {len(self.movement_buffer)}",
            f"Enviados: {self.commands_sent} | Uptime: {int(time.time() - self.start_time)}s"
        ]
        
        for i, line in enumerate(info_lines):
            y_pos = 15 + i * 16
            cv2.putText(debug_frame, line, (5, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return debug_frame

    def calculate_fps(self):
        """Cálculo de FPS en tiempo real"""
        self.fps_counter += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.fps_counter / (current_time - self.last_fps_time)
            self.fps_counter = 0
            self.last_fps_time = current_time

    def run_ai_system(self):
        """Bucle principal del sistema IA avanzado"""
        print("\n🚀 SISTEMA IA AVANZADO INICIADO")
        print("="*50)
        print("🎮 CONTROLES AVANZADOS:")
        print("  ESPACIO: Activar/pausar robot")
        print("  'S': Parada de emergencia")
        print("  'D': Toggle debug visual")
        print("  'I': Toggle IA")
        print("  'A': Toggle modo adaptativo")
        print("  '+': Aumentar velocidad")
        print("  '-': Disminuir velocidad")
        print("  'R': Reset estadísticas")
        print("  'Q': Salir del sistema")
        print("="*50)
        
        # Bucle principal
        while True:
            loop_start = time.time()
            self.frame_count += 1
            
            # Capturar frame optimizado
            frame = self.capture_frame_optimized()
            if frame is None:
                if self.debug_enabled and self.frame_count % 30 == 0:
                    print("⚠️ Sin frame disponible")
                time.sleep(0.05)
                continue
            
            # Procesar con IA avanzada
            binary = self.preprocess_advanced(frame)
            line_points, confidence_values = self.detect_line_ai_advanced(binary)
            
            # Variables de control
            x_vel, y_vel, rot_vel = 0, 0, 0
            ref_x, slope, global_confidence, line_angle = None, None, 0, 0
            
            # Ejecutar IA si está activa
            if self.robot_active and self.ia_enabled:
                if line_points:
                    # IA: Calcular trayectoria
                    ref_x, slope, global_confidence, line_angle = self.calculate_trajectory_ai(
                        line_points, confidence_values)
                    
                    if ref_x is not None:
                        self.successful_detections += 1
                    
                    # IA: Generar comandos de movimiento
                    x_vel, y_vel, rot_vel = self.ai_movement_controller(
                        ref_x, slope, global_confidence, line_angle)
                    
                    # Enviar comandos al robot
                    self.send_omnidirectional_command(x_vel, y_vel, rot_vel)
                else:
                    # Sin detecciones - manejar línea perdida
                    x_vel, y_vel, rot_vel = self.handle_lost_line_ai()
                    if x_vel != 0 or y_vel != 0 or rot_vel != 0:
                        self.send_omnidirectional_command(x_vel, y_vel, rot_vel)
            
            # Calcular FPS
            self.calculate_fps()
            
            # Mostrar debug si está activado
            if self.debug_enabled:
                debug_frame = self.draw_advanced_debug(
                    frame, binary, line_points, confidence_values,
                    ref_x, slope, global_confidence, x_vel, y_vel, rot_vel)
                
                try:
                    cv2.imshow('Robot IA - Cámara Principal', debug_frame)
                    cv2.imshow('Robot IA - Procesamiento', binary)
                except:
                    pass  # Ignorar errores de display
            
            # Estadísticas periódicas
            if self.frame_count % 100 == 0:
                success_rate = (self.successful_detections / self.frame_count) * 100
                print(f"📊 Frame {self.frame_count} | Éxito: {success_rate:.1f}% | "
                      f"FPS: {self.current_fps:.1f} | Comandos: {self.commands_sent}")
            
            # Manejar teclas
            try:
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord(' '):  # ESPACIO
                    self.robot_active = not self.robot_active
                    if not self.robot_active:
                        self.emergency_stop()
                    status = "ACTIVADO" if self.robot_active else "PAUSADO"
                    print(f"🎮 Robot {status}")
                    
                elif key == ord('s'):  # S - Stop
                    self.emergency_stop()
                    
                elif key == ord('d'):  # D - Debug
                    self.debug_enabled = not self.debug_enabled
                    status = "ON" if self.debug_enabled else "OFF"
                    print(f"🔍 Debug {status}")
                    
                elif key == ord('i'):  # I - IA
                    self.ia_enabled = not self.ia_enabled
                    status = "ACTIVADA" if self.ia_enabled else "DESACTIVADA"
                    print(f"🧠 IA {status}")
                    
                elif key == ord('a'):  # A - Adaptativo
                    self.adaptive_mode = not self.adaptive_mode
                    status = "ON" if self.adaptive_mode else "OFF"
                    print(f"🔄 Modo adaptativo {status}")
                    
                elif key == ord('+') or key == ord('='):  # Aumentar velocidad
                    self.speed_boost = min(2.0, self.speed_boost + 0.1)
                    print(f"⚡ Boost velocidad: {self.speed_boost:.1f}x")
                    
                elif key == ord('-'):  # Disminuir velocidad
                    self.speed_boost = max(0.3, self.speed_boost - 0.1)
                    print(f"⚡ Boost velocidad: {self.speed_boost:.1f}x")
                    
                elif key == ord('r'):  # R - Reset
                    self.frame_count = 0
                    self.successful_detections = 0
                    self.commands_sent = 0
                    self.start_time = time.time()
                    print("🔄 Estadísticas reseteadas")
                    
                elif key == ord('q'):  # Q - Quit
                    break
                    
            except:
                pass
            
            # Control de FPS del bucle
            elapsed = time.time() - loop_start
            target_fps = 15  # FPS objetivo
            if elapsed < 1/target_fps:
                time.sleep(1/target_fps - elapsed)
        
        # Cleanup final
        self.emergency_stop()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        
        # Estadísticas finales
        total_time = time.time() - self.start_time
        print("\n" + "="*50)
        print("📊 ESTADÍSTICAS FINALES")
        print(f"⏱️ Tiempo total: {total_time:.1f}s")
        print(f"🖼️ Frames procesados: {self.frame_count}")
        print(f"✅ Detecciones exitosas: {self.successful_detections}")
        print(f"📤 Comandos enviados: {self.commands_sent}")
        print(f"🎯 Tasa de éxito: {(self.successful_detections/max(self.frame_count,1))*100:.1f}%")
        print(f"📈 FPS promedio: {self.frame_count/total_time:.1f}")
        print("🏁 Sistema IA finalizado")

def main():
    print("🤖 ROBOT OMNIDIRECCIONAL IA - VERSIÓN FINAL")
    print("🌟 Características: Detección avanzada, predicción IA, control omnidireccional")
    print("📡 Tasa de éxito esperada: >80%")
    
    # IP del ESP32-CAM (la que funciona al 80%)
    ESP32_IP = "192.168.10.43"
    
    try:
        # Crear sistema IA avanzado
        robot = RobotOmnidireccionalIA(ESP32_IP)
        
        # Ejecutar sistema principal
        robot.run_ai_system()
        
    except KeyboardInterrupt:
        print("\n⚠️ Interrupción por teclado")
    except Exception as e:
        print(f"❌ Error crítico: {e}")
    finally:
        print("🔧 Limpieza del sistema completada")

if __name__ == "__main__":
    main()