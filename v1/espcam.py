import cv2
import numpy as np
import requests
import json
import time
import math
from collections import deque
import threading

class RobotVisionLineaNegra:
    def __init__(self, esp32_ip="192.168.182.182:80"):
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
        
        # IA: Sistema avanzado para línea negra sobre fondo blanco
        self.line_history = deque(maxlen=15)
        self.curve_history = deque(maxlen=8)
        self.direction_buffer = deque(maxlen=5)
        
        # Parámetros de visión para línea negra
        self.binary_threshold = 60    # Umbral para detectar negro
        self.min_line_area = 150      # Área mínima de línea
        self.max_line_area = 8000     # Área máxima de línea
        self.roi_height_ratio = 0.6   # Región de interés (60% inferior)
        
        # Control omnidireccional optimizado para pista compleja
        self.base_speed = 0.25         # Velocidad base reducida para precisión
        self.curve_speed = 0.15        # Velocidad en curvas cerradas
        self.straight_speed = 0.35     # Velocidad en rectas
        self.lateral_gain = 0.7        # Ganancia lateral
        self.rotation_gain = 0.5       # Ganancia rotacional
        self.max_speed = 0.6           # Velocidad máxima
        
        # Estados del sistema
        self.robot_active = False
        self.vision_active = True
        self.debug_mode = True
        self.auto_speed = True
        
        # Métricas de rendimiento
        self.frame_count = 0
        self.detection_count = 0
        self.command_count = 0
        self.start_time = time.time()
        self.lost_line_count = 0
        self.max_lost_frames = 25
        
        # Variables de análisis de curva
        self.current_curve_type = "STRAIGHT"  # STRAIGHT, LEFT, RIGHT, SHARP_LEFT, SHARP_RIGHT
        self.curve_confidence = 0.0
        
        print("🤖 ROBOT VISIÓN ARTIFICIAL - LÍNEA NEGRA")
        print("="*50)
        print(f"🎯 ESP32-CAM: {esp32_ip}")
        print(f"⚫ Optimizado para: LÍNEA NEGRA sobre FONDO BLANCO")
        print(f"🏁 Tipo de pista: COMPLEJA con curvas y zigzag")
        
        self.verify_connection()

    def verify_connection(self):
        """Verificar conexión con el robot"""
        print("\n🔍 Verificando conexión con robot...")
        try:
            response = requests.get(self.status_url, timeout=5)
            if response.status_code == 200:
                status = response.json()
                print("✅ Robot conectado y funcionando")
                print(f"📊 Estado: {status}")
                
                # Test de captura
                print("📸 Probando captura de imagen...")
                capture_response = requests.get(self.capture_url, timeout=8)
                if capture_response.status_code == 200:
                    print(f"✅ Cámara OK - {len(capture_response.content)} bytes")
                    return True
                else:
                    print(f"❌ Error cámara: {capture_response.status_code}")
            else:
                print(f"❌ Robot no responde: {response.status_code}")
        except Exception as e:
            print(f"❌ Error de conexión: {e}")
            print("🔧 Verifica que el ESP32-CAM esté encendido y conectado")
        return False

    def capture_frame(self):
        """Captura frame optimizada con retry"""
        for attempt in range(3):
            try:
                response = requests.get(self.capture_url, timeout=3)
                if response.status_code == 200:
                    img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    if frame is not None and frame.shape[0] > 0:
                        return frame
            except Exception as e:
                if attempt == 2:
                    print(f"⚠️ Error captura: {str(e)[:50]}...")
                time.sleep(0.1)
        return None

    def preprocess_for_black_line(self, frame):
        """Preprocesamiento específico para línea negra sobre fondo blanco"""
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Aplicar desenfoque gaussiano para reducir ruido
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Binarización: negro=255 (línea), blanco=0 (fondo)
        # Invertimos para que la línea negra sea blanca en la imagen binaria
        _, binary = cv2.threshold(blurred, self.binary_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # Operaciones morfológicas para limpiar
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        # Filtro adicional para eliminar ruido pequeño
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)
        
        return binary

    def detect_black_line(self, binary):
        """Detección avanzada de línea negra con análisis de forma"""
        height, width = binary.shape
        
        # Región de interés - parte inferior donde está la línea
        roi_start = int(height * (1 - self.roi_height_ratio))
        roi = binary[roi_start:height, :]
        
        # Encontrar contornos de la línea
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        line_points = []
        line_contours = []
        
        if contours:
            # Filtrar contornos por área y forma
            valid_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filtro por área
                if self.min_line_area < area < self.max_line_area:
                    # Análisis de forma - las líneas suelen ser alargadas
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0
                    
                    # Filtrar formas que parecen líneas
                    if aspect_ratio > 1.2:  # Formas alargadas
                        valid_contours.append((contour, area))
            
            # Ordenar por área (más grande = más probable que sea la línea principal)
            valid_contours.sort(key=lambda x: x[1], reverse=True)
            
            # Procesar los contornos más probables
            for contour, area in valid_contours[:3]:  # Máximo 3 segmentos de línea
                # Calcular centroide
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = roi_start + int(M["m01"] / M["m00"])
                    
                    line_points.append((cx, cy))
                    line_contours.append(contour)
        
        return line_points, line_contours

    def analyze_curve_type(self, line_points):
        """Análisis del tipo de curva basado en los puntos detectados"""
        if len(line_points) < 2:
            return "UNKNOWN", 0.0
        
        # Calcular la tendencia de los puntos
        x_coords = [p[0] for p in line_points]
        y_coords = [p[1] for p in line_points]
        
        # Regresión lineal para obtener pendiente
        if len(x_coords) >= 2:
            try:
                slope = np.polyfit(y_coords, x_coords, 1)[0]
                
                # Analizar curvatura si hay suficientes puntos
                curvature = 0
                if len(x_coords) >= 3:
                    # Aproximar curvatura con regresión cuadrática
                    try:
                        quad_coeffs = np.polyfit(y_coords, x_coords, 2)
                        curvature = abs(quad_coeffs[0])  # Coeficiente cuadrático
                    except:
                        curvature = 0
                
                # Clasificar tipo de curva
                confidence = min(len(line_points) / 5.0, 1.0)
                
                if curvature > 0.002:  # Curva pronunciada
                    if slope > 0.5:
                        return "SHARP_RIGHT", confidence
                    elif slope < -0.5:
                        return "SHARP_LEFT", confidence
                    else:
                        return "SHARP_CURVE", confidence
                elif abs(slope) > 0.3:  # Curva suave
                    if slope > 0:
                        return "RIGHT", confidence
                    else:
                        return "LEFT", confidence
                else:  # Línea recta
                    return "STRAIGHT", confidence
                    
            except:
                return "UNKNOWN", 0.0
        
        return "UNKNOWN", 0.0

    def calculate_line_center(self, line_points):
        """Calcula el centro de la línea para seguimiento"""
        if not line_points:
            return None, None
        
        # Punto de referencia para control (más cerca del robot)
        reference_y = self.frame_height * 0.85
        
        if len(line_points) == 1:
            return line_points[0][0], line_points[0][1]
        
        # Regresión lineal para encontrar posición en Y de referencia
        x_coords = np.array([p[0] for p in line_points])
        y_coords = np.array([p[1] for p in line_points])
        
        try:
            # Regresión lineal
            coeffs = np.polyfit(y_coords, x_coords, 1)
            reference_x = coeffs[0] * reference_y + coeffs[1]
            
            # Asegurar que esté dentro de los límites
            reference_x = np.clip(reference_x, 0, self.frame_width)
            
            return reference_x, reference_y
        except:
            # Fallback a promedio simple
            avg_x = np.mean(x_coords)
            avg_y = np.mean(y_coords)
            return avg_x, avg_y

    def calculate_omnidirectional_control(self, line_center_x, line_center_y, curve_type):
        """Cálculo de control omnidireccional basado en la línea detectada"""
        if line_center_x is None:
            return self.handle_lost_line()
        
        # Error lateral (posición de la línea respecto al centro)
        lateral_error = (line_center_x - self.center_x) / self.center_x
        
        # Velocidad adaptativa según tipo de curva
        if curve_type in ["SHARP_LEFT", "SHARP_RIGHT", "SHARP_CURVE"]:
            forward_speed = self.curve_speed
        elif curve_type in ["LEFT", "RIGHT"]:
            forward_speed = (self.base_speed + self.curve_speed) / 2
        else:  # STRAIGHT
            forward_speed = self.straight_speed if self.auto_speed else self.base_speed
        
        # Control omnidireccional
        # X: Movimiento lateral para centrar en la línea
        x_velocity = -lateral_error * self.lateral_gain
        
        # Y: Velocidad hacia adelante adaptativa
        y_velocity = forward_speed
        
        # Rotación: Corrección angular basada en error lateral
        rotation_velocity = -lateral_error * self.rotation_gain
        
        # Suavizado temporal
        self.direction_buffer.append((x_velocity, y_velocity, rotation_velocity))
        if len(self.direction_buffer) > 1:
            # Promedio ponderado para suavizar movimientos
            recent = list(self.direction_buffer)[-3:]
            weights = [0.5, 0.7, 1.0][:len(recent)]
            weights = np.array(weights) / sum(weights)
            
            movements = np.array(recent)
            x_velocity = np.average(movements[:, 0], weights=weights)
            y_velocity = np.average(movements[:, 1], weights=weights)
            rotation_velocity = np.average(movements[:, 2], weights=weights)
        
        # Limitar velocidades
        x_velocity = np.clip(x_velocity, -self.max_speed, self.max_speed)
        y_velocity = np.clip(y_velocity, 0, self.max_speed)
        rotation_velocity = np.clip(rotation_velocity, -self.max_speed, self.max_speed)
        
        # Reset contador de línea perdida
        self.lost_line_count = 0
        
        return x_velocity, y_velocity, rotation_velocity

    def handle_lost_line(self):
        """Manejo cuando se pierde la línea"""
        self.lost_line_count += 1
        
        if self.lost_line_count < self.max_lost_frames:
            # Continuar con el último movimiento conocido pero reducido
            if self.direction_buffer:
                last_movement = self.direction_buffer[-1]
                reduction_factor = 0.5 * (1.0 - self.lost_line_count / self.max_lost_frames)
                
                x_vel = last_movement[0] * reduction_factor
                y_vel = self.base_speed * 0.3  # Velocidad muy lenta
                rot_vel = last_movement[2] * reduction_factor
                
                return x_vel, y_vel, rot_vel
            else:
                return 0, 0.1, 0  # Movimiento mínimo hacia adelante
        else:
            # Parar completamente
            print(f"❌ Línea perdida por {self.lost_line_count} frames - PARADA")
            return 0, 0, 0

    def send_control_command(self, x, y, rotation):
        """Enviar comando al robot"""
        try:
            command = {
                "x": float(x),
                "y": float(y),
                "rotation": float(rotation)
            }
            
            response = requests.post(
                self.control_url,
                json=command,
                headers={'Content-Type': 'application/json'},
                timeout=1.5
            )
            
            if response.status_code == 200:
                self.command_count += 1
                return True
            else:
                print(f"❌ Error comando HTTP: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ Error enviando comando: {str(e)[:40]}...")
            return False

    def emergency_stop(self):
        """Parada de emergencia"""
        try:
            response = requests.get(self.stop_url, timeout=2)
            print("🛑 PARADA DE EMERGENCIA")
            self.robot_active = False
        except Exception as e:
            print(f"⚠️ Error en parada: {e}")

    def draw_debug_overlay(self, frame, binary, line_points, line_center, curve_type, x_vel, y_vel, rot_vel):
        """Dibujar información de debug"""
        debug_frame = frame.copy()
        
        # ROI
        roi_start = int(self.frame_height * (1 - self.roi_height_ratio))
        cv2.rectangle(debug_frame, (0, roi_start), (self.frame_width, self.frame_height), (0, 255, 255), 1)
        
        # Centro de referencia
        cv2.circle(debug_frame, (self.center_x, self.center_y), 3, (0, 255, 0), -1)
        cv2.line(debug_frame, (self.center_x, 0), (self.center_x, self.frame_height), (0, 255, 0), 1)
        
        # Puntos de línea detectados
        for i, point in enumerate(line_points):
            cv2.circle(debug_frame, point, 4, (255, 0, 0), -1)
            cv2.putText(debug_frame, str(i), (point[0]+5, point[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)
        
        # Centro de línea calculado
        if line_center[0] is not None:
            center_point = (int(line_center[0]), int(line_center[1]))
            cv2.circle(debug_frame, center_point, 6, (0, 255, 255), -1)
            cv2.line(debug_frame, (self.center_x, self.center_y), center_point, (0, 255, 255), 2)
        
        # Información de estado
        info_lines = [
            f"Estado: {'ACTIVO' if self.robot_active else 'PAUSADO'}",
            f"Curva: {curve_type} ({self.curve_confidence:.2f})",
            f"Control: X={x_vel:.2f} Y={y_vel:.2f} R={rot_vel:.2f}",
            f"Detecciones: {self.detection_count}/{self.frame_count}",
            f"Perdida: {self.lost_line_count}/{self.max_lost_frames}",
            f"Comandos: {self.command_count}"
        ]
        
        for i, line in enumerate(info_lines):
            y_pos = 15 + i * 16
            cv2.putText(debug_frame, line, (5, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return debug_frame

    def run_vision_system(self):
        """Bucle principal del sistema de visión"""
        print("\n🚀 INICIANDO SISTEMA DE VISIÓN ARTIFICIAL")
        print("="*60)
        print("🎮 CONTROLES:")
        print("  ESPACIO: Activar/pausar robot")
        print("  'S': Parada de emergencia")
        print("  'D': Toggle debug")
        print("  'A': Toggle velocidad automática")
        print("  '+/-': Ajustar velocidad base")
        print("  'Q': Salir")
        print("="*60)
        
        fps_counter = 0
        fps_time = time.time()
        
        while True:
            loop_start = time.time()
            self.frame_count += 1
            
            # Capturar frame
            frame = self.capture_frame()
            if frame is None:
                time.sleep(0.05)
                continue
            
            # Procesar imagen para línea negra
            binary = self.preprocess_for_black_line(frame)
            line_points, line_contours = self.detect_black_line(binary)
            
            # Variables de control
            x_vel, y_vel, rot_vel = 0, 0, 0
            line_center = (None, None)
            curve_type = "UNKNOWN"
            
            if line_points:
                self.detection_count += 1
                
                # Calcular centro de línea y tipo de curva
                line_center = self.calculate_line_center(line_points)
                curve_type, self.curve_confidence = self.analyze_curve_type(line_points)
                self.current_curve_type = curve_type
                
                # Control del robot
                if self.robot_active:
                    x_vel, y_vel, rot_vel = self.calculate_omnidirectional_control(
                        line_center[0], line_center[1], curve_type)
                    self.send_control_command(x_vel, y_vel, rot_vel)
            else:
                # Línea no detectada
                if self.robot_active:
                    x_vel, y_vel, rot_vel = self.handle_lost_line()
                    if x_vel != 0 or y_vel != 0 or rot_vel != 0:
                        self.send_control_command(x_vel, y_vel, rot_vel)
            
            # Mostrar debug
            if self.debug_mode:
                debug_frame = self.draw_debug_overlay(frame, binary, line_points, 
                                                    line_center, curve_type, x_vel, y_vel, rot_vel)
                try:
                    cv2.imshow('Robot Vision - Linea Negra', debug_frame)
                    cv2.imshow('Procesamiento Binario', binary)
                except:
                    pass
            
            # Calcular FPS
            fps_counter += 1
            if time.time() - fps_time >= 1.0:
                fps = fps_counter / (time.time() - fps_time)
                if self.frame_count % 30 == 0:
                    success_rate = (self.detection_count / self.frame_count) * 100
                    print(f"📊 FPS: {fps:.1f} | Detección: {success_rate:.1f}% | Curva: {self.current_curve_type}")
                fps_counter = 0
                fps_time = time.time()
            
            # Manejar controles
            try:
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord(' '):
                    self.robot_active = not self.robot_active
                    if not self.robot_active:
                        self.emergency_stop()
                    print(f"🎮 Robot: {'ACTIVADO' if self.robot_active else 'PAUSADO'}")
                    
                elif key == ord('s'):
                    self.emergency_stop()
                    
                elif key == ord('d'):
                    self.debug_mode = not self.debug_mode
                    print(f"🔍 Debug: {'ON' if self.debug_mode else 'OFF'}")
                    
                elif key == ord('a'):
                    self.auto_speed = not self.auto_speed
                    print(f"⚡ Velocidad automática: {'ON' if self.auto_speed else 'OFF'}")
                    
                elif key == ord('+') or key == ord('='):
                    self.base_speed = min(0.8, self.base_speed + 0.05)
                    print(f"🏎️ Velocidad base: {self.base_speed:.2f}")
                    
                elif key == ord('-'):
                    self.base_speed = max(0.1, self.base_speed - 0.05)
                    print(f"🐌 Velocidad base: {self.base_speed:.2f}")
                    
                elif key == ord('q'):
                    break
            except:
                pass
            
            # Control de FPS (15 FPS target)
            elapsed = time.time() - loop_start
            if elapsed < 1/15:
                time.sleep(1/15 - elapsed)
        
        # Cleanup
        self.emergency_stop()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        
        # Estadísticas finales
        total_time = time.time() - self.start_time
        print("\n" + "="*60)
        print("📊 ESTADÍSTICAS FINALES")
        print(f"⏱️ Tiempo total: {total_time:.1f}s")
        print(f"🖼️ Frames procesados: {self.frame_count}")
        print(f"✅ Detecciones exitosas: {self.detection_count}")
        print(f"📤 Comandos enviados: {self.command_count}")
        print(f"🎯 Tasa de detección: {(self.detection_count/max(self.frame_count,1))*100:.1f}%")
        print(f"📈 FPS promedio: {self.frame_count/total_time:.1f}")
        print("🏁 Sistema de visión finalizado")

def main():
    print("🤖 ROBOT VISIÓN ARTIFICIAL - LÍNEA NEGRA v2.0")
    print("🎯 Optimizado para pistas complejas con curvas y zigzag")
    print("⚫ Detección: LÍNEA NEGRA sobre FONDO BLANCO")
    
    ESP32_IP = "192.168.10.43"
    
    try:
        robot = RobotVisionLineaNegra(ESP32_IP)
        robot.run_vision_system()
    except KeyboardInterrupt:
        print("\n⚠️ Interrupción por teclado")
    except Exception as e:
        print(f"❌ Error crítico: {e}")
    finally:
        print("🔧 Sistema finalizado")

if __name__ == "__main__":
    main()