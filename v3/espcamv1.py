"""
Robot Omnidireccional con Visión por Computadora
Sistema de seguimiento de líneas usando ESP32-CAM y Python

Características:
- Procesamiento de imágenes en tiempo real
- Binarización para detección de líneas negras
- Red neuronal simple para toma de decisiones
- Control omnidireccional del robot
- Manejo de cámara invertida
"""

import cv2
import numpy as np
import requests
import time
import threading
import json
from collections import deque
import os

class RobotVisionController:
    def __init__(self, esp32_ip="192.168.196.182"):
        self.esp32_ip = esp32_ip
        self.base_url = f"http://{esp32_ip}"
        self.running = False
        self.auto_mode = False
        
        # Parámetros de procesamiento de imagen
        self.img_width = 320
        self.img_height = 240
        self.roi_height = 80  # Altura de la región de interés
        
        # Historial de decisiones para suavizado
        self.decision_history = deque(maxlen=5)
        
        # Variables de control
        self.last_command_time = time.time()
        self.command_interval = 0.1  # Enviar comandos cada 100ms
        
        print(f"Controlador inicializado para ESP32 en: {self.base_url}")

    def start_vision_system(self):
        """Iniciar sistema de visión"""
        self.running = True
        
        # Activar modo automático en ESP32
        self.set_auto_mode(True)
        
        # Iniciar hilo de procesamiento de video
        vision_thread = threading.Thread(target=self.vision_loop)
        vision_thread.daemon = True
        vision_thread.start()
        
        print("Sistema de visión iniciado")
        
    def stop_vision_system(self):
        """Detener sistema de visión"""
        self.running = False
        self.set_auto_mode(False)
        self.send_stop_command()
        print("Sistema de visión detenido")
    
    def vision_loop(self):
        """Bucle principal de procesamiento de visión"""
        while self.running:
            try:
                # Capturar imagen desde ESP32-CAM
                image = self.capture_image()
                if image is None:
                    time.sleep(0.1)
                    continue
                
                # Procesar imagen
                processed_features = self.process_image(image)
                
                # Tomar decisión usando procesamiento tradicional
                decision = self.make_decision(processed_features)
                
                # Enviar comando al robot
                self.send_movement_command(decision)
                
                # Mostrar información básica en consola
                line_pos = processed_features['line_position']
                line_ahead = processed_features.get('line_position_ahead', 0)
                curvature = processed_features.get('curvature', 0)
                
                # Mostrar info completa cada 10 frames para no saturar
                if not hasattr(self, 'frame_count'):
                    self.frame_count = 0
                self.frame_count += 1
                
                if self.frame_count % 10 == 0:
                    print(f"🎯 Pos: {line_pos:+.2f} | Ahead: {line_ahead:+.2f} | Curve: {curvature:+.2f} | {decision:>12}")
                
                # Mostrar debug visual si está habilitado
                if hasattr(self, 'show_debug') and self.show_debug:
                    self.display_debug_info(image, processed_features, decision)
                
                time.sleep(0.05)  # 20 FPS
                
            except Exception as e:
                print(f"Error en bucle de visión: {e}")
                time.sleep(0.5)
    
    def capture_image(self):
        """Capturar imagen desde ESP32-CAM"""
        try:
            response = requests.get(f"{self.base_url}/capture", timeout=2)
            if response.status_code == 200:
                # Convertir bytes a imagen OpenCV
                img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
                image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                
                # Voltear imagen ya que la cámara está invertida
                image = cv2.flip(image, -1)  # Voltear horizontal y verticalmente
                
                # Redimensionar para procesamiento más rápido
                image = cv2.resize(image, (self.img_width, self.img_height))
                
                return image
            else:
                print(f"Error capturando imagen: {response.status_code}")
                return None
                
        except Exception as e:
            print(f"Error en captura: {e}")
            return None
    
    def process_image(self, image):
        """Procesar imagen para detección de líneas - OPTIMIZADO PARA CURVAS"""
        # Convertir a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Mejorar contraste para líneas más claras
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        
        # Aplicar desenfoque gaussiano para reducir ruido
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        
        # Binarización adaptativa mejorada para líneas negras
        binary = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 10
        )
        
        # Operaciones morfológicas para limpiar línea
        kernel = np.ones((3,3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # Definir múltiples ROIs para mejor detección de curvas
        h, w = binary.shape
        roi_height = self.roi_height
        
        # ROI principal (parte inferior)
        roi_main = binary[h-roi_height:, :]
        
        # ROI adelantado (para ver curvas)
        roi_ahead = binary[h-roi_height-30:h-roi_height+10, :]
        
        # Calcular posición de la línea en ambas ROIs
        line_position_main = self.calculate_line_position(roi_main)
        line_position_ahead = self.calculate_line_position(roi_ahead) if roi_ahead.shape[0] > 0 else 0
        
        # Detectar curvatura
        curvature = self.detect_curvature(binary)
        
        return {
            'binary': binary,
            'roi_main': roi_main,
            'roi_ahead': roi_ahead,
            'line_position': line_position_main,
            'line_position_ahead': line_position_ahead,
            'curvature': curvature
        }
    
    def detect_curvature(self, binary_image):
        """Detectar curvatura de la línea"""
        h, w = binary_image.shape
        
        # Dividir imagen en 3 secciones horizontales
        section_height = h // 3
        sections = [
            binary_image[i*section_height:(i+1)*section_height, :] 
            for i in range(3)
        ]
        
        # Calcular centro de línea en cada sección
        centers = []
        for section in sections:
            if section.shape[0] > 0:
                center = self.calculate_line_position(section)
                centers.append(center)
        
        if len(centers) >= 3:
            # Calcular curvatura aproximada
            curvature = (centers[0] - 2*centers[1] + centers[2])
            return curvature
        
        return 0
    
    def calculate_line_position(self, roi):
        """Calcular posición de la línea - MEJORADO PARA CURVAS"""
        if roi.shape[0] == 0:
            return 0
            
        h, w = roi.shape
        
        # Usar múltiples filas para mejor precisión
        bottom_section = roi[-min(20, h):, :]  # Últimas 20 filas o menos
        
        # Encontrar contornos
        contours, _ = cv2.findContours(bottom_section, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Encontrar el contorno más grande (línea principal)
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 10:  # Filtrar ruido
                # Calcular momento para encontrar centroide
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = M['m10'] / M['m00']
                    # Normalizar posición (-1 = izquierda, 0 = centro, 1 = derecha)
                    normalized_position = (cx - w/2) / (w/2)
                    return np.clip(normalized_position, -1, 1)
        
        # Método alternativo: buscar píxeles blancos
        white_pixels = np.where(bottom_section > 0)
        
        if len(white_pixels[1]) > 0:
            line_center = np.mean(white_pixels[1])
            normalized_position = (line_center - w/2) / (w/2)
            return np.clip(normalized_position, -1, 1)
        
        return 0  # Sin línea detectada
    
    def make_decision(self, processed_data):
        """Tomar decisión de movimiento - MEJORADO PARA CURVAS"""
        try:
            line_position = processed_data['line_position']
            line_ahead = processed_data['line_position_ahead']
            curvature = processed_data['curvature']
            
            # Umbrales adaptativos
            straight_threshold = 0.1
            turn_threshold = 0.25
            sharp_turn_threshold = 0.5
            
            # Detectar tipo de curva
            if abs(curvature) > 0.3:
                # Curva pronunciada detectada
                if curvature > 0:
                    decision = 'sharp_right'
                else:
                    decision = 'sharp_left'
            elif abs(line_position) > sharp_turn_threshold:
                # Giro cerrado
                if line_position > 0:
                    decision = 'sharp_right'
                else:
                    decision = 'sharp_left'
            elif abs(line_position) > turn_threshold:
                # Giro normal
                if line_position > 0:
                    decision = 'right'
                else:
                    decision = 'left'
            elif abs(line_position) > straight_threshold:
                # Corrección suave
                if line_position > 0:
                    decision = 'slight_right'
                else:
                    decision = 'slight_left'
            else:
                # Línea recta
                decision = 'forward'
            
            # Considerar línea adelantada para anticipar curvas
            if abs(line_ahead) > turn_threshold and abs(line_position) < straight_threshold:
                if line_ahead > 0:
                    decision = 'prepare_right'
                else:
                    decision = 'prepare_left'
            
            # Suavizar decisiones
            self.decision_history.append(decision)
            if len(self.decision_history) >= 3:
                # Si hay cambios bruscos, usar decisión más conservadora
                recent_decisions = list(self.decision_history)[-3:]
                if len(set(recent_decisions)) == 1:
                    smoothed_decision = decision
                else:
                    # Usar decisión más común o conservadora
                    smoothed_decision = max(set(self.decision_history), key=self.decision_history.count)
            else:
                smoothed_decision = decision
            
            return smoothed_decision
            
        except Exception as e:
            print(f"Error en toma de decisión: {e}")
            return 'forward'
    
    def traditional_decision(self, line_position):
        """Método tradicional de seguimiento de líneas - DEPRECADO"""
        # Esta función se mantiene por compatibilidad pero ya no se usa
        # La lógica está integrada en make_decision()
        threshold = 0.15
        
        if line_position < -threshold:
            return 'left'
        elif line_position > threshold:
            return 'right'
        else:
            return 'forward'
    
    def send_movement_command(self, decision):
        """Enviar comando de movimiento al robot - COMANDOS MEJORADOS"""
        current_time = time.time()
        
        # Limitar frecuencia de comandos
        if current_time - self.last_command_time < self.command_interval:
            return
        
        try:
            # Mapear decisiones a comandos específicos
            speed_map = {
                'forward': {'cmd': 'forward', 'speed': 180},
                'left': {'cmd': 'left', 'speed': 150},
                'right': {'cmd': 'right', 'speed': 150},
                'slight_left': {'cmd': 'left', 'speed': 120},
                'slight_right': {'cmd': 'right', 'speed': 120},
                'sharp_left': {'cmd': 'rotate_left', 'speed': 200},
                'sharp_right': {'cmd': 'rotate_right', 'speed': 200},
                'prepare_left': {'cmd': 'left', 'speed': 100},
                'prepare_right': {'cmd': 'right', 'speed': 100},
                'stop': {'cmd': 'stop', 'speed': 0}
            }
            
            cmd_data = speed_map.get(decision, speed_map['forward'])
            
            # Enviar comando
            response = requests.post(
                f"{self.base_url}/move",
                json={
                    'direction': cmd_data['cmd'],
                    'speed': cmd_data['speed']
                },
                timeout=1
            )
            
            if response.status_code == 200:
                self.last_command_time = current_time
                # Solo imprimir comandos importantes para reducir spam
                if decision in ['sharp_left', 'sharp_right', 'forward']:
                    print(f">>> {decision.upper()}")
            
        except Exception as e:
            print(f"Error enviando comando: {e}")
    
    def set_auto_mode(self, enabled):
        """Activar/desactivar modo automático"""
        try:
            response = requests.post(
                f"{self.base_url}/auto",
                json={'auto': enabled},
                timeout=2
            )
            
            if response.status_code == 200:
                self.auto_mode = enabled
                print(f"Modo automático: {'ON' if enabled else 'OFF'}")
            
        except Exception as e:
            print(f"Error configurando modo automático: {e}")
    
    def send_stop_command(self):
        """Enviar comando de parada"""
        try:
            requests.post(f"{self.base_url}/stop", timeout=1)
            print("Comando de parada enviado")
        except Exception as e:
            print(f"Error enviando parada: {e}")
    
    def display_debug_info(self, original_image, processed_data, decision):
        """Mostrar información de depuración mejorada"""
        line_pos = processed_data['line_position']
        line_ahead = processed_data.get('line_position_ahead', 0)
        curvature = processed_data.get('curvature', 0)
        
        print(f"🎯 Línea: {line_pos:+.3f} | Adelante: {line_ahead:+.3f} | Curva: {curvature:+.3f} | {decision:>12}")
        
        # Solo intentar mostrar imágenes si OpenCV GUI está disponible
        try:
            # Crear imagen de depuración
            debug_img = original_image.copy()
            
            # Dibujar ROI principal
            roi_y = self.img_height - self.roi_height
            cv2.rectangle(debug_img, (0, roi_y), (self.img_width, self.img_height), (0, 255, 0), 2)
            
            # Dibujar ROI adelantado
            roi_ahead_y = roi_y - 30
            cv2.rectangle(debug_img, (0, roi_ahead_y), (self.img_width, roi_y + 10), (255, 0, 0), 1)
            
            # Mostrar posición de línea actual
            center_x = int(self.img_width / 2 + line_pos * self.img_width / 2)
            cv2.circle(debug_img, (center_x, self.img_height - 20), 8, (0, 0, 255), -1)
            
            # Mostrar posición adelantada
            ahead_x = int(self.img_width / 2 + line_ahead * self.img_width / 2)
            cv2.circle(debug_img, (ahead_x, roi_ahead_y + 20), 6, (255, 0, 255), -1)
            
            # Mostrar información de texto
            cv2.putText(debug_img, f"Decision: {decision}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_img, f"Pos: {line_pos:.2f}", (10, 55), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(debug_img, f"Ahead: {line_ahead:.2f}", (10, 75), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            cv2.putText(debug_img, f"Curve: {curvature:.2f}", (10, 95), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Mostrar imágenes
            cv2.imshow('Debug View', debug_img)
            cv2.imshow('Binary', processed_data['binary'])
            
            # Mostrar ROIs si están disponibles
            if 'roi_main' in processed_data:
                cv2.imshow('ROI Main', processed_data['roi_main'])
            if 'roi_ahead' in processed_data and processed_data['roi_ahead'].shape[0] > 0:
                cv2.imshow('ROI Ahead', processed_data['roi_ahead'])
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop_vision_system()
                
        except cv2.error as e:
            if "is not implemented" in str(e):
                print("ℹ️  Debug visual deshabilitado (OpenCV sin GUI)")
                # Desactivar debug automáticamente
                self.show_debug = False
            else:
                print(f"Error OpenCV: {e}")

def test_connection(esp32_ip):
    """Probar conexión con ESP32"""
    try:
        response = requests.get(f"http://{esp32_ip}", timeout=5)
        if response.status_code == 200:
            print(f"✅ Conexión exitosa con ESP32 en {esp32_ip}")
            return True
        else:
            print(f"❌ ESP32 respondió con código: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Error conectando con ESP32: {e}")
        return False

def main():
    # Configurar IP del ESP32-CAM
    ESP32_IP = input("Ingresa la IP del ESP32-CAM [192.168.196.182]: ").strip()
    if not ESP32_IP:
        ESP32_IP = "192.168.196.182"  # IP detectada de tu ESP32
    
    print("=== ROBOT OMNIDIRECCIONAL CON VISION ===")
    print(f"Intentando conectar con ESP32-CAM en: {ESP32_IP}")
    
    # Probar conexión
    if not test_connection(ESP32_IP):
        print("No se pudo conectar con el ESP32-CAM")
        print("Verifica:")
        print("1. Que el ESP32-CAM esté encendido")
        print("2. Que esté conectado al WiFi")
        print("3. Que la IP sea correcta")
        return
    
    # Crear controlador
    controller = RobotVisionController(ESP32_IP)
    
    try:
        print("\n=== INICIANDO SISTEMA ===")
        print("Comandos disponibles:")
        print("- 'q' para salir")
        print("- 'd' para activar/desactivar debug")
        print("- 's' para parar robot")
        print("- 'r' para reiniciar")
        
        # Habilitar debug por defecto
        controller.show_debug = False  # Desactivado para evitar errores OpenCV
        
        # Iniciar sistema de visión
        controller.start_vision_system()
        
        # Bucle de control
        while controller.running:
            try:
                comando = input("\nComando: ").lower().strip()
                
                if comando == 'q':
                    print("Saliendo...")
                    break
                elif comando == 'd':
                    controller.show_debug = not getattr(controller, 'show_debug', False)
                    print(f"Debug: {'ON' if controller.show_debug else 'OFF'}")
                elif comando == 's':
                    print("Parando robot...")
                    controller.send_stop_command()
                elif comando == 'r':
                    print("Reiniciando sistema...")
                    controller.stop_vision_system()
                    time.sleep(1)
                    controller.start_vision_system()
                elif comando == '':
                    continue
                else:
                    print("Comando no reconocido")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                print("\nInterrumpido por teclado")
                break
    
    except Exception as e:
        print(f"Error general: {e}")
    
    finally:
        print("\n=== FINALIZANDO SISTEMA ===")
        controller.stop_vision_system()
        cv2.destroyAllWindows()
        print("Sistema finalizado correctamente")

if __name__ == "__main__":
    main()