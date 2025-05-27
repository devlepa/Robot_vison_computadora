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
    def __init__(self, esp32_ip="192.168.1.100"):
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
                print(f"🎯 Línea: {line_pos:+.3f} | Decisión: {decision:>8}")
                
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
        """Procesar imagen para detección de líneas"""
        # Convertir a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Aplicar desenfoque gaussiano para reducir ruido
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Binarización adaptativa para detectar línea negra sobre fondo blanco
        binary = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2
        )
        
        # Definir región de interés (parte inferior de la imagen)
        roi_y_start = self.img_height - self.roi_height
        roi = binary[roi_y_start:, :]
        
        # Calcular posición de la línea
        line_position = self.calculate_line_position(roi)
        
        return {
            'binary': binary,
            'roi': roi,
            'line_position': line_position
        }
    
    def calculate_line_position(self, roi):
        """Calcular posición de la línea"""
        h, w = roi.shape
        
        # Buscar línea en la parte inferior del ROI
        bottom_rows = roi[-15:, :]  # Últimas 15 filas
        
        # Encontrar centro de masa de píxeles blancos
        white_pixels = np.where(bottom_rows > 0)
        
        if len(white_pixels[1]) > 0:
            line_center = np.mean(white_pixels[1])
            # Normalizar posición (-1 = izquierda, 0 = centro, 1 = derecha)
            normalized_position = (line_center - w/2) / (w/2)
            return np.clip(normalized_position, -1, 1)
        
        return 0  # Sin línea detectada, asumir centro
    
    def make_decision(self, processed_data):
        """Tomar decisión de movimiento"""
        try:
            line_position = processed_data['line_position']
            decision = self.traditional_decision(line_position)
            
            # Suavizar decisiones
            self.decision_history.append(decision)
            smoothed_decision = max(set(self.decision_history), key=self.decision_history.count)
            
            return smoothed_decision
            
        except Exception as e:
            print(f"Error en toma de decisión: {e}")
            return 'forward'
    
    def traditional_decision(self, line_position):
        """Método tradicional de seguimiento de líneas"""
        threshold = 0.15
        
        if line_position < -threshold:
            return 'left'
        elif line_position > threshold:
            return 'right'
        else:
            return 'forward'
    
    def send_movement_command(self, decision):
        """Enviar comando de movimiento al robot"""
        current_time = time.time()
        
        # Limitar frecuencia de comandos
        if current_time - self.last_command_time < self.command_interval:
            return
        
        try:
            # Enviar comando
            response = requests.post(
                f"{self.base_url}/move",
                json={
                    'direction': decision,
                    'speed': 180
                },
                timeout=1
            )
            
            if response.status_code == 200:
                self.last_command_time = current_time
                # print(f"Comando enviado: {decision}")  # Comentado para reducir spam
            
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
        """Mostrar información de depuración (solo consola)"""
        line_pos = processed_data['line_position']
        print(f"🎯 Línea: {line_pos:+.3f} | Decisión: {decision:>8} | Imagen: {original_image.shape}")
        
        # Solo intentar mostrar imágenes si OpenCV GUI está disponible
        try:
            # Crear imagen de depuración
            debug_img = original_image.copy()
            
            # Dibujar ROI
            roi_y = self.img_height - self.roi_height
            cv2.rectangle(debug_img, (0, roi_y), (self.img_width, self.img_height), (0, 255, 0), 2)
            
            # Mostrar posición de línea
            center_x = int(self.img_width / 2 + line_pos * self.img_width / 2)
            cv2.circle(debug_img, (center_x, self.img_height - 20), 10, (0, 0, 255), -1)
            
            # Mostrar decisión
            cv2.putText(debug_img, f"Decision: {decision}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(debug_img, f"Line pos: {line_pos:.2f}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Mostrar imágenes
            cv2.imshow('Original + Debug', debug_img)
            cv2.imshow('Binary', processed_data['binary'])
            cv2.imshow('ROI', processed_data['roi'])
            
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