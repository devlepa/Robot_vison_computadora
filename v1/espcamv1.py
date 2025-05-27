import cv2
import numpy as np
import requests
import json
import time
import threading

class TestMovimientoRobot:
    def __init__(self, esp32_ip="192.168.182.182"):  # IP CORREGIDA
        self.esp32_ip = esp32_ip
        self.capture_url = f"http://{esp32_ip}/capture"
        self.control_url = f"http://{esp32_ip}/control"
        self.stop_url = f"http://{esp32_ip}/stop"
        self.status_url = f"http://{esp32_ip}/status"
        
        # Variables de control
        self.robot_active = False
        self.test_mode = "MANUAL"  # MANUAL, AUTO_PATTERN, CAMERA_REACTIVE
        self.command_count = 0
        
        print("🤖 TEST MOVIMIENTO ROBOT")
        print("="*40)
        print(f"🌐 ESP32-CAM: {esp32_ip}")
        print(f"🎯 Puerto: 80")
        
        self.verify_connection()

    def verify_connection(self):
        """Verificar conexión con IP corregida"""
        print(f"\n🔍 Verificando conexión con {self.esp32_ip}...")
        try:
            response = requests.get(self.status_url, timeout=5)
            if response.status_code == 200:
                status = response.json()
                print("✅ ¡CONEXIÓN EXITOSA!")
                print(f"📊 Estado: {status}")
                
                # Test de captura
                print("📸 Probando captura...")
                capture_response = requests.get(self.capture_url, timeout=5)
                if capture_response.status_code == 200:
                    print(f"✅ Cámara OK - {len(capture_response.content)} bytes")
                    return True
                else:
                    print(f"❌ Error cámara: {capture_response.status_code}")
            else:
                print(f"❌ Error HTTP: {response.status_code}")
        except Exception as e:
            print(f"❌ Error: {e}")
        return False

    def send_motor_command(self, x, y, rotation, duration=2.0):
        """Enviar comando de movimiento con duración"""
        try:
            command = {
                "x": float(x),
                "y": float(y),
                "rotation": float(rotation)
            }
            
            print(f"📤 Enviando: X={x:.2f} Y={y:.2f} R={rotation:.2f} por {duration}s")
            
            # Enviar comando
            response = requests.post(
                self.control_url,
                json=command,
                headers={'Content-Type': 'application/json'},
                timeout=3
            )
            
            if response.status_code == 200:
                print("✅ Comando enviado exitosamente")
                self.command_count += 1
                
                # Mantener movimiento por duración especificada
                time.sleep(duration)
                
                # Parar
                stop_command = {"x": 0, "y": 0, "rotation": 0}
                requests.post(self.control_url, json=stop_command, timeout=2)
                print("🛑 Motor detenido")
                
                return True
            else:
                print(f"❌ Error HTTP: {response.status_code}")
                return False
                
        except Exception as e:
            print(f"❌ Error enviando comando: {e}")
            return False

    def emergency_stop(self):
        """Parada de emergencia"""
        try:
            response = requests.get(self.stop_url, timeout=3)
            print("🛑 PARADA DE EMERGENCIA")
            self.robot_active = False
        except Exception as e:
            print(f"⚠️ Error en parada: {e}")

    def test_individual_motors(self):
        """Test individual de cada motor"""
        print("\n🔧 TEST INDIVIDUAL DE MOTORES")
        print("="*40)
        
        tests = [
            # (x, y, rotation, descripción)
            (0, 0.3, 0, "ADELANTE - Motor A principalmente"),
            (0, -0.3, 0, "ATRÁS - Motor A principalmente"),
            (0.3, 0, 0, "DERECHA - Motor B y C"),
            (-0.3, 0, 0, "IZQUIERDA - Motor B y C"),
            (0, 0, 0.3, "ROTAR DERECHA - Todos los motores"),
            (0, 0, -0.3, "ROTAR IZQUIERDA - Todos los motores"),
            (0.2, 0.2, 0, "DIAGONAL - Combinación"),
        ]
        
        for i, (x, y, rot, desc) in enumerate(tests):
            print(f"\n🎮 Test {i+1}/7: {desc}")
            input("   Presiona ENTER para continuar...")
            
            if self.send_motor_command(x, y, rot, 3.0):
                print("   ✅ Test completado")
            else:
                print("   ❌ Test falló")
            
            time.sleep(1)
        
        print("\n✅ TEST INDIVIDUAL COMPLETADO")

    def test_pattern_movement(self):
        """Test con patrón de movimiento automático"""
        print("\n🤖 TEST PATRÓN AUTOMÁTICO")
        print("="*40)
        
        patterns = [
            # Cuadrado
            [(0, 0.3, 0), (0.3, 0, 0), (0, -0.3, 0), (-0.3, 0, 0)],
            # Círculo (aproximado)
            [(0.2, 0.2, 0), (0.3, 0, 0), (0.2, -0.2, 0), (0, -0.3, 0), 
             (-0.2, -0.2, 0), (-0.3, 0, 0), (-0.2, 0.2, 0), (0, 0.3, 0)],
            # Zigzag
            [(0.3, 0.2, 0), (-0.3, 0.2, 0), (0.3, 0.2, 0), (-0.3, 0.2, 0)]
        ]
        
        pattern_names = ["CUADRADO", "CÍRCULO", "ZIGZAG"]
        
        for pattern_idx, pattern in enumerate(patterns):
            print(f"\n🔄 Patrón {pattern_idx+1}: {pattern_names[pattern_idx]}")
            input("   Presiona ENTER para iniciar patrón...")
            
            for step, (x, y, rot) in enumerate(pattern):
                print(f"   Paso {step+1}/{len(pattern)}")
                self.send_motor_command(x, y, rot, 2.0)
                time.sleep(0.5)
            
            print(f"   ✅ Patrón {pattern_names[pattern_idx]} completado")

    def test_camera_reactive(self):
        """Test reactivo basado en cambios en la cámara"""
        print("\n📹 TEST REACTIVO CON CÁMARA")
        print("="*40)
        print("El robot se moverá según cambios en la imagen")
        print("Mueve objetos frente a la cámara para ver reacción")
        
        previous_frame = None
        movement_threshold = 5000  # Umbral de cambio para reaccionar
        
        try:
            for i in range(50):  # 50 iteraciones de prueba
                # Capturar frame
                response = requests.get(self.capture_url, timeout=3)
                if response.status_code == 200:
                    img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        # Convertir a escala de grises
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        
                        if previous_frame is not None:
                            # Calcular diferencia
                            diff = cv2.absdiff(previous_frame, gray)
                            change_amount = np.sum(diff)
                            
                            print(f"📊 Iteración {i+1}/50 - Cambio: {change_amount}")
                            
                            # Reaccionar si hay suficiente cambio
                            if change_amount > movement_threshold:
                                print("🎯 ¡CAMBIO DETECTADO! - Moviendo robot")
                                
                                # Movimiento basado en la ubicación del cambio
                                # Calcular centro de masa del cambio
                                moments = cv2.moments(diff)
                                if moments["m00"] != 0:
                                    cx = int(moments["m10"] / moments["m00"])
                                    cy = int(moments["m01"] / moments["m00"])
                                    
                                    # Convertir posición a comando de movimiento
                                    center_x = frame.shape[1] // 2
                                    center_y = frame.shape[0] // 2
                                    
                                    # Cálculo simple de dirección
                                    x_command = (cx - center_x) / center_x * 0.3
                                    y_command = 0.2  # Siempre un poco hacia adelante
                                    
                                    self.send_motor_command(x_command, y_command, 0, 1.5)
                                else:
                                    # Movimiento aleatorio si no se puede calcular centro
                                    import random
                                    x_cmd = random.uniform(-0.3, 0.3)
                                    y_cmd = random.uniform(0.1, 0.3)
                                    self.send_motor_command(x_cmd, y_cmd, 0, 1.0)
                            
                        previous_frame = gray.copy()
                        
                        # Mostrar imagen si es posible
                        try:
                            cv2.imshow('Robot Vision Test', frame)
                            if cv2.waitKey(100) & 0xFF == ord('q'):
                                break
                        except:
                            pass
                
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n⚠️ Test interrumpido por usuario")
        except Exception as e:
            print(f"❌ Error en test reactivo: {e}")
        finally:
            try:
                cv2.destroyAllWindows()
            except:
                pass

    def run_interactive_test(self):
        """Menú interactivo de pruebas"""
        print("\n🎮 MENÚ DE PRUEBAS INTERACTIVO")
        print("="*50)
        
        while True:
            print("\n📋 OPCIONES DISPONIBLES:")
            print("1. 🔧 Test individual de motores")
            print("2. 🤖 Test patrón automático")
            print("3. 📹 Test reactivo con cámara")
            print("4. 🎮 Control manual simple")
            print("5. 🛑 Parada de emergencia")
            print("6. 📊 Ver estado del robot")
            print("0. ❌ Salir")
            
            try:
                opcion = input("\n🎯 Selecciona opción (0-6): ").strip()
                
                if opcion == "1":
                    self.test_individual_motors()
                elif opcion == "2":
                    self.test_pattern_movement()
                elif opcion == "3":
                    self.test_camera_reactive()
                elif opcion == "4":
                    self.manual_control()
                elif opcion == "5":
                    self.emergency_stop()
                elif opcion == "6":
                    self.show_robot_status()
                elif opcion == "0":
                    print("👋 ¡Hasta luego!")
                    break
                else:
                    print("⚠️ Opción no válida")
                    
            except KeyboardInterrupt:
                print("\n⚠️ Saliendo...")
                break
        
        # Parada final
        self.emergency_stop()

    def manual_control(self):
        """Control manual simple"""
        print("\n🎮 CONTROL MANUAL SIMPLE")
        print("="*30)
        print("Comandos disponibles:")
        print("w: Adelante    s: Atrás")
        print("a: Izquierda   d: Derecha") 
        print("q: Rotar izq   e: Rotar der")
        print("x: PARAR       0: Salir")
        
        while True:
            cmd = input("\n🎯 Comando: ").strip().lower()
            
            if cmd == 'w':
                self.send_motor_command(0, 0.3, 0, 1.5)
            elif cmd == 's':
                self.send_motor_command(0, -0.3, 0, 1.5)
            elif cmd == 'a':
                self.send_motor_command(-0.3, 0, 0, 1.5)
            elif cmd == 'd':
                self.send_motor_command(0.3, 0, 0, 1.5)
            elif cmd == 'q':
                self.send_motor_command(0, 0, -0.3, 1.5)
            elif cmd == 'e':
                self.send_motor_command(0, 0, 0.3, 1.5)
            elif cmd == 'x':
                self.emergency_stop()
            elif cmd == '0':
                break
            else:
                print("⚠️ Comando no reconocido")

    def show_robot_status(self):
        """Mostrar estado actual del robot"""
        try:
            response = requests.get(self.status_url, timeout=3)
            if response.status_code == 200:
                status = response.json()
                print("\n📊 ESTADO ACTUAL DEL ROBOT:")
                print("="*40)
                for key, value in status.items():
                    print(f"  {key}: {value}")
            else:
                print(f"❌ Error obteniendo status: {response.status_code}")
        except Exception as e:
            print(f"❌ Error: {e}")

def main():
    print("🤖 ROBOT TEST DE MOVIMIENTO")
    print("Probando conexión y movimiento con ESP32-CAM")
    
    # IP CORREGIDA automáticamente detectada
    ESP32_IP = "192.168.182.182"
    
    try:
        robot_test = TestMovimientoRobot(ESP32_IP)
        robot_test.run_interactive_test()
    except KeyboardInterrupt:
        print("\n⚠️ Programa interrumpido")
    except Exception as e:
        print(f"❌ Error crítico: {e}")

if __name__ == "__main__":
    main()