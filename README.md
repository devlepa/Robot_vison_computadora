# Robot con Visión Computacional

Este proyecto implementa un robot móvil controlado mediante visión artificial usando una cámara ESP32-CAM y motores controlados por código en C. El sistema tiene como objetivo permitir la movilidad del robot en respuesta a estímulos visuales, siendo útil para investigaciones, educación o desarrollos de robótica autónoma.

## 📁 Estructura del Proyecto

El repositorio contiene varias versiones del sistema, cada una con tres archivos principales:

```
version_final copy 2/
├── codigo_motores_.c               # Control básico de motores
├── codigo_motores_serviodor.c     # Versión con control remoto o servidor
└── espcamv1.py                     # Procesamiento de imágenes y visión artificial
```

Las carpetas `version_final copy` y `vfinal copy 3` contienen variantes de la versión final con ligeras modificaciones para pruebas o mejoras.

## ⚙️ Tecnologías Usadas

- **ESP32-CAM**: Módulo de cámara para capturar imágenes.
- **Lenguaje C**: Para el control de motores en microcontroladores.
- **Python (OpenCV)**: Para la visión artificial en la ESP32-CAM.
- **Motores controlados con puente H (como DRV8833 o L298N)**

## 🚀 Cómo Usar

1. **Subir el código C a la placa**:
   - Utiliza un entorno como Arduino IDE o PlatformIO.
   - Sube `codigo_motores_.c` o `codigo_motores_serviodor.c` a tu microcontrolador.

2. **Ejecutar el script de visión**:
   - Asegúrate de tener Python 3.7+ instalado.
   - Instala dependencias: `pip install opencv-python`
   - Ejecuta el archivo `espcamv1.py`.

3. **Conexión**:
   - Conecta la ESP32-CAM a la red Wi-Fi.
   - Asegúrate de que el script en Python pueda acceder al feed de la cámara.

## 🧪 Notas

- El código en las carpetas `copy` parece ser experimental. Se recomienda utilizar la versión `version_final copy 2/` como principal.
- Verifica los pines conectados en tu circuito para que coincidan con el código C.

## 📷 Funcionalidad de Visión

El archivo `espcamv1.py` incluye funciones para:

- Capturar imágenes en tiempo real desde la ESP32-CAM.
- Procesar la imagen con OpenCV.
- Tomar decisiones (ej. mover motores) en función de lo que "ve" el robot.

## 📌 Requisitos

- Python 3.7 o superior
- OpenCV (`pip install opencv-python`)
- ESP32-CAM configurada
- Microcontrolador con soporte para C

## 📄 Licencia

Este proyecto está disponible bajo una licencia abierta. Siéntete libre de modificarlo y utilizarlo en tus propios proyectos educativos o personales.
