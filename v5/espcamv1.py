import tkinter as tk
from PIL import Image, ImageTk
import requests
import io
import numpy as np
import cv2

class LineFollowerUI:
    def __init__(self, root, esp_ip):
        self.root = root
        self.esp_ip = esp_ip
        self.root.title("Seguimiento de Línea - Robot Omnidireccional")

        # Configurar tamaños
        self.img_width = 320
        self.img_height = 240

        # Crear frames
        self.frame_original = tk.Frame(root)
        self.frame_original.grid(row=0, column=0, padx=10, pady=10)

        self.frame_threshold = tk.Frame(root)
        self.frame_threshold.grid(row=0, column=1, padx=10, pady=10)

        self.frame_contour = tk.Frame(root)
        self.frame_contour.grid(row=0, column=2, padx=10, pady=10)

        # Etiquetas para imágenes
        self.label_original = tk.Label(self.frame_original)
        self.label_original.pack()
        tk.Label(self.frame_original, text="Imagen Original", font=("Arial", 14)).pack()

        self.label_threshold = tk.Label(self.frame_threshold)
        self.label_threshold.pack()
        tk.Label(self.frame_threshold, text="Imagen Umbralizada", font=("Arial", 14)).pack()

        self.label_contour = tk.Label(self.frame_contour)
        self.label_contour.pack()
        tk.Label(self.frame_contour, text="Contornos Detectados", font=("Arial", 14)).pack()

        # Controles
        self.control_frame = tk.Frame(root)
        self.control_frame.grid(row=1, column=0, columnspan=3, pady=20)

        self.btn_auto = tk.Button(self.control_frame, text="ACTIVAR IA", command=self.toggle_auto, width=15, height=2)
        self.btn_auto.pack(side=tk.LEFT, padx=10)

        self.btn_stop = tk.Button(self.control_frame, text="DETENER", command=self.stop_robot, width=15, height=2)
        self.btn_stop.pack(side=tk.LEFT, padx=10)

        # URLs de los streams
        self.url_original = f"http://{self.esp_ip}/stream_original"
        self.url_threshold = f"http://{self.esp_ip}/stream_threshold"
        self.url_contour = f"http://{self.esp_ip}/stream_contour"

        # Iniciar actualización
        self.update_images()

    def get_image_from_url(self, url):
        try:
            response = requests.get(url, timeout=2.0)
            if response.status_code == 200:
                return Image.open(io.BytesIO(response.content))
        except Exception as e:
            print(f"Error obteniendo imagen: {e}")
        return None

    def update_images(self):
        # Obtener imágenes
        img_original = self.get_image_from_url(self.url_original)
        img_threshold = self.get_image_from_url(self.url_threshold)
        img_contour = self.get_image_from_url(self.url_contour)

        # Actualizar GUI
        if img_original:
            img_original = img_original.resize((self.img_width, self.img_height))
            tk_img_original = ImageTk.PhotoImage(img_original)
            self.label_original.configure(image=tk_img_original)
            self.label_original.image = tk_img_original

        if img_threshold:
            img_threshold = img_threshold.resize((self.img_width, self.img_height))
            tk_img_threshold = ImageTk.PhotoImage(img_threshold)
            self.label_threshold.configure(image=tk_img_threshold)
            self.label_threshold.image = tk_img_threshold

        if img_contour:
            img_contour = img_contour.resize((self.img_width, self.img_height))
            tk_img_contour = ImageTk.PhotoImage(img_contour)
            self.label_contour.configure(image=tk_img_contour)
            self.label_contour.image = tk_img_contour

        # Programar próxima actualización
        self.root.after(200, self.update_images)

    def toggle_auto(self):
        try:
            current_text = self.btn_auto.cget("text")
            if current_text == "ACTIVAR IA":
                requests.post(f"http://{self.esp_ip}/auto", json={"auto": True}, timeout=1)
                self.btn_auto.config(text="DESACTIVAR IA", bg="red")
            else:
                requests.post(f"http://{self.esp_ip}/auto", json={"auto": False}, timeout=1)
                self.btn_auto.config(text="ACTIVAR IA", bg="SystemButtonFace")
        except:
            print("Error comunicándose con el robot")

    def stop_robot(self):
        try:
            requests.post(f"http://{self.esp_ip}/stop", timeout=1)
        except:
            print("Error comunicándose con el robot")

if __name__ == "__main__":
    # Reemplaza con la IP real de tu ESP32
    ESP_IP = "192.168.3.149"  # Asegúrate de actualizar esta IP

    root = tk.Tk()
    app = LineFollowerUI(root, ESP_IP)
    root.mainloop()