from ultralytics import YOLO
from PIL import Image, ImageDraw, ImageFont
import requests
from io import BytesIO
import paho.mqtt.client as mqtt
import base64
import time
from datetime import datetime
import os

# Załaduj model YOLOv8
model = YOLO("yolov8n.pt")

# Adres ESP32-CAM
url = "http://192.168.239.193/capture"

# MQTT konfiguracja
mqtt_broker = "test.mosquitto.org"
mqtt_port = 1883

# Folder na zdjęcia
os.makedirs("photos", exist_ok=True)

# Czcionka
try:
    font = ImageFont.truetype("arial.ttf", 16)
except:
    font = ImageFont.load_default()

while True:
    try:
        client = mqtt.Client()
        client.connect(mqtt_broker, mqtt_port, 60)
        
        response = requests.get(url, timeout=2)
        img = Image.open(BytesIO(response.content)).convert("RGB")
        img_bytes = response.content

        # Wykrycie obiektów
        results = model(img)
        boxes = results[0].boxes

        found_person = False
        draw = ImageDraw.Draw(img)

        for box in boxes:
            cls = int(box.cls)
            conf = float(box.conf)

            if cls == 0:  # Osoba
                found_person = True
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = f"Person {conf*100:.1f}%"
                draw.rectangle([(x1, y1), (x2, y2)], outline="red", width=3)
                text_position = (x1, max(y1 - 20, 0))  # zapobiega rysowaniu poza obrazem
                draw.text(text_position, label, fill="red", font=font)


        # Zakoduj zdjęcie z ramkami jako JPEG
        buf = BytesIO()
        img.save(buf, format="JPEG")
        annotated_img_bytes = buf.getvalue()
        img_b64 = base64.b64encode(annotated_img_bytes).decode('utf-8')

        if found_person:
            print("🧍 Człowiek wykryty!")
            client.publish("kandefer/camera/img_human", img_b64)
            client.publish("kandefer/camera/human_detected", "1")

            # Zapisz na dysku
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"photos/person_{timestamp}.jpg"
            with open(filename, "wb") as f:
                f.write(annotated_img_bytes)
            print(f"📸 Zapisano z ramką: {filename}")
        else:
            print("❌ Brak człowieka.")
            client.publish("kandefer/camera/human_detected", "0")

        # Wysłanie obrazu kontrolnego
        client.publish("kandefer/camera/img_control", img_b64)
        time.sleep(3)
        client.disconnect()

    except Exception as e:
        print("Błąd:", e)
