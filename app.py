from flask import Flask, render_template, jsonify, send_file, Response
import cv2
import numpy as np
import requests
import json
import time
import threading
import datetime
from fpdf import FPDF
from ultralytics import YOLO  # Updated for YOLO11 support

# ---------------- FLASK ----------------
app = Flask(__name__)

# ---------------- ESP32 ----------------
ESP32_IP = "192.168.1.120"   # CHANGE THIS
CAPTURE_URL = f"http://192.168.86.15/capture"
CONTROL_URL = f"http://192.168.86.15/control?bin="

# ---------------- AI MODEL (YOLO11 Small) -----
# Upgraded to 'Small' model for significantly higher accuracy than 'Nano'
model = YOLO("yolo11s.pt") 

def preprocess_low_light(image):
    """Deeply enhance visibility and sharpness specifically for small objects"""
    # 1. Contrast Enhancement (CLAHE)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(4,4)) # Smaller grid for finer detail
    cl = clahe.apply(l)
    limg = cv2.merge((cl, a, b))
    enhanced = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    
    # 2. Strong Sharpening for Fine Details
    # Using a stronger Laplacian-based kernel
    kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    enhanced = cv2.filter2D(enhanced, -1, kernel)
    
    # 3. Targeted Noise Reduction
    # Light denoising so we don't blur the small objects
    enhanced = cv2.fastNlMeansDenoisingColored(enhanced, None, 5, 5, 7, 21)
        
    return enhanced

# ---------------- DATA FILE ------------
DATA_FILE = "waste_data.json"


# Initial data (PDF aligned)
def init_data():
    return {
        "plastic": 0,
        "medical": 0,
        "general": 0,
        "last_detected": "None",
        "total": 0,
        "esp32_status": "OFFLINE",
        "dominant": "None",
        "recommendation": "System initializing...",
        "history": []
    }


# Load existing data if available
try:
    with open(DATA_FILE, "r") as f:
        data = json.load(f)
except Exception:
    data = init_data()


# ---------------- WEBSITE ROUTES -------
@app.route("/")
def dashboard():
    return render_template("dashboard.html")


@app.route("/data")
def data_api():
    return jsonify(data)


@app.route("/reset", methods=["POST"])
def reset_data():
    global data
    data = init_data()
    with open(DATA_FILE, "w") as f:
        json.dump(data, f)
    return jsonify({"status": "success", "message": "Data reset successfully"})


@app.route("/report")
def download_report():
    pdf = FPDF()
    pdf.add_page()
    
    # Title
    pdf.set_font("Arial", 'B', 16)
    pdf.cell(200, 10, txt="Smart Garbage Segregation Report", ln=1, align="C")
    
    # Timestamp
    pdf.set_font("Arial", size=10)
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    pdf.cell(200, 10, txt=f"Generated on: {timestamp}", ln=1, align="C")
    pdf.ln(10)
    
    # Statistics
    pdf.set_font("Arial", 'B', 12)
    pdf.cell(200, 10, txt="Waste Statistics:", ln=1)
    
    pdf.set_font("Arial", size=12)
    pdf.cell(0, 10, txt=f"Total Waste Detected: {data['total']}", ln=1)
    pdf.cell(0, 10, txt=f"Plastic Waste: {data['plastic']}", ln=1)
    pdf.cell(0, 10, txt=f"Medical Waste: {data['medical']}", ln=1)
    pdf.cell(0, 10, txt=f"General Waste: {data['general']}", ln=1)
    pdf.ln(5)
    
    # Analysis
    pdf.set_font("Arial", 'B', 12)
    pdf.cell(200, 10, txt="Analysis & Recommendation:", ln=1)
    
    pdf.set_font("Arial", size=12)
    pdf.cell(0, 10, txt=f"Dominant Waste Type: {data['dominant']}", ln=1)
    
    # Multi-line cell for recommendation to avoid visual overflow
    pdf.multi_cell(0, 10, txt=f"Suggestion: {data.get('recommendation', 'N/A')}")
    
    # Save temporarily
    filename = "Waste_Report.pdf"
    pdf.output(filename)
    
    return send_file(filename, as_attachment=False)


# ---------------- STREAMING & AI -------
latest_frame = None
last_detected_category = "none"
frame_lock = threading.Lock()

def camera_thread():
    global latest_frame, last_detected_category
    print("📸 YOLO11 Low-Light Optimized Thread Started")
    
    while True:
        try:
            # 1. Fetch raw frame from ESP32
            response = requests.get(CAPTURE_URL, timeout=2)
            if response.status_code != 200: continue
                
            raw_frame = cv2.imdecode(np.frombuffer(response.content, np.uint8), cv2.IMREAD_COLOR)
            
            # 2. Low Light Enhancement
            enhanced_frame = preprocess_low_light(raw_frame)
            frame = enhanced_frame.copy()
            
            # --- DETECTION ZONE & DIGITAL ZOOM ---
            # We crop the zone and resize it to make small objects (pills) "Big" for the AI
            fh, fw, _ = enhanced_frame.shape
            zone_size = 360
            zx1, zy1 = (fw - zone_size) // 2, (fh - zone_size) // 2
            zx2, zy2 = zx1 + zone_size, zy1 + zone_size
            
            # "Zoom" into the detection zone
            zoom_crop = enhanced_frame[zy1:zy2, zx1:zx2]
            zoom_input = cv2.resize(zoom_crop, (640, 640)) # Blow up the small box to full AI resolution
            
            cv2.rectangle(frame, (zx1, zy1), (zx2, zy2), (0, 0, 255), 2)
            cv2.putText(frame, "ZOOMED DETECTION ZONE", (zx1, zy1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 3. Run YOLO11 Detection on the ZOOMED Frame
            # This makes a small pill look like a large object to the AI
            results = model(zoom_input, conf=0.4, verbose=False)
            frame_detected = "none"
            
            # --- CATEGORY MAPPING ---
            PLASTIC = ["bottle", "cup", "bowl", "fork", "knife", "spoon", "frisbee"]
            MEDICAL = ["scissors", "knife", "syringe", "pill", "spray", "bandage", "gauze"]
            GENERAL = ["apple", "banana", "orange", "sandwich", "book", "vase", "notebook", 
                       "remote", "laptop", "cell phone", "tv", "monitor"]
            IGNORE  = ["person", "chair", "dining table", "couch", "bed", "toilet", "traffic light", 
                       "train", "bus", "truck", "airplane", "boat", "bird"]

            detected_objects_in_zone = []

            for r in results:
                for box in r.boxes:
                    # Note: Since we are using the Zoom frame, everything detected IS inside the zone
                    label = model.names[int(box.cls[0])]
                    if label in IGNORE: continue
                    
                    # Draw in the main view (map coords back)
                    x1_z, y1_z, x2_z, y2_z = map(int, box.xyxy[0])
                    # Scale back to original zone coordinates
                    scale = zone_size / 640.0
                    rx1 = zx1 + int(x1_z * scale)
                    ry1 = zy1 + int(y1_z * scale)
                    rx2 = zx1 + int(x2_z * scale)
                    ry2 = zy1 + int(y2_z * scale)

                    cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label}", (rx1, ry1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    print(f"🎯 YOLO11 Match: {label}")
                    if label in PLASTIC: detected_objects_in_zone.append("plastic")
                    elif label in MEDICAL: detected_objects_in_zone.append("medical")
                    else: detected_objects_in_zone.append("general")

            # Update voting buffer for stability (Strict: 4 matches in last 7 frames)
            if not hasattr(camera_thread, "buffer"): camera_thread.buffer = []
            
            current_frame_choice = detected_objects_in_zone[0] if detected_objects_in_zone else "none"
            camera_thread.buffer.append(current_frame_choice)
            if len(camera_thread.buffer) > 7: camera_thread.buffer.pop(0)
            
            # Require 4/7 frames to trigger (Much more reliable)
            if len(camera_thread.buffer) >= 4:
                from collections import Counter
                most_common = Counter(camera_thread.buffer).most_common(1)[0]
                if most_common[0] != "none" and most_common[1] >= 4: 
                    last_detected_category = most_common[0]
                else:
                    last_detected_category = "none"
            else:
                last_detected_category = "none"
            
            # Encode frame
            _, buffer = cv2.imencode('.jpg', frame)
            with frame_lock:
                latest_frame = buffer.tobytes()
            
            data["esp32_status"] = "ONLINE"
            
        except Exception as e:
            data["esp32_status"] = "OFFLINE"
            # print(f"Camera Error: {e}")
            time.sleep(1)

def generate_frames():
    while True:
        with frame_lock:
            if latest_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        time.sleep(0.05) # ~20 FPS cap for the generator

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


# ---------------- SEGREGATION LOOP -----
def ai_loop():
    global data, last_detected_category 
    print("⚙️ Segregation Logic Loop Started")

    while True:
        try:
            if data["esp32_status"] == "OFFLINE":
                time.sleep(2)
                continue

            # Use the latest category detected by the camera thread
            detected = last_detected_category

            if detected == "none":
                time.sleep(1)
                continue

            # Update counts
            data[detected] += 1
            data["total"] += 1
            data["last_detected"] = detected

            # ---- Trend History ----
            timestamp = datetime.datetime.now().strftime("%H:%M:%S")
            if "history" not in data: 
                data["history"] = []
            
            data["history"].append({
                "time": timestamp,
                "plastic": data["plastic"],
                "medical": data["medical"],
                "general": data["general"]
            })
            
            if len(data["history"]) > 20:
                data["history"].pop(0)

            # ---- Analytics: Dominant Waste ----
            max_val = max(data["plastic"], data["medical"], data["general"])

            if max_val == 0:
                data["dominant"] = "None"
                data["recommendation"] = "Waiting for data..."
            elif max_val == data["plastic"]:
                data["dominant"] = "Plastic Waste"
                data["recommendation"] = "Critical: High plastic levels! Switch to reusable items immediately."
            elif max_val == data["medical"]:
                data["dominant"] = "Medical Waste"
                data["recommendation"] = "Alert: Medical waste spike. Review clinical disposal guidelines."
            else:
                data["dominant"] = "General Waste"
                data["recommendation"] = "Tip: Compost organic waste to reduce general landfill contribution."

            # Save data
            with open(DATA_FILE, "w") as f:
                json.dump(data, f)

            # Send command to ESP32 (Fire and Forget to avoid UI block)
            try:
                requests.get(CONTROL_URL + detected, timeout=1) 
            except Exception:
                # If it times out, the ESP32 is likely moving the servo already
                pass

            # Cooldown to allow servo to finish and object to be cleared
            time.sleep(4)
            last_detected_category = "none" # Reset to avoid double counting same object
            camera_thread.buffer = [] # Clear buffer after a successful segregation

        except Exception as e:
            print("⚠ Logic Error:", e)
            time.sleep(2)


# ---------------- MAIN -----------------
if __name__ == "__main__":
    # Start both threads
    threading.Thread(target=camera_thread, daemon=True).start()
    threading.Thread(target=ai_loop, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)

