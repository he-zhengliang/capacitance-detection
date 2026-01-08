import tkinter as tk
from tkinter import messagebox
import serial
import time
import csv
import datetime
import threading
import sys
import os
import numpy as np

# === CONFIG ===
ARDUINO_PORT = '/dev/ttyACM1'
ARDUINO_BAUD = 250000
GRIPPER_PORT = '/dev/ttyACM0'
GRIPPER_BAUD = 9600

LOG_DURATION = 4.0  # seconds
CHANNELS = 12       # Number of capacitance channels

LABEL_MAP = {'g': 1, 'b': 0, 'n': 0}   # numeric label mapping
current_label = ['n']  # default label

# === Serial ===
def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[INFO] Connected to {port} at {baud}")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port}: {e}")
        return None

def send_hex(ser, hex_command):
    try:
        hex_command = hex_command.replace(" ", "")
        bytes_cmd = bytes.fromhex(hex_command)
        ser.write(bytes_cmd)
        time.sleep(0.1)
    except Exception as e:
        print(f"[ERROR] Sending command failed: {e}")

# === Gripper Control ===
def control_gripper(gripper_ser):
    print("[INFO] Starting gripper sequence")
    send_hex(gripper_ser, 'AA 55 00 01 01 01 01 05 FF 03 FF 01 2F 02 2A')  # Open hand
    time.sleep(2)
    send_hex(gripper_ser, 'AA 55 01 00 00 00 00 08 FF 05 96 00 90 01 F4')  # Close thumb
    time.sleep(1)
    send_hex(gripper_ser, 'AA 55 00 01 01 01 01 08 FF 02 58 01 20 01 8D')  # Start grasp
    time.sleep(LOG_DURATION)
    send_hex(gripper_ser, 'AA 55 00 01 01 01 01 05 FF 03 FF 01 2F 02 2A')  # Open again
    print("[INFO] Gripper sequence completed")

# === Data Logging (Threaded with new trigger) ===
def log_data(arduino_ser, label, baseline_event, finished_event):
    numeric_label = LABEL_MAP[label]
    date_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = "good" if numeric_label == 1 else "fail"
    filename = f"{folder}/{numeric_label}_{date_str}.csv"

    os.makedirs(folder, exist_ok=True)

    baseline_window = 20
    WINDOW_SIZE = 64

    baseline_samples = []
    triggered = False
    window_buffer = []

    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["t"] + [f"c{i+1}" for i in range(CHANNELS)])

        print("[INFO] Waiting for incoming data...")

        while True:
            raw = arduino_ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode(errors='ignore').strip()
                if "," not in line:
                    continue
                values = line.split(",")
                floats = list(map(float, values[1:1 + CHANNELS]))
            except:
                continue

            # ============ Baseline Collection (Delay Only) ============
            if len(baseline_samples) < baseline_window:
                baseline_samples.append(floats)
                if len(baseline_samples) == baseline_window:
                    print("[INFO] Baseline samples collected (trigger: 3 channels > 1)")
                    baseline_event.set()
                continue

            # ============ NEW TRIGGER: instant activation when 3+ channels > 1 ============
            num_over = sum(f > 1.0 for f in floats)

            if not triggered and num_over >= 3:
                triggered = True
                print("[INFO] Trigger reached → capturing 64 samples")
                window_buffer = []
                continue

            # ============ Collect 64 Samples ============
            if triggered:
                window_buffer.append(floats)
                if len(window_buffer) >= WINDOW_SIZE:
                    print("[INFO] 64-sample window collected")
                    break

        # ============ Save the Window ============
        for idx, sample in enumerate(window_buffer):
            writer.writerow([idx] + sample)

    print(f"[INFO] Finished writing: {filename}")
    finished_event.set()

# === GUI Logic ===
def set_label(l):
    current_label[0] = l
    label_status.config(text=f"Current Label: {l.upper()}", fg="green")

def run_trial():
    label = current_label[0]
    status_label.config(text="Running Trial...", fg="blue")
    trial_btn.config(state='disabled')

    baseline_event = threading.Event()
    finished_event = threading.Event()

    def task():
        try:
            # Start log_data thread
            log_thread = threading.Thread(target=log_data, args=(arduino_ser, label, baseline_event, finished_event))
            log_thread.start()

            # Wait for baseline samples
            baseline_event.wait()

            # Run gripper motion
            control_gripper(gripper_ser)

            # Wait until logging is done
            finished_event.wait()

            status_label.config(text=f"Saved with label '{label.upper()}'", fg="black")
        except Exception as e:
            messagebox.showerror("Error", str(e))
            status_label.config(text="Trial failed", fg="red")
        finally:
            trial_btn.config(state='normal')

    threading.Thread(target=task, daemon=True).start()

# === MAIN ===
if __name__ == "__main__":

    gripper_ser = open_serial(GRIPPER_PORT, GRIPPER_BAUD)
    arduino_ser = open_serial(ARDUINO_PORT, ARDUINO_BAUD)

    if not gripper_ser or not arduino_ser:
        print("[FATAL] Could not open serial ports.")
        sys.exit(1)

    root = tk.Tk()
    root.title("Grasp Logger")

    # Window layout
    root.update_idletasks()
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    width = 800
    height = 400
    x = (screen_width // 2) - (width // 2)
    y = (screen_height // 2) - (height // 2)
    root.geometry(f"{width}x{height}+{x}+{y}")

    label_frame = tk.LabelFrame(root, text="Select Grasp Label", font=("Helvetica", 14))
    label_frame.pack(padx=20, pady=10, fill="x")

    tk.Button(label_frame, text="Good", font=("Helvetica", 14), command=lambda: set_label('g')).pack(side="left", expand=True, fill="x", padx=10, pady=5)
    tk.Button(label_frame, text="Bad", font=("Helvetica", 14), command=lambda: set_label('b')).pack(side="left", expand=True, fill="x", padx=10, pady=5)
    tk.Button(label_frame, text="Null", font=("Helvetica", 14), command=lambda: set_label('n')).pack(side="left", expand=True, fill="x", padx=10, pady=5)

    label_status = tk.Label(root, text="Current Label: N", fg="green", font=("Helvetica", 14))
    label_status.pack(pady=10)

    trial_btn = tk.Button(root, text="Start Trial", font=("Helvetica", 18), command=run_trial)
    trial_btn.pack(pady=10)

    status_label = tk.Label(root, text="Ready", fg="black", font=("Helvetica", 14))
    status_label.pack(pady=10)

    root.protocol("WM_DELETE_WINDOW", root.quit)
    root.mainloop()

    gripper_ser.close()
    arduino_ser.close()
    print("[INFO] Exiting.")
