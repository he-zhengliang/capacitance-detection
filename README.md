# capacitance-detection
capacitive sensor robot hand grasping direction

import serial
import time
import threading
import sys

# ---------------- CONFIG ----------------
ARDUINO_PORT = '/dev/ttyACM1'  # Arduino for readings
ARDUINO_BAUD = 250000

GRIPPER_PORT = '/dev/ttyACM0'  # Gripper controller
GRIPPER_BAUD = 9600
# ----------------------------------------

# ---------------- Gripper Functions ----------------
def send_hex_command(ser, hex_command):
    """Send a hex command to the gripper"""
    hex_command = hex_command.replace(" ", "")
    hex_bytes = bytes.fromhex(hex_command)
    ser.write(hex_bytes)
    time.sleep(0.1)

def gripper_sequence(gripper_ser):
    """Control the gripper: open, close thumb, close fingers, open"""
    try:
        print("[INFO] Starting gripper sequence...")

        # Step 1: Open hand (optional)
        # send_hex_command(gripper_ser, 'AA 55 01 00 00 00 00 05 FF 03 FF 00 9F 02 9A')
        # send_hex_command(gripper_ser, 'AA 55 00 01 01 01 01 05 FF 03 FF 01 2F 02 2A')
        time.sleep(2)

        # Step 2: Close thumb
        send_hex_command(gripper_ser, 'AA 55 01 00 00 00 00 08 FF 05 96 00 90 01 F4')
        time.sleep(1)

        # Step 3: Adaptive grasp control algorithm
        # step 1 -> Read sensor data
        # step 2 -> Interprate sensing information:
                                            # Simple logic: if si not 100: increase or reduce the angle of that finger
                                            # AI classification logic based on the sensors' information
                                            # Print the grasping class
        # step 3 -> (Optional) Implement PID controller for position and speed control - carefully adjustement fingers as we are already in contact                                   
        # index finger
        # send_hex_command(gripper_ser, 'AA 55 00 01 00 00 00 08 FF 02 58 01 20 01 8D')
        # Middle finger
        # send_hex_command(gripper_ser, 'AA 55 00 00 01 00 00 08 FF 02 58 01 20 01 8D')
        # Ring finger
        # send_hex_command(gripper_ser, 'AA 55 00 00 00 01 00 08 FF 02 58 01 20 01 8D')
        # step 3 -> Little finger (completing the grasp)
        # send_hex_command(gripper_ser, 'AA 55 00 00 00 01 00 08 FF 02 58 01 20 01 8D')
        
        # step 3 -> Close finger (completing the grasp)
        send_hex_command(gripper_ser, 'AA 55 00 01 01 01 01 08 FF 02 58 01 20 01 8D')
        time.sleep(3)

        # Step 4: Open hand to release
        send_hex_command(gripper_ser, 'AA 55 00 01 01 01 01 05 FF 03 FF 01 2F 02 2A')

        print("[INFO] Gripper sequence completed.")

    except Exception as e:
        print("[ERROR] Gripper sequence failed:", e)

# ---------------- Arduino Reading ----------------
def read_arduino(arduino_ser):
    """Continuously read and print Arduino data"""
    try:
        while True:
            line = arduino_ser.readline().decode(errors='ignore').strip()
            if line:
                print(line)
    except Exception as e:
        print("[ERROR] Arduino reading failed:", e)

# ---------------- MAIN ----------------
def main():
    # Open serial connections
    try:
        print("[INFO] Connecting to gripper...")
        gripper_ser = serial.Serial(GRIPPER_PORT, GRIPPER_BAUD, timeout=1)
        print(f"[INFO] Gripper connected on {GRIPPER_PORT}")

        print("[INFO] Connecting to Arduino...")
        arduino_ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        print(f"[INFO] Arduino connected on {ARDUINO_PORT}")

    except serial.SerialException as e:
        print("[ERROR] Could not open serial ports:", e)
        sys.exit(1)

    # Start Arduino reading in a separate thread
    arduino_thread = threading.Thread(target=read_arduino, args=(arduino_ser,), daemon=True)
    arduino_thread.start()

    # Run gripper sequence (blocking)
    gripper_sequence(gripper_ser)

    # Keep main thread alive while Arduino prints
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")

    # Cleanup
    if gripper_ser.is_open:
        gripper_ser.close()
    if arduino_ser.is_open:
        arduino_ser.close()
    print("[INFO] Serial connections closed.")

if __name__ == "__main__":
    main()
