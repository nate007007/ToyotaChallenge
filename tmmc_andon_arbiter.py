import serial
import serial.tools.list_ports
import time
import socket
from plyer import notification

# ==========================================
# ⚙️ CONFIGURATION
# ==========================================
BAUD_RATE = 300 
FLEET_PORT = 8080             
ROBOT_A_IP = '10.37.69.255'  
ROBOT_B_IP = '192.168.1.101'  

def find_arduino():
    for port in serial.tools.list_ports.comports():
        if 'Arduino' in port.description or 'USB Serial' in port.description:
            return port.device
    return None

def send_network_command(target_ip, command):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect((target_ip, FLEET_PORT))
        s.sendall(command.encode('utf-8'))
        s.close()
    except: pass

print("TMMC Arbiter Active. Waiting for hardware...")

while True:
    port = find_arduino()
    if not port:
        time.sleep(1)
        continue
        
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        print(f"Connected to {port}. Monitoring for sustained stall...")
        
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if "STALL_DETECTED" in data:
                    print("🚨 SUSTAINED STALL DETECTED - DISPATCHING...")
                    try:
                        notification.notify(title="🚨 TMMC Alert", message="Motor Failure.", timeout=1)
                    except: pass
                    send_network_command(ROBOT_A_IP, "S")
                    send_network_command(ROBOT_B_IP, "RESCUE")
                    time.sleep(2) 
            
    except (serial.SerialException, OSError):
        # Hardware reset occurred. Silent recovery.
        time.sleep(0.5) 
        continue