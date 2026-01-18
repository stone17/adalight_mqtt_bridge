import paho.mqtt.client as mqtt
import serial
import json
import time
import os
import sys

# --- CONFIGURATION ---
MQTT_BROKER = os.getenv('MQTT_BROKER', '192.168.1.100')
MQTT_PORT = int(os.getenv('MQTT_PORT', 1883))
MQTT_USER = os.getenv('MQTT_USER', None)
MQTT_PASS = os.getenv('MQTT_PASS', None)

SERIAL_PORT = os.getenv('SERIAL_PORT', '/dev/ttyUSB0')
BAUD_RATE = int(os.getenv('BAUD_RATE', 115200))
LED_COUNT = int(os.getenv('LED_COUNT', 35))
DEVICE_NAME = os.getenv('DEVICE_NAME', 'Hyperion Lite')
UNIQUE_ID = os.getenv('UNIQUE_ID', f'mqtt_led_{LED_COUNT}')
STATE_FILE = '/data/state.json'

# Defaults (Fallbacks if no file found)
DEFAULT_HEX = os.getenv('DEFAULT_COLOR', 'FFAA55')
DEFAULT_BRT = int(os.getenv('DEFAULT_BRIGHTNESS', 80))

# --- GLOBAL STATE ---
ser = None
last_heartbeat = 0

def hex_to_rgb(hex_str):
    hex_str = hex_str.lstrip('#')
    try:
        return tuple(int(hex_str[i:i+2], 16) for i in (0, 2, 4))
    except ValueError:
        return (255, 255, 255)

# --- PERSISTENCE LOGIC ---
def save_state():
    try:
        with open(STATE_FILE, 'w') as f:
            json.dump(current_state, f)
    except Exception as e:
        print(f"[ERROR] Could not save state: {e}")

def load_state():
    # 1. Start with hardcoded defaults
    start_r, start_g, start_b = hex_to_rgb(DEFAULT_HEX)
    state = {
        "state": "OFF",  # Always boot OFF
        "brightness": DEFAULT_BRT,
        "color": {"r": start_r, "g": start_g, "b": start_b}
    }
    
    # 2. Try to overwrite with saved file
    if os.path.exists(STATE_FILE):
        try:
            print("[INIT] Found saved state file. Loading...")
            with open(STATE_FILE, 'r') as f:
                saved = json.load(f)
                # Restore color/brightness, but FORCE state to OFF
                if 'brightness' in saved: state['brightness'] = saved['brightness']
                if 'color' in saved: state['color'] = saved['color']
                state['state'] = "OFF" # <--- Force Lights Off on Boot
        except Exception as e:
            print(f"[ERROR] Corrupt state file, using defaults: {e}")
            
    return state

# Initialize immediately
current_state = load_state()

# --- SERIAL SETUP ---
def setup_serial():
    global ser
    try:
        print(f"[INIT] Opening Serial Port {SERIAL_PORT} at {BAUD_RATE}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("[INIT] Waiting 3 seconds for bootloader...")
        time.sleep(3)
        print("[INIT] Serial Ready.")
    except Exception as e:
        print(f"[FATAL] Could not open serial port: {e}")
        sys.exit(1)

# --- PHYSICS ENGINE ---
def send_adalight(force_log=False):
    global ser
    
    brightness_factor = current_state['brightness'] / 255.0
    
    target_r = current_state['color']['r']
    target_g = current_state['color']['g']
    target_b = current_state['color']['b']

    final_r = int(target_r * brightness_factor)
    final_g = int(target_g * brightness_factor)
    final_b = int(target_b * brightness_factor)

    # If OFF, send Black
    if current_state['state'] == "OFF":
        final_r, final_g, final_b = 0, 0, 0

    if force_log:
        print(f"State: {current_state['state']} | RGB: ({target_r},{target_g},{target_b}) | Brt: {current_state['brightness']}")

    # Adalight Protocol
    count = LED_COUNT - 1
    hi = (count >> 8) & 0xFF
    lo = count & 0xFF
    checksum = hi ^ lo ^ 0x55
    
    header = bytearray([0x41, 0x64, 0x61, hi, lo, checksum])
    payload = bytearray([final_r, final_g, final_b]) * LED_COUNT

    try:
        ser.write(header + payload)
        ser.reset_output_buffer()
    except Exception as e:
        print(f"[ERROR] Serial Write Failed: {e}")

# --- MQTT CALLBACKS ---
def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] Connected with result code {rc}")
    
    topic = f"homeassistant/light/{UNIQUE_ID}/config"
    payload = {
        "name": DEVICE_NAME,
        "unique_id": UNIQUE_ID,
        "schema": "json",
        "command_topic": f"homeassistant/light/{UNIQUE_ID}/set",
        "state_topic": f"homeassistant/light/{UNIQUE_ID}/state",
        "supported_color_modes": ["rgb"], 
        "brightness": True,
        "device": {"identifiers": [UNIQUE_ID], "name": DEVICE_NAME, "manufacturer": "DIY Physics"}
    }
    client.publish(topic, json.dumps(payload), retain=True)
    client.subscribe(f"homeassistant/light/{UNIQUE_ID}/set")
    
    # Send initial "OFF" packet to ensure lights are actually dark
    send_adalight(force_log=True)
    client.publish(f"homeassistant/light/{UNIQUE_ID}/state", json.dumps(current_state), retain=True)

def on_message(client, userdata, msg):
    global current_state
    try:
        payload_str = msg.payload.decode()
        data = json.loads(payload_str)
        
        changed = False
        if 'state' in data: 
            current_state['state'] = data['state']
            changed = True
        if 'brightness' in data: 
            current_state['brightness'] = data['brightness']
            changed = True
        if 'color' in data: 
            current_state['color'] = data['color']
            changed = True
        
        if changed:
            save_state() # <--- Save to disk whenever changed
            
        send_adalight(force_log=True)
        client.publish(f"homeassistant/light/{UNIQUE_ID}/state", json.dumps(current_state), retain=True)
        
    except Exception as e:
        print(f"[ERROR] Message parsing: {e}")

# --- MAIN LOOP ---
if __name__ == "__main__":
    # Ensure data directory exists
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    
    setup_serial()
    
    client = mqtt.Client()
    if MQTT_USER: client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"[INIT] Connecting to Broker {MQTT_BROKER}...")
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start() 
    except Exception as e:
        print(f"[FATAL] MQTT Connection failed: {e}")
        sys.exit(1)

    print("[SYSTEM] Running. Heartbeat active.")
    
    while True:
        try:
            if time.time() - last_heartbeat > 2.0:
                send_adalight(force_log=False) 
                last_heartbeat = time.time()
            time.sleep(0.1)
        except KeyboardInterrupt:
            client.loop_stop()
            sys.exit(0)