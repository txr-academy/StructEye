import paho.mqtt.client as mqtt, json
from datetime import datetime

def on_connect(c, u, f, rc):
    c.subscribe("structeye/#")
    print("Subscribed. Waiting for packets...\n")

def on_message(c, u, msg):
    d = json.loads(msg.payload.decode())
    alert = {0:"OK", 1:"WARN", 2:"CRITICAL"}.get(d["alert"], "?")
    print(f"[{datetime.now().strftime('%H:%M:%S')}] Node:{d['node']} Seq:{d['seq']}")
    print(f"  Vib:{d['vib']} m/s²  Tilt:{d['tilt']}°  Alert:{alert}")
    print(f"  Accel X={d['ax']} Y={d['ay']} Z={d['az']}")
    print("-"*40)

c = mqtt.Client()
c.on_connect = on_connect
c.on_message = on_message
c.connect("192.168.1.69", 1883, 60)
c.loop_forever()