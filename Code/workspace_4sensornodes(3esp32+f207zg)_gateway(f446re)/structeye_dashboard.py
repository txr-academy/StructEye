"""
StructEye Dashboard — STM32F446RE Gateway
Parses format:
  --- Packet Received ---
  Node: 2 | Seq: 166 | RX: 291 Valid: 288
  Accel: X=9.99 Y=0.08 Z=2.44 m/s2
  Gyro:  X=-1.12 Y=0.69 Z=1.08 deg/s
  Vib: 10.28 m/s2 | Tilt: 13.74 deg
  Status: 0x00 | Alert: 1 [WARN]

Install: py -m pip install pyserial rich
Run:     py structeye_dashboard.py
"""

import serial
import serial.tools.list_ports
import threading
import time
import re
import sys
from datetime import datetime
from collections import deque

try:
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel
    from rich.live import Live
    from rich.text import Text
    from rich.align import Align
    from rich.columns import Columns
    from rich import box
except ImportError:
    print("Run:  py -m pip install pyserial rich")
    sys.exit(1)

SERIAL_PORT = "COM16"
BAUD_RATE   = 115200
REFRESH_HZ  = 2
MAX_HISTORY = 30

class Node:
    def __init__(self, nid):
        self.nid=nid; self.name=f"Node {nid}"
        self.seq=0; self.last_seq=-1
        self.ax=self.ay=self.az=0.0
        self.gx=self.gy=self.gz=0.0
        self.vib=0.0; self.tilt=0.0
        self.status=0; self.alert=0
        self.packet_count=0; self.missed=0
        self.last_seen=None
        self.vib_hist=deque(maxlen=MAX_HISTORY)
        self.tilt_hist=deque(maxlen=MAX_HISTORY)

nodes={}; total_rx=0; total_valid=0
serial_status="Connecting..."; lock=threading.Lock()
raw_log=deque(maxlen=60); console=Console()
pkt={}

def reset_pkt(): pkt.clear()

def commit():
    nid=pkt.get('nid')
    if nid is None: return
    with lock:
        if nid not in nodes: nodes[nid]=Node(nid)
        n=nodes[nid]; seq=pkt.get('seq',0)
        if n.last_seq>=0:
            gap=(seq-n.last_seq-1)%256
            if gap>0: n.missed+=gap
        n.last_seq=seq; n.seq=seq
        n.ax=pkt.get('ax',0.0); n.ay=pkt.get('ay',0.0); n.az=pkt.get('az',0.0)
        n.gx=pkt.get('gx',0.0); n.gy=pkt.get('gy',0.0); n.gz=pkt.get('gz',0.0)
        n.vib=pkt.get('vib',0.0); n.tilt=pkt.get('tilt',0.0)
        n.status=pkt.get('status',0); n.alert=pkt.get('alert',0)
        n.packet_count+=1; n.last_seen=datetime.now()
        n.vib_hist.append(n.vib); n.tilt_hist.append(n.tilt)
    reset_pkt()

def parse(line):
    global total_rx,total_valid
    line=line.strip()
    if not line: return
    with lock: raw_log.append(f"{datetime.now().strftime('%H:%M:%S')}  {line}")

    if '--- Packet Received ---' in line:
        if pkt.get('nid') is not None and 'vib' in pkt: commit()
        reset_pkt(); return

    m=re.search(r'Node:\s*(\d+)\s*\|\s*Seq:\s*(\d+)\s*\|\s*RX:\s*(\d+)\s*Valid:\s*(\d+)',line)
    if m:
        pkt['nid']=int(m.group(1)); pkt['seq']=int(m.group(2))
        with lock: total_rx=int(m.group(3)); total_valid=int(m.group(4))
        return

    m=re.search(r'Accel:\s*X=([+-]?\d+\.?\d*)\s+Y=([+-]?\d+\.?\d*)\s+Z=([+-]?\d+\.?\d*)',line)
    if m: pkt['ax']=float(m.group(1)); pkt['ay']=float(m.group(2)); pkt['az']=float(m.group(3)); return

    m=re.search(r'Gyro:\s*X=([+-]?\d+\.?\d*)\s+Y=([+-]?\d+\.?\d*)\s+Z=([+-]?\d+\.?\d*)',line)
    if m: pkt['gx']=float(m.group(1)); pkt['gy']=float(m.group(2)); pkt['gz']=float(m.group(3)); return

    m=re.search(r'Vib:\s*([+-]?\d+\.?\d*).*Tilt:\s*([+-]?\d+\.?\d*)',line)
    if m: pkt['vib']=float(m.group(1)); pkt['tilt']=float(m.group(2)); return

    m=re.search(r'Status:\s*(0x[0-9A-Fa-f]+)\s*\|\s*Alert:\s*(\d+)',line)
    if m: pkt['status']=int(m.group(1),16); pkt['alert']=int(m.group(2)); commit(); return

def find_port():
    ports=serial.tools.list_ports.comports()
    for p in ports:
        d=(p.description or "").lower()
        if any(x in d for x in ['stlink','st-link','stm32','cp210','ch340']): return p.device
    return ports[0].device if ports else None

def serial_thread():
    global serial_status
    port=SERIAL_PORT or find_port()
    if not port: serial_status="❌  No port found — plug in F446RE USB"; return
    while True:
        try:
            serial_status=f"Connecting → {port}..."
            with serial.Serial(port,BAUD_RATE,timeout=1) as ser:
                serial_status=f"✅  {port}  @  {BAUD_RATE} baud"
                while True:
                    raw=ser.readline()
                    if raw:
                        try: parse(raw.decode('utf-8',errors='replace'))
                        except: pass
        except Exception as e:
            serial_status=f"❌  {str(e)[:55]}"; time.sleep(3)

AC={0:"bright_green",1:"yellow",2:"bright_red"}
AL={0:"✅  OK",1:"⚠️  WARN",2:"🚨 CRITICAL"}
BC={0:"green",1:"yellow",2:"red"}

def spark(hist,w=22):
    chars=" ▁▂▃▄▅▆▇█"; data=list(hist)
    if not data: return Text("no data",style="dim")
    mx=max(data) if max(data)>0 else 1
    return Text("".join(chars[min(int(v/mx*8),8)] for v in data[-w:]),style="bright_cyan")

def bar(v,mx=15.0,w=16):
    ratio=min(v/mx,1.0); filled=int(ratio*w)
    b="█"*filled+"░"*(w-filled)
    c="bright_green" if ratio<0.67 else("yellow" if ratio<0.87 else "bright_red")
    return Text(f"{b} {v:5.2f}",style=c)

def ago(dt):
    if dt is None: return Text("never",style="dim red")
    s=(datetime.now()-dt).total_seconds()
    c="bright_green" if s<15 else("yellow" if s<35 else "red")
    return Text(f"{s:.0f}s ago" if s<60 else f"{s/60:.1f}m ago",style=c)

def header():
    now=datetime.now().strftime("%Y-%m-%d   %H:%M:%S")
    with lock: sc=serial_status
    t=Text()
    t.append("  ⚡  STRUCTEYE  ",style="bold white on navy_blue")
    t.append("  LoRa Structural Health Monitor  ",style="dim white")
    t.append(f"  {now}  ",style="cyan")
    t.append(f"  {sc}  ",style="bright_green" if "✅" in sc else "red")
    return Panel(Align.center(t),style="navy_blue",padding=(0,0))

def summary():
    t=Table(box=box.SIMPLE_HEAD,header_style="bold white on grey23",
            padding=(0,1),expand=True,show_edge=False)
    t.add_column("Node",style="bold cyan",width=8)
    t.add_column("Seq",style="dim",width=5)
    t.add_column("Vibration (m/s²)",width=22)
    t.add_column("Tilt (°)",width=18)
    t.add_column("Accel X / Y / Z",width=24)
    t.add_column("Gyro X / Y / Z",width=24)
    t.add_column("RX",style="cyan",width=5)
    t.add_column("Missed",width=8)
    t.add_column("Alert",width=14)
    t.add_column("Last Seen",width=10)
    with lock: nlist=sorted(nodes.values(),key=lambda n:n.nid)
    if not nlist:
        t.add_row(Text("Waiting for packets...",style="dim"),*[""]*9); return t
    for n in nlist:
        mc="bright_red" if n.missed>3 else("yellow" if n.missed>0 else "dim")
        t.add_row(n.name,str(n.seq),bar(n.vib,15.0),bar(n.tilt,25.0),
                  Text(f"{n.ax:+.2f} / {n.ay:+.2f} / {n.az:+.2f}",style="blue"),
                  Text(f"{n.gx:+.2f} / {n.gy:+.2f} / {n.gz:+.2f}",style="magenta"),
                  str(n.packet_count),Text(str(n.missed),style=mc),
                  Text(AL[n.alert],style=f"bold {AC[n.alert]}"),ago(n.last_seen))
    return t

def detail_cards():
    with lock: nlist=sorted(nodes.values(),key=lambda n:n.nid)
    if not nlist:
        return Panel(Text("No nodes heard yet",style="dim"),title="Node Details",border_style="grey50")
    panels=[]
    for n in nlist:
        flags=[]
        if n.status&0x01: flags.append("[red]VIB![/]")
        if n.status&0x02: flags.append("[red]TILT![/]")
        if n.status&0x04: flags.append("[yellow]FAULT[/]")
        if n.status&0x08: flags.append("[blue]BOOT[/]")
        fstr=" ".join(flags) if flags else "[dim green]NORMAL[/]"
        g=Table.grid(padding=(0,1))
        g.add_column(style="dim",width=10); g.add_column(min_width=22)
        g.add_row("Alert",Text(AL[n.alert],style=f"bold {AC[n.alert]}"))
        g.add_row("Flags",Text.from_markup(fstr))
        g.add_row("Seq",Text(str(n.seq)))
        g.add_row("Packets",Text(f"{n.packet_count}  missed:{n.missed}",
                                 style="cyan" if n.missed==0 else "yellow"))
        g.add_row("",Text(""))
        g.add_row("Vib",bar(n.vib,15.0,14))
        g.add_row("Tilt",bar(n.tilt,25.0,14))
        g.add_row("",Text(""))
        g.add_row("Vib hist",spark(n.vib_hist,20))
        g.add_row("",Text(""))
        g.add_row("Accel X",Text(f"{n.ax:+.2f} m/s²",style="bright_blue"))
        g.add_row("Accel Y",Text(f"{n.ay:+.2f} m/s²",style="bright_blue"))
        g.add_row("Accel Z",Text(f"{n.az:+.2f} m/s²",style="blue"))
        g.add_row("Gyro X",Text(f"{n.gx:+.2f} °/s",style="magenta"))
        g.add_row("Gyro Y",Text(f"{n.gy:+.2f} °/s",style="magenta"))
        g.add_row("Gyro Z",Text(f"{n.gz:+.2f} °/s",style="magenta"))
        g.add_row("",Text(""))
        g.add_row("Last",ago(n.last_seen))
        panels.append(Panel(g,title=f"[bold white] {n.name} [/]",
            subtitle=Text(AL[n.alert],style=f"bold {AC[n.alert]}"),
            border_style=BC[n.alert],padding=(0,1)))
    return Columns(panels,equal=True,expand=True)

def stats():
    with lock: rx=total_rx; v=total_valid; nc=len(nodes)
    d=rx-v; pct=(d/rx*100) if rx>0 else 0.0
    hc="bright_green" if pct<5 else("yellow" if pct<20 else "bright_red")
    hl="EXCELLENT" if pct<5 else("DEGRADED" if pct<20 else "POOR")
    g=Table.grid(padding=(0,2))
    g.add_column(style="dim",width=18); g.add_column(style="white",width=10)
    g.add_column(style="dim",width=18); g.add_column(style="white",width=10)
    g.add_row("Total RX",Text(str(rx),style="cyan"),"Valid",Text(str(v),style="bright_green"))
    g.add_row("Dropped",Text(str(d),style="red"),"Drop Rate",Text(f"{pct:.1f}%",style=hc))
    g.add_row("Active Nodes",Text(str(nc),style="cyan"),"Link Health",Text(hl,style=f"bold {hc}"))
    if nc>1: g.add_row("",Text("  ⚠  Add startup delays to reduce collisions",style="yellow"),"",Text(""))
    return Panel(g,title="[bold white] Network Statistics [/]",border_style="blue",padding=(0,1))

def log_panel():
    with lock: lines=list(raw_log)[-10:]
    t=Text()
    for l in lines:
        if "CRITICAL" in l: t.append(l+"\n",style="red")
        elif "WARN" in l: t.append(l+"\n",style="yellow")
        elif "DROP" in l: t.append(l+"\n",style="dim red")
        elif "Packet Received" in l: t.append(l+"\n",style="dim cyan")
        else: t.append(l+"\n",style="dim")
    return Panel(t,title="[bold white] Raw Serial Log [/]",border_style="grey30",padding=(0,1))

def screen():
    from rich.console import Group
    return Group(header(),
        Panel(summary(),title="[bold white] All Nodes — Live Summary [/]",
              border_style="bright_blue",padding=(0,0)),
        detail_cards(),stats(),log_panel())

def main():
    console.clear()
    t=threading.Thread(target=serial_thread,daemon=True); t.start()
    time.sleep(1.0)
    try:
        with Live(screen(),console=console,refresh_per_second=REFRESH_HZ,screen=True) as live:
            while True:
                live.update(screen()); time.sleep(1.0/REFRESH_HZ)
    except KeyboardInterrupt:
        console.clear(); console.print("\n[bold cyan]StructEye stopped.[/bold cyan]\n")

if __name__=="__main__":
    main()