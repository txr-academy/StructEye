"""
===============================================================
  STM32 TCP Receiver — Run this on any Windows/Mac/Linux PC
===============================================================

  1. Set your PC's Ethernet IP to:  192.168.7.10
     Subnet mask:                   255.255.255.0
  2. Run this script:               python tcp_server.py
  3. Power on / reset the STM32
  4. You should see "Hello World" printed every 2 seconds

  Port: 7777  (must match PC_TCP_PORT in STM32 firmware)

  Press Ctrl+C to stop.
===============================================================
"""

import socket

HOST = "0.0.0.0"   # Listen on ALL interfaces (works on any PC)
PORT = 7777        # Must match PC_TCP_PORT in tcp_hello.h

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[*] Listening on port {PORT} — waiting for STM32 ...")

        while True:
            conn, addr = s.accept()
            print(f"[+] STM32 connected from {addr[0]}:{addr[1]}")

            try:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        print("[-] STM32 disconnected.")
                        break
                    print(f"    >> {data.decode('utf-8', errors='replace').strip()}")
            except ConnectionResetError:
                print("[-] Connection reset by STM32.")
            finally:
                conn.close()

            print("[*] Waiting for next connection ...")

if __name__ == "__main__":
    main()
