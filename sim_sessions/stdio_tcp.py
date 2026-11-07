"""Small client-side socket adapter executed by Docker inside the laptop container."""
import os
import select
import socket
import sys

with socket.create_connection(('127.0.0.1', int(sys.argv[1])), timeout=4) as conn:
    conn.settimeout(None)
    inputs = [conn, 0]
    while inputs:
        for source in select.select(inputs, [], [])[0]:
            if source == 0:
                data = os.read(0,65536)
                if data:
                    conn.sendall(data)
                else:
                    conn.shutdown(socket.SHUT_WR)
                    inputs.remove(0)
            else:
                data = conn.recv(65536)
                if not data:
                    sys.exit(0)
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
