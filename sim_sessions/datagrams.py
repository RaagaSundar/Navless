#!/usr/bin/env python3
"""Preserve bidirectional XRCE UDP datagram boundaries over an SSH reverse stream."""
import argparse
import asyncio
import socket
import struct

LOOP = '127.0.0.1'


async def pump(udp, reader, writer, agent):
    loop = asyncio.get_running_loop()
    peer = (LOOP, agent) if agent else None

    async def incoming():
        while True:
            length, = struct.unpack('!H', await reader.readexactly(2))
            data = await reader.readexactly(length)
            if peer is not None:
                udp.sendto(data, peer)

    async def outgoing():
        nonlocal peer
        while True:
            ready = loop.create_future()
            loop.add_reader(udp.fileno(), lambda: None if ready.done() else ready.set_result(None))
            try:
                await ready
                data, address = udp.recvfrom(65535)
            finally:
                loop.remove_reader(udp.fileno())
            if peer is None:
                peer = address
            if address == peer:
                writer.write(struct.pack('!H', len(data)) + data)
                await writer.drain()

    jobs = [asyncio.create_task(fn()) for fn in (incoming, outgoing)]
    try:
        done, _ = await asyncio.wait(jobs, return_when=asyncio.FIRST_COMPLETED)
        for job in done:
            job.result()
    finally:
        for job in jobs:
            job.cancel()
        await asyncio.gather(*jobs, return_exceptions=True)
        writer.close()
        await writer.wait_closed()


async def serve(side, tcp, udp_port):
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)
    udp.bind((LOOP, udp_port if side == 'laptop' else 0))
    try:
        if side == 'laptop':
            # Only reachable through the compose loopback publication and the
            # isolated laptop network. One pinned PX4 peer per connection.
            occupied = False
            async def client(reader, writer):
                nonlocal occupied
                if occupied:
                    writer.close()
                    return
                occupied = True
                try:
                    await pump(udp, reader, writer, None)
                except (EOFError, OSError):
                    pass
                finally:
                    occupied = False
            server = await asyncio.start_server(client, '0.0.0.0', tcp)
            async with server:
                await server.serve_forever()
        else:
            while True:
                try:
                    reader, writer = await asyncio.open_connection(LOOP, tcp)
                    await pump(udp, reader, writer, udp_port)
                except (OSError, EOFError):
                    await asyncio.sleep(.25)
    finally:
        udp.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('side', choices=['laptop', 'jetson'])
    parser.add_argument('tcp', type=int)
    parser.add_argument('udp', type=int)
    args = parser.parse_args()
    asyncio.run(serve(args.side, args.tcp, args.udp))
