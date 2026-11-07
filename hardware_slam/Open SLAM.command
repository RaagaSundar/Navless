#!/bin/bash
# Double-click on this Mac to reopen the live viewer. SSH asks for the existing
# Jetson password when needed. No password or private key is stored here.
set -e
if ! curl -fsS --max-time 2 http://127.0.0.1:8765/state >/dev/null 2>&1; then
  ssh -fN -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 \
    -o ServerAliveCountMax=3 -L 127.0.0.1:8765:127.0.0.1:8765 \
    dronehacks@skybeta-11.local
fi
open http://127.0.0.1:8765/
