#!/usr/bin/env bash
# usage: labrun.sh "<shell cmd>" [wait_s]  -> types the command into the FlyX lab Workbench terminal (gstack browse, headed), prints its output
B="$HOME/.claude/skills/gstack/browse/dist/browse"
W=${2:-8}; N="__DONE_${RANDOM}${RANDOM}__"
"$B" --headed frame main >/dev/null 2>&1
"$B" --headed frame 'iframe[src*="/code/"]' >/dev/null 2>&1
"$B" --headed js "(()=>{const x=[...document.querySelectorAll('.xterm')].find(e=>e.offsetWidth>0);x.querySelector('textarea').focus();return 1})()" >/dev/null 2>&1
"$B" --headed type "{ $1 ; } > /tmp/cl_out.txt 2>&1; echo $N >> /tmp/cl_out.txt" >/dev/null
"$B" --headed press Enter >/dev/null
for i in $(seq 1 $W); do
  out=$("$B" --headed js "new Promise(r=>setTimeout(r,1000)).then(()=>fetch('/code/vscode-remote-resource?path=/tmp/cl_out.txt',{cache:'no-store'})).then(r=>r.text())" 2>&1 | grep -v UNTRUSTED)
  echo "$out" | grep -q "^$N" && { echo "$out" | sed "s/.*]633;C//" | grep -v "$N"; exit 0; }
done
echo "$out"; echo "(timed out waiting for $N)"
