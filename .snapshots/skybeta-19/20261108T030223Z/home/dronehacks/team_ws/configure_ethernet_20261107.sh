#!/usr/bin/env bash
set -euo pipefail
[[ $(hostname) == skybeta-19 ]]
iface=enP8p1s0
[[ $(cat /sys/class/net/$iface/address) == 3c:6d:66:b5:1b:00 ]]
[[ $(nmcli -g connection.uuid connection show direct) == 2197ab49-0d09-4382-bc16-bd441c9cbd4f ]]
backup="direct-before-dhcp-$(date -u +%Y%m%dT%H%M%SZ)"
nmcli connection clone direct "$backup"
nmcli connection modify "$backup" connection.autoconnect no
rollback() {
  nmcli connection modify direct ipv4.method manual ipv4.addresses '10.55.0.2/24,169.254.55.2/16' ipv4.never-default no connection.mdns -1
  nmcli --wait 20 connection up direct ifname "$iface"
}
trap rollback ERR
nmcli connection modify direct ipv4.method shared ipv4.addresses '10.55.0.2/24,169.254.55.2/16' ipv4.never-default yes connection.mdns 2 connection.autoconnect yes
nmcli --wait 20 connection up direct ifname "$iface"
[[ $(nmcli -g ipv4.method connection show direct) == shared ]]
ip -4 address show "$iface"
nmcli -f connection.id,connection.autoconnect,connection.mdns,ipv4.method,ipv4.addresses,ipv4.never-default connection show direct
pgrep -a dnsmasq
printf 'Saved backup profile: %s\n' "$backup"
trap - ERR
