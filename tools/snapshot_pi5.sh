#!/usr/bin/env bash
# snapshot_pi5.sh — READ-ONLY state report for the AIV rear Pi5.
# Writes nothing except one report file in /tmp (or $OUT). Run:  bash tools/snapshot_pi5.sh
# Purpose: find out what coworkers changed since the handoff docs were written.
OUT="${OUT:-/tmp/aiv_snapshot_$(date +%Y%m%d_%H%M%S).txt}"

{
echo "=== AIV Pi5 snapshot $(date -Is) on $(hostname) ($(whoami)) ==="
uname -a; echo

echo "=== Devices ==="
ls -l /dev/ttyAMA* /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/ 2>&1
echo "--- who holds the serial ports:"
command -v fuser >/dev/null && fuser -v /dev/ttyAMA2 /dev/ttyACM0 2>&1 || echo "(fuser not installed)"
echo

echo "=== Services (aiv*, gpsd) ==="
systemctl list-units --type=service --all 2>/dev/null | grep -Ei "aiv|gpsd" || echo "none listed"
systemctl cat aiv-ego-speed.service 2>&1 | head -20
echo

echo "=== Running python/tmux ==="
ps -eo pid,etime,cmd | grep -Ei "python|tmux|drive_logger|v2v|yolo|gpsd" | grep -v grep
echo

echo "=== Files in ~ (newest first, py/sh/md/service/bin) ==="
ls -lt --time-style=long-iso ~ 2>/dev/null | grep -Ei "\.(py|sh|md|bin|json|yaml|ino|c|h)$|^d" | head -60
echo
echo "=== Python files: size, mtime, sha256 (first 12) ==="
for f in ~/*.py ~/*/*.py; do
  [ -f "$f" ] && printf "%s  %s  %s  %s\n" "$(stat -c '%y' "$f" | cut -c1-16)" "$(stat -c '%s' "$f")" "$(sha256sum "$f" | cut -c1-12)" "$f"
done 2>/dev/null | sort -r | head -80
echo

echo "=== Git repos under ~ (status) ==="
for g in $(find ~ -maxdepth 3 -name .git -type d 2>/dev/null); do
  d=$(dirname "$g"); echo "--- $d"; git -C "$d" status -sb 2>&1 | head -15; git -C "$d" log --oneline -5 2>&1
done
echo

echo "=== Recent logs ==="
ls -dt ~/vehicle_test_logs/drive_* ~/aiv_drive_logs/drive_* 2>/dev/null | head -5
df -h ~ | tail -1
echo

echo "=== Config touched? ==="
grep -nE "uart|dtoverlay|enable_uart" /boot/firmware/config.txt 2>/dev/null
cat /etc/default/gpsd 2>/dev/null | grep -vE "^\s*#|^\s*$"
echo

echo "=== Power / throttling ==="
vcgencmd get_throttled 2>&1; vcgencmd measure_temp 2>&1
vcgencmd pmic_read_adc 2>/dev/null | grep -E "EXT5V_V|3V3_SYS_V"
echo

echo "=== STM32 / ST-Link visible? ==="
lsusb 2>/dev/null | grep -Ei "st-?link|stmicro|realsense|intel|silicon|cp210" || echo "no matching usb devices"
command -v st-flash >/dev/null && echo "st-flash: $(command -v st-flash)" || echo "st-flash not found"
echo
echo "=== done; report: $OUT ==="
} 2>&1 | tee "$OUT"
