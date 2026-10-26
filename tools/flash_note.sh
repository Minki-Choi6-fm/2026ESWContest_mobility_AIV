#!/bin/bash
# 펌웨어를 올리기 "전에" ~/flash_log.txt 에 한 줄 기록만 한다. 이 스크립트는 플래시/빌드/보드 접속을 하지 않는다 (플래시 도구 호출 없음).
# 사용: tools/flash_note.sh <파일.bin 또는 .c> "<올린 사람/PC>"
#   예: tools/flash_note.sh ~/rear_v18d2.bin "최민기/노트북 CubeIDE"
set -u
f="${1:-}"; who="${2:-}"
if [ -z "$f" ] || [ -z "$who" ] || [ ! -f "$f" ]; then
  echo "사용법: $0 <파일> \"<올린 사람/PC>\"   (파일이 있어야 합니다)"; exit 1
fi
h=$(sha256sum "$f" | cut -c1-16)
line="$(date '+%Y-%m-%d %H:%M') | $(basename "$f") | $h | $who"
echo "$line" >> "$HOME/flash_log.txt"
echo "기록했습니다: $line"
echo "(참고: 이 스크립트는 기록만 합니다. 플래시는 따로 직접 하세요.)"
