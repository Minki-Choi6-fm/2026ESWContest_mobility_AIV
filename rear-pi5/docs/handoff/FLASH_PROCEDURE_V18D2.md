# v18d2 플래시 절차 (따라 하기용 · 이 문서의 명령은 실행하지 않았습니다)

대상: 후방 STM32(NUCLEO-F446RE). 올릴 소스: `firmware/stm32_rear/main_v18d2_final_proposed.c` (sha256 앞 16자 `c69b3c66b48d4700`).
바뀌는 것 (v18c 대비): ① 레이더 타임아웃 100 → 1000 ms ② RADSTAT 에 `txdrop=` 추가 ③ 부팅 배너가 `FW v18d2 (OLED I2C guard + diag + radar timeout 1000 + txdrop)` 로 바뀌고 BOOT2 뒤에 `RESET CAUSE … (re)`, `FW … (re)` 가 한 번 더 나옴. 그 외 동작은 같습니다 (`main_v18c_to_v18d2_final.diff`).

## 0. 올리기 전 체크리스트 (팀 규칙 5, 8 리뷰 기록)
- [ ] 규칙 5: 안전 관련 100 ms 기준을 1000 ms 로 바꾸는 것임을 팀이 알고 있다 (감지 지연 최대 약 1초: 진짜 레이더가 죽어도 그동안 Pi5 는 "타깃 없음(레이더 정상)"으로 본다).
- [ ] 규칙 8: 안전 상수 변경 리뷰자 이름과 날짜를 `~/flash_log.txt` 또는 이 문서 아래에 적었다: 리뷰자 ______ / 날짜 ______
- [ ] 되돌릴 파일 확인: `~/rear_v18c.bin` 의 sha256 앞 16자가 `d47f01a6f976840f` 이다 (`sha256sum ~/rear_v18c.bin`).
- [ ] 사용자(최민기)가 적용을 승인했다 (S9 결정: "추천대로").

## 1. 노트북 CubeIDE 에서 빌드
1. CubeIDE 프로젝트 `AIV_rear_stm32` 의 `Core/Src/main.c` 를 **백업**(복사본 이름 끝에 날짜)한다.
2. `main_v18d2_final_proposed.c` 내용을 `Core/Src/main.c` 에 덮어쓴다 (또는 `main_v18c_to_v18d2_final.diff` 를 적용).
3. **`ssd1306.c` 확인**: 프로젝트의 `Core/Src/ssd1306.c` 가 저장소의 `ssd1306_v18.c` 와 같은 내용이어야 한다(현재 같음: sha256 앞 16자 `1056f87960583b6c`). 다르면 `ssd1306_v18.c` 로 교체한다. `imu_uart.c`, `ssd1306_fonts.c` 는 기존 그대로.
4. Build(망치) → 오류 0 이어야 한다. 경고가 v18c 빌드 때보다 늘었는지 확인. 이 PC(Pi5)에서는 ARM 빌드를 해 본 적이 없다(호스트 구문 검사만, v18c 와 같은 헤더 오류에서 멈춤) → **첫 빌드 결과가 최초의 컴파일 검증**입니다.
5. 빌드 산출물 `.bin`(예: `Debug/AIV_rear_stm32.bin`)을 `rear_v18d2.bin` 이름으로 복사해 둔다.

## 2. .bin 을 Pi5 로 복사
- 노트북 → Pi5 홈(`~/rear_v18d2.bin`)으로 복사(scp, USB 등 편한 방법). 크기가 약 48.8 KB 안팎인지 본다(v18c 는 48,748 B).

## 3. sha256 계산과 기록 (플래시 전에)
```
sha256sum ~/rear_v18d2.bin
~/ESWContest_ws/tools/flash_note.sh ~/rear_v18d2.bin "최민기/Pi5"
```
`~/flash_log.txt` 에 `날짜 | 파일명 | sha256 앞 16자 | 올린 사람/PC` 한 줄이 남는다. 노트북 CubeIDE 의 Run/Debug 로 올릴 때도 올리기 전에 같은 한 줄을 직접 적는다. (`flash_note.sh` 는 기록만 하며 플래시를 하지 않는다.)

## 4. 플래시 (예시 — 텍스트일 뿐, 이 문서를 쓴 쪽은 실행하지 않았음)
```
st-flash --connect-under-reset --reset write ~/rear_v18d2.bin 0x8000000
```
- 실행 전 `/dev/ttyACM0` 를 읽는 프로그램(로거 등)이 있으면 끝낸다. 성공 메시지(`Flash written and verified!`)를 확인한다.
- 실패하면 NUCLEO 의 USB 를 다시 꽂고 `--connect-under-reset` 상태로 재시도.

## 5. 책상 검증
1. **빈 장면**(레이더 앞에 아무것도 없음)에서 RADSTAT 의 `safe=` 값이 **더 이상 증가하지 않는다**(기존 약 3.4회/s). `RADAR TIMEOUT -> SAFE MODE` 줄이 안 나온다.
2. 물체를 놓으면 `b`, `exp` 가 이전처럼 올라간다. RADSTAT 줄에 `txdrop=0` 이 보인다.
3. `OLEDSTAT ok=1`, `fails=0` 이고 `pushes` 가 증가한다. OLED 두 줄(`AIV REAR MCP:…`, `RADAR: OK`) 정상.
4. 레이더 CAN 선(또는 레이더 전원)을 뽑으면 **약 1초 뒤** `RADAR TIMEOUT -> SAFE MODE` 가 나오고 OLED 가 `RADAR: SAFE (no data)` 가 된다. 다시 꽂으면 `RADAR RECOVERED`.

## 6. 배너 재캡처 (읽는 쪽을 먼저 띄우고 리셋)
- 이 단계는 **사용자가 "지금 리셋 누른다"라고 말할 때만** 에이전트가 실행한다(읽기 전용, 포트에 쓰기 없음).
- 순서: ① 읽기 프로세스를 먼저 시작(15초) ② 시작 메시지가 나온 뒤 NUCLEO 검정 RESET 버튼을 누른다.
- 기대 결과: 리셋 직후 `RESET CAUSE: …`, `FW v18d2 (…)`, `BOOT1`, 약 2초 뒤 `BOOT2`, 이어서 `RESET CAUSE: … (re)`, `FW v18d2 (…) (re)`. **두 줄이 나오면 보드가 v18d2 라는 것이 시리얼로 처음 확정**된다. 첫 번째 배너가 또 없고 (re) 만 있어도 v18d2 확정(원인 가설 H1/H2 힌트로 기록). 둘 다 없으면 v18d2 가 올라가지 않은 것으로 의심.

## 7. 실패 시 되돌리기
```
sha256sum ~/rear_v18c.bin     # 앞 16자가 d47f01a6f976840f 인지 확인
~/ESWContest_ws/tools/flash_note.sh ~/rear_v18c.bin "최민기/Pi5 (되돌림)"
st-flash --connect-under-reset --reset write ~/rear_v18c.bin 0x8000000
```
(위 명령도 텍스트 예시.) 되돌린 뒤 §5 의 1~3 을 확인(v18c 라면 빈 장면에서 `safe=` 가 다시 증가하고 `txdrop=` 는 없음).

## 8. 문제가 생길 만한 점
- 레이더가 정말 죽은 경우 감지가 약 1초 늦다(규칙 5 항목).
- 빈 장면 침묵이 1초를 넘는 경우(10/16 로그 29건, `SAFE_GAP_EVENTS.md`)는 여전히 Safe Mode 로 들어갈 수 있다 — 이는 정상 동작이며 원인은 별도 측정.
