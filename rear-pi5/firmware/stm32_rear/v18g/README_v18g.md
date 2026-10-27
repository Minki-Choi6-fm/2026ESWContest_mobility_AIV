# 후방 v18g — UART 오류 폭주 방지 (v18f 기준)

## 왜 만드나 (사실 / 가설)
- 사실: v18f 20분 시험에서 STM32 가 멎기 직전 1초 사이 **USART3(IMU) 오류 콜백이 1건 → 4577건**(ErrorCode=8=오버런, ORE)으로 폭주했고, 그 직후 워치독 리셋이 났다(마지막 단계 radstat, 하드폴트 없음, 다른 단계는 모두 8 ms 이하).
- 가설: HAL 은 오버런이 나면 수신을 끝내고 `HAL_UART_ErrorCallback` 을 부른다. 콜백이 ORE 플래그를 지우지 않은 채 곧바로 수신을 다시 켜면(오류 인터럽트가 켜진 상태) 같은 인터럽트가 끝없이 다시 걸려 메인 루프가 굶고 워치독이 문다. 같은 패턴이 USART1(Pi), USART6(OBD)에도 있다(관측된 것은 USART3 뿐).

## 바뀌는 것 (main_v18f.c 대비, 추가 55줄 / 변경 7줄)
1. 오류 콜백에서 `__HAL_UART_CLEAR_OREFLAG`(SR→DR 읽기)로 ORE/NE/FE/PE 를 먼저 지운다.
2. 5 ms 안에 오류가 9번 넘으면 폭주로 보고 **그 UART 의 수신 재시작을 20 ms 미룬다**. 메인 루프가 20 ms 뒤 다시 시작하고 `UART STORM GUARD #n mask=.. t=..` 한 줄을 출력한다. (인터럽트 안에서는 HAL_GetTick 이 멈출 수 있어 DWT->CYCCNT 로 시간을 잰다. 사이클 카운터가 꺼져 있으면 폭주 판정은 건너뛴다.)
3. `BCMAX` 줄 끝에 `ustorm=` (폭주 보호 누적 횟수) 추가. 배너 `FW v18g (v18f + UART error-storm guard)`.
- 안 바뀐 것: 레이더/IMU/OBD/OLED/프로토콜 로직, IWDG(≈40 ms), RADAR_TIMEOUT_MS 1000, TEST_ALL_CLASSES 1, 브레드크럼.
- 알려진 부작용: 폭주 보호가 걸리면 그 UART 의 바이트를 20 ms 놓친다(IMU 한 프레임 정도). 오류 콜백에서 DR 을 한 번 더 읽으므로 드물게 바이트 1개가 사라질 수 있다.

## 1. 노트북에서 (프로젝트 경로는 본인 것으로)
```
cd <이 폴더>
sha256sum ~/ESWContest/AIV_rear_stm32/Core/Src/main.c | cut -c1-16
```
- **`4a73d8aaf710acca` 이어야 v18f 소스가 맞다.** 다르면 여기서 멈추고 알려 주세요.
- 같을 때만:
```
cp ~/ESWContest/AIV_rear_stm32/Core/Src/main.c ~/ESWContest/backup_src/main_v18f_before_v18g.c
cp main_v18g.c ~/ESWContest/AIV_rear_stm32/Core/Src/main.c
```
- CubeIDE 에서 빌드(Debug). 새 경고가 생기면 알려 주세요. `.bin` 만들기와 해시 보내기는 v18f 때와 같다(`rear_v18g.bin`).

## 2. 올린 뒤 (Pi5)
- 부팅 문구에 `FW v18g`, `PREVBC ...`. 5초마다 `BCMAX ... ustorm=0` (0 이 정상).
- **플래시 직후 보드 전원을 한 번 껐다 켠 뒤 시험**하는 것을 권장(디버거 상태 가설 때문, `PROJECT_STATE.md` 95번).
- 시험 중 `UART STORM GUARD` 줄이 나오면 폭주가 실제로 있었고 보호가 동작했다는 뜻이다(그때 `uerr=` 값을 같이 본다).
- 롤백: `~/rear_v18f.bin` (해시 4e0277752ef17281).

## 3. 이 변경으로 확인되지 않은 것 [확인 필요]
- 폭주의 원인이 정말 'ORE 플래그를 안 지운 재시작'인지(가설). 보호가 걸리면 줄이 남아 확인할 수 있다.
- 폭주를 만든 근본 원인(IMU 선 잡음, 전원 요동 등)은 그대로일 수 있다. 이 변경은 폭주가 와도 워치독 리셋으로 이어지지 않게 하는 방어다.
