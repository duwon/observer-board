# BLE Observer Board 

nRF52833 기반 BLE 수신(Observer) 보드의 펌웨어

---

## 1. MCU 및 기본 설정

| 항목 | 내용 |
|------|------|
| **MCU** | Nordic nRF52833 |
| **클럭** | HFXO 32 MHz + LFXO 32.768 kHz |
| **SDK 버전** | nRF Connect SDK v3.1.0 (Zephyr v4.1.99 기반) |
| **보드 타깃** | `nrf52833dk_nrf52833` |

---

## 2. 핀맵

| 기능 | 핀 | 방향 | 설명 |
|------|----|------|------|
| **LFXO** | P0.00 / P0.01 | — | 32.768 kHz 저주파 크리스털 |
| **UART TX/RX** | P0.15 / P0.17 | 출력 / 입력 | UART0 115200 8N1 (HW Flow Control off) |
| **LED** | P0.11 | 출력 | 상태 LED (Active High, 1 초 토글) |
| **RESET** | P0.18 | 입력 | JTAG Reset |
| **USB D+/D-** | 전용핀 | 양방향 | USB CDC 통신용 |

---

## 3. 기능

### USB CDC (가상 COM 포트)
- DTR 신호 감지 후 연결된 터미널로만 메시지를 전송  
- `Init_usbdev()` 함수에서 초기화 및 대기  
- `usbdev_puts()` 함수로 텍스트 송신  
- PID `0x0001` 로 설정하여 Windows에서 포트 자동 인식 가능  

### UART0 디버그 로그
- `LOG_INF()` / `printk()` 등 Zephyr 로그 출력이 UART0로 출력됨  
- USB CDC와 별도로 항상 활성화되어 있음  
- 개발 시 디버깅 로그 전용 포트로 사용  

### GPIO 제어
- `gpio_if.c` 모듈에서 LED 제어 함수 제공  
  - `gpio_if_led_on()`, `gpio_if_led_off()`, `gpio_if_led_toggle()`
- 부트 완료 후 1 초 주기로 상태 LED 토글  

### BLE 스캔 기능
- `ble_adv.c` 모듈에서 BLE Observer 기능 구현  
- Active Scan 모드로 광고/스캔 응답 수신  
- Manufacturer Specific Data 중 **Company ID = 0xFFFF** 만 필터링  
- Phase 1/Phase 2 데이터 구조체 파싱 및 로그 출력  
- 모든 로그는 UART0로 출력됨  

---

## 4. 프로젝트 구조

```

BLE-Observer-Board/
├── CMakeLists.txt
├── prj.conf
├── boards/
│   └── observer.overlay
├── include/
│   ├── ble_adv.h
│   ├── gpio_if.h
│   └── usb_dev.h
└── src/
    ├── main.c          # 메인 엔트리: 초기화 + LED + BLE 스캔 시작
    ├── usb_dev.c       # USB CDC 초기화 및 DTR 대기/송신
    ├── gpio_if.c       # LED GPIO 제어 함수
    └── ble_adv.c       # BLE 스캔 및 데이터 파싱 로직

````

---



