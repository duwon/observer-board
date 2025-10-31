
# BLE Sensor Board (Broadcaster) 

## 1️ MCU 및 기본 설정 

* **MCU:** nRF52833
* **클럭:** HFXO 32 MHz + LFXO 32.768 kHz
* **SDK:** nRF Connect SDK v3.1.0 (Zephyr v4.1.99)
* **보드 타깃:** `nrf52833dk_nrf52833` (커스텀 오버레이 적용)

---

## 2️ 핀맵 요약

| 기능               | 핀                 | 방향    | 설명                             |
| ---------------- | ----------------- | ----- | ------------------------------ |
| LFXO             | P0.00 / P0.01     | —     | 32.768 kHz                     |
| **UART TX/RX**   | **P0.15 / P0.17** | 출력/입력 | UART0 115200 8N1 (**HWFC 끔**)  |
| **LED**          | P0.11             | 출력    | 상태 LED (Active High)           |
| **RESET**        | P0.18             | 입력    | JTAG Reset                     |