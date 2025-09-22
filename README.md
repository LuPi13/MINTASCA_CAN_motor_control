# MINTASCA CAN Motor Control

STM32G474RETx 기반의 UART–CAN 브릿지 예제입니다. UART로 입력한 명령을 파싱해 FDCAN으로 송신하고, 수신된 CAN 메시지는 UART로 출력합니다.

## 개요

- UART 명령 입력 → CAN 전송
- CAN 수신 → UART 출력

## 사용 방법

UART 터미널에서 아래 형식으로 전송합니다.

```
<CAN_ID> <DLC> <DATA0> <DATA1> ... <DATA7>
```

예시

```
1A0 8 11 22 33 44 55 66 77 88
D4 4 01 02 03 04
```

- CAN_ID: 16진수 식별자
- DLC: 0~8
- DATAx: 16진수 바이트

수신된 CAN 메시지는 다음 형식으로 UART에 출력됩니다.

```
ID: 0xD4, DLC: 8, Data: 01 02 03 04 05 06 07 08
```

## 참고 사항

- Classic CAN(표준 ID, 0~8바이트) 기준입니다.
- 기본 필터는 0xD4~0x7FF 범위를 수신하도록 설정되어 있습니다.
