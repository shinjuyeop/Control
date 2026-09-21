# Hardware Documents

이 디렉터리에는 차량의 구동 및 조향 시스템에 사용되는 하드웨어의 제조사 매뉴얼과 통신 프로토콜 문서를 보관합니다. 내부 설정 메모는 아래에서 공식 자료와 구분합니다.

## Display Controller

### Sitronix ST7796S Datasheet V1.0

- 파일: [ST7796S-Sitronix (display).pdf](<./ST7796S-Sitronix (display).pdf>)
- 320RGB × 480 dot, 262K color TFT controller/driver의 제조사 데이터시트
- 3-line/4-line SPI timing과 interface selection, display command 및 register 정의 포함
- 현재 QDtech MSP4030 모듈의 ST7796S command, pixel format, 방향 설정과 SPI timing 확인에 사용
- 이 문서는 raw ST7796S IC 기준이므로 MSP4030 모듈의 5 V 입력, level shifter,
  backlight와 header pinout 근거로 사용하지 않음

## MSP4030/MSP4031 Module and Capacitive Touch

- [MSP4030/MSP4031 Specification V1.0](./MSP4030_MSP4031_Specification_EN_V1.0.pdf):
  MSP4030은 터치 없음, MSP4031은 FT6336U I2C 정전식 터치 모델. 공용 VCC/GND,
  헤더 핀 기능과 모듈 전압을 설명합니다.
- [제조사 제품 페이지](https://www.lcdwiki.com/4.0inch_Capacitive_SPI_Module_ST7796)
- [모듈 회로도](https://www.lcdwiki.com/res/MSP4030_MSP4031/4.0inch_SPI_Module_MSP4030_MSP4031_Schematic.pdf)
- [FT6336U datasheet V1.1](https://www.lcdwiki.com/res/MSP4030_MSP4031/DFT6336UDataSheetV1.1.pdf)
- [FT6336U register table](https://www.lcdwiki.com/res/MSP4030_MSP4031/FT6336U_Register.xlsx)

UNO 드라이버의 주소/리셋 절차는 제품 페이지의 MSP4030/MSP4031 V1.0 예제 중
`Demo_UNO_Mega2560/Install libraries/LCDWIKI_TOUCH/ft6336g.cpp`와 `.h`를
참조합니다. I2C register table의 normal work mode만 사용하며 factory mode,
터치 칩 firmware, calibration 설정은 변경하지 않습니다.

## Drive Motor Controller

### GoldenMotor EZkontrol User Manual

- 파일: [GoldenMotor_EZkontrol_User_Manual.pdf](./GoldenMotor_EZkontrol_User_Manual.pdf)
- EZkontrol 컨트롤러 제품군의 사양, 모델, 배선 및 설정 방법
- 현재 차량의 구동 모터 컨트롤러 설정과 배선 확인에 사용

### GoldenMotor EZkontrol MCU-to-VCU CAN Protocol V1.0

- 파일: [GoldenMotor_EZkontrol_MCU_to_VCU_CAN_Protocol_V1.0.pdf](./GoldenMotor_EZkontrol_MCU_to_VCU_CAN_Protocol_V1.0.pdf)
- VCU와 모터 컨트롤러 사이의 CAN ID 및 데이터 형식 정의
- 제어 명령, 상태 피드백, 핸드셰이크 및 Life Signal 규격 포함

### GoldenMotor EZkontrol CAN Protocol Instruction V1.0

- 파일: [GoldenMotor_EZkontrol_CAN_Protocol_Instruction_V1.0.pdf](./GoldenMotor_EZkontrol_CAN_Protocol_Instruction_V1.0.pdf)
- CAN_H, CAN_L 및 종단저항 배선 안내
- 통신 속도, 컨트롤러 주소, 핸드셰이크 및 데이터 변환 예제 포함

### GoldenMotor EZkontrol MCU-to-Meter CAN Protocol V1.1

- 파일: [GoldenMotor_EZkontrol_MCU_to_Meter_CAN_Protocol_V1.1.pdf](./GoldenMotor_EZkontrol_MCU_to_Meter_CAN_Protocol_V1.1.pdf)
- 모터 컨트롤러에서 계기판으로 전달하는 CAN 상태 데이터 규격 (V1.1, 2023-07-27)
- MCU-to-VCU 제어 명령 프로토콜과 구분해서 참고

### LCD 상태·고장 표시용 SIF 프로토콜

- 파일: [新思维-高标100513.pdf](./新思维-高标100513.pdf)
- LCD 통신 프로토콜 2.0: 컨트롤러에서 표시기로 보내는 단선 SIF 상태·고장 전송 자료
- EZkontrol CAN 제어 명령이나 회생 torque 명령의 근거로 사용하지 않음

### 내부 설정 참고 메모

- 파일: [EZKONTROL_SETTINGS_NOTES.md](./EZKONTROL_SETTINGS_NOTES.md)
- 설정 항목 해설, 2026-09-08 설정 기록, 날짜 미상 설정 목록
- 제조사 공식 문서 및 현재 장치 readback과 구분

## Steering Motor

### KEYA KY170 Auto Steering Motor User Manual V1.1

- 파일: [KEYA_KY170DJD01005-08G-V1_User_Manual_V1.1.pdf](./KEYA_KY170DJD01005-08G-V1_User_Manual_V1.1.pdf)
- 적용 모델: `KY170DJD01005-08G-V1`
- 조향 모터의 전기적 사양, 핀맵, 절대 엔코더 및 설정 방법
- CAN 기반 속도, 토크, 절대 위치 및 상대 위치 제어 명령 포함
