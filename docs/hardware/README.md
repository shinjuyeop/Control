# Hardware Documents

이 디렉터리에는 차량의 구동 및 조향 시스템에 사용되는 하드웨어의 제조사 매뉴얼과 통신 프로토콜 문서를 보관합니다.

## Display Controller

### Sitronix ST7796S Datasheet V1.0

- 파일: [ST7796S-Sitronix (display).pdf](<./ST7796S-Sitronix (display).pdf>)
- 320RGB × 480 dot, 262K color TFT controller/driver의 제조사 데이터시트
- 3-line/4-line SPI timing과 interface selection, display command 및 register 정의 포함
- 현재 QDtech MSP4030 모듈의 ST7796S command, pixel format, 방향 설정과 SPI timing 확인에 사용
- 이 문서는 raw ST7796S IC 기준이므로 MSP4030 모듈의 5 V 입력, level shifter,
  backlight와 header pinout 근거로 사용하지 않음

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

## Steering Motor

### KEYA KY170 Auto Steering Motor User Manual V1.1

- 파일: [KEYA_KY170DJD01005-08G-V1_User_Manual_V1.1.pdf](./KEYA_KY170DJD01005-08G-V1_User_Manual_V1.1.pdf)
- 적용 모델: `KY170DJD01005-08G-V1`
- 조향 모터의 전기적 사양, 핀맵, 절대 엔코더 및 설정 방법
- CAN 기반 속도, 토크, 절대 위치 및 상대 위치 제어 명령 포함
