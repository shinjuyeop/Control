# RP2350 프로젝트 CAD 자료

이 디렉터리는 RP2350 프로젝트의 enclosure, bracket, PCB와 harness 설계에 필요한
원본 CAD·도면을 부품별로 모읍니다. 파일을 내려받았다는 사실이 주문품과의 기구적
일치를 의미하지는 않습니다. 아래 등급과 각 폴더의 `SOURCE.md`를 먼저 확인하십시오.

수집 기준일: 2026-08-26

## 자료 등급

| 등급 | 의미 |
|---|---|
| Exact | 제조사와 model/revision이 일치하는 원본 설계 자료 |
| Candidate | model은 맞지만 AliExpress 복제판·leadform과 실물 대조 필요 |
| Reference only | 일반 외형 참고용이며 mounting hole 제작 기준으로 사용 금지 |
| Drawing only | 공개 STEP이 없어 치수도만 보관 |

## 보유 자료

| 부품 | 폴더 | 현재 가장 유용한 자료 | 등급 |
|---|---|---|---|
| XIAO RP2350 | [`xiao_rp2350/`](xiao_rp2350/) | KiCad PCB, DXF, footprint | Exact |
| Adafruit 5832 INA228 | [`ina228_5832/`](ina228_5832/) | 완성 보드 STEP/F3D, Eagle PCB | Candidate |
| ACS758 | [`acs758/`](acs758/) | package drawing, 공식 eval-board Gerber | Candidate |
| 100 A/75 mV shunt | [`shunt_100a_75mv/`](shunt_100a_75mv/) | FL-2 치수도 | Drawing only |
| LM2596 module | [`lm2596/`](lm2596/) | 일반 module STEP·치수 자료 | Reference only |
| EBIMU-9DOFV6 | [`ebimu_9dofv6/`](ebimu_9dofv6/) | 공식 specification PDF | Drawing only |
| MSP4030 | [`msp4030/`](msp4030/) | module 치수도·회로도·Altium LCD package | Drawing only |

## 아직 자동 확보하지 못한 파일

- XIAO RP2350 완성 보드 STEP: Seeed 공식 문서가 가리키는
  [GrabCAD model](https://grabcad.com/library/seeed-studio-xiao-rp2350-1)은 로그인과
  CloudFront 접근 제한 때문에 자동 다운로드하지 못했습니다. 공식 DXF와 KiCad PCB는
  확보했습니다.
- ACS758LCB-100B-PFF-T STEP: Allegro 공식 ECAD button과
  [SnapMagic part page](https://www.snapeda.com/parts/ACS758LCB-100B-PFF-T/Allegro/view-part/)
  모두 계정 기반 다운로드입니다. datasheet의 PFF package drawing은 확보했습니다.
- 100 A shunt, LM2596 board와 ACS758 breakout의 정확한 완성품 STEP은 판매자별 PCB와
  단자 위치가 달라 주문 링크 또는 실물 치수 없이는 확정하지 않습니다.
- EBIMU-9DOFV6와 MSP4030 완성품 STEP은 제조사 공개 자료에서 찾지 못했습니다.

## 무결성

다운로드 당시 SHA-256은 [`SHA256SUMS`](SHA256SUMS)에 기록합니다. 이 해시는 외부
제조사의 서명이나 공급망 보증이 아니라, 이후 로컬 파일이 바뀌었는지 확인하기 위한
기준입니다. 외부 ZIP과 CAD 파일은 실행하지 않았고 archive listing과 file type만
확인했습니다.
