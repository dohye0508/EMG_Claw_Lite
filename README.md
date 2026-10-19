# EMG Claw Lite — UI 없는 SpikerBox 근전도 집게 컨트롤러

Windows 10/11 + Python 3.10+ 기준. Linux/macOS의 시리얼 장치도 `pyserial` 지원 범위에서 동작 가능하지만 실제 장비 검증은 하지 않았습니다.

**목적:** Backyard Brains SpikerBox USB → EMG 근육 활성도(0–100%) → USB 시리얼 → Arduino Uno/Nano → 서보모터 집게.
기존 Spike Recorder GUI, BASS, SDL, FFT, 녹음 기능은 필요 없습니다. 공통 USB 프레임/메시지 프로토콜을 필요한 만큼 독립 재구현했습니다.

## 폴더

- `emg_claw.py`: USB CDC/시리얼 + 구형 USB HID 신호 수신, EMG RMS, 보정, Arduino 송신
- `ArduinoClaw/ArduinoClaw.ino`: Arduino Servo 구동, 히스테리시스 및 300ms 통신 타임아웃
- `requirements.txt`: pyserial / hidapi
- `tests/test_emg_claw.py`: 장치 없이 실행되는 신호·프로토콜 테스트
- `calibration.json`: 첫 실행 시 보정하면 **자동 생성**됨. 센서/전극을 바꾸면 `--calibrate` 권장

## 준비

1. Python 3.10 이상 설치(Windows에서 설치 시 Add Python to PATH 체크).
2. 명령 프롬프트에서 프로그램 폴더로 이동:

   ```bat
   cd EMG_Claw_Lite
   py -m pip install -r requirements.txt
   py emg_claw.py --list
   ```

3. 출력 Arduino Uno/Nano에 Arduino IDE로 `ArduinoClaw/ArduinoClaw.ino`를 업로드합니다. 업로드 뒤 **시리얼 모니터를 닫아야** 프로그램이 COM 포트를 열 수 있습니다.
4. 서보 모터의 신호선은 D9, 전원은 모터 모델에 적합한 **별도 전원**, GND는 Arduino와 공통 연결합니다. 반드시 무부하로 열림/닫힘 각도를 먼저 조절하세요. **Arduino 5V 핀에 고전류 서보를 직접 연결하지 마세요.**
5. SpikerBox(입력)와 Arduino(출력)를 각각 USB로 PC에 연결합니다. SpikerBox와 Arduino는 별도의 COM 포트 또는 HID 인터페이스입니다.
6. EMG 피부 전극은 **상용 SpikerBox 입력에만** 연결하고, 모터 전원이나 Arduino 전선에 연결하지 마세요. 피부 접촉 장치는 제조사 안전 지침에 맞춰 사용하며, 비상 해제를 위해 모터 전원 차단 스위치를 두는 것이 좋습니다.

## 실행 예시

### 연결된 USB 장치를 확인

```bat
py emg_claw.py --list
```

예를 들어 SpikerBox가 `COM5`, Arduino가 `COM7`이라면:

```bat
py emg_claw.py --input-mode serial --input-port COM5 --output-port COM7
```

구형 HID SpikerBox Pro(`VID 2E73:PID 0001` 또는 `0002`)는:

```bat
py emg_claw.py --input-mode hid --output-port COM7
```

BYB 장치가 하나만 연결되고 장치 ID로 발견된다면 자동 선택도 가능합니다:

```bat
py emg_claw.py --output-port COM7
```

**SpikerBox 없이 Arduino/서보 배선부터 시험하려면** (서보를 집게에서 분리한 상태에서):

```bat
py emg_claw.py --servo-test --output-port COM7
```

**실제 Arduino 없이 EMG만 먼저 시험하려면:**

```bat
py emg_claw.py --input-mode serial --input-port COM5 --dry-run
```

**보정을 새로 하고 싶다면:**

```bat
py emg_claw.py --input-mode serial --input-port COM5 --output-port COM7 --calibrate
```

다른 채널(두 번째 채널)을 사용한다면 `--channel 1`을 추가합니다. 기본값은 첫 번째 채널(`--channel 0`)입니다. 처음 사용하는 SpikerBox는 실행 중 콘솔에서 2회 Enter를 눌러 **이완 → 편안한 강한 수축**을 측정합니다. 성공하면 `calibration.json`에 저장됩니다. 그 뒤 준비 확인 Enter로 실제 제어를 시작합니다. 종료: `Ctrl+C`.

명령 메시지는 `G,0\n`에서 `G,100\n`까지의 ASCII 문자열입니다. Arduino는 45 이상이면 집게를 닫고 25 이하이면 열며, 26–44 구간에서는 현재 상태를 유지합니다. 이는 서보의 실제 힘을 직접 측정하는 것이 아니라 **EMG 상대 활성도에 의한 개폐 제어**입니다. 닫힘 각도 및 실제 쥐는 힘은 집게 기구에 따라 달라집니다.

## 동작 및 안전

- 입력: BYB HID 2채널/10 kHz(확장보드는 BRD 메시지에 따라 3/4채널 5 kHz), BYB CDC/시리얼은 기기가 보내는 `HWT` 식별자를 사용.
- 디코딩: 2바이트/프레임/채널의 7비트 묶음과 메시지 escape sequence를 분리. 기본 형식 `HWT:HUMANSB;` 등.
- 신호: 20 Hz 1차 고역통과 필터 → 80ms 이동 RMS → 50ms 업데이트 → RMS 지수평활 → 사용자 보정 → 0–100%.
- Arduino: 115200 baud, PWM 서보 D9, 출력 히스테리시스 (닫기 ≥45%, 열기 ≤25%).
- **자동 해제:** PC가 250ms 이상 EMG 데이터를 못 받으면 `G,0`을 전송. USB 신호가 3초 동안 없으면 오류와 함께 종료. 독립적인 Arduino 300ms 패킷 타임아웃에서도 집게를 엽니다. **PC나 Arduino의 전체 전원이 끊어졌을 때 집게가 물리적으로 열린다는 보장은 없습니다.** 안전이 중요한 기구에는 복귀 스프링·기계적 안전장치가 필요합니다.
- 애초에 잘못된 센서 신호, 전극 탈락으로 인한 잡음, 잘못된 보정이 있으면 의도치 않게 닫힐 수 있습니다. 본 코드는 의료용·산업 안전용이 아닙니다.
- Arduino와 SpikerBox는 서로 다른 포트여야 합니다. 같은 포트는 사용 불가.
- 프로그램은 장치의 펌웨어를 변경하지 않습니다.

## 기기 지원 범위/제약

이 프로젝트는 **기존 Spike Recorder 프로그램 전체를 수정한 것이 아니라, 필요한 USB 통신 규약과 EMG 처리만 새로 구현한 코드**입니다. 공식 USB 통신 문서에 나오는 여러 세대의 SpikerBox를 대상으로 했지만, **실물 장비에서 연결·샘플 품질·반응 속도는 아직 확인하지 못했습니다.** 장치가 `HWT`를 응답하지 않거나 특정 펌웨어에서 프로토콜이 다르면 바로 동작하지 않을 수 있습니다. 아래 확인이 우선입니다.

- HID: 구형 Muscle/Neuron SpikerBox Pro (VID `2E73`, PID `0001`/`0002`). 다른 USB VID/PID를 갖는 제품이 모두 HID인 것은 아닙니다.
- CDC/시리얼: Human SpikerBox (`2E73:0004`), 신형 Muscle Pro (`2E73:0006`), Neuron Pro (`2E73:0007`), 관련 제품. HWT 메시지와 장치 설정으로 식별합니다. VID `0403:6015`처럼 다른 제품과 구분이 어려운 구형 장치는 `--input-port`를 직접 지정합니다.
- 사용한 장치별 디지털 중심값(512 또는 8192)과 스케일은 기존 Classic Spike Recorder 코드에서 확인했습니다. 일부 장치의 샘플 속도는 펌웨어 버전에 따라 달라질 수 있어 초기 반응속도 및 보정 결과를 검증해야 합니다.
- 특정 장치/드라이버에서 BAUD 지원이 안 되는 경우 `--baud 500000` 또는 `--baud 222222`를 명시할 수 있습니다.
- 센서의 RMS는 절대 힘(N)이 아닌 **상대적인 근활성도**입니다. 진짜 잡는 힘 제어는 별도 힘 센서와 폐회로 제어가 필요합니다.

## 오프라인 테스트

```bat
py -m unittest discover -s tests -v
```

실물 장치 없이 프레임 파싱/메타데이터 분리, 다채널 복원, EMG RMS, 보정, Arduino 출력 문자열 등을 검증합니다. **컴퓨터에서 실물 장치를 구동하는 통합 테스트는 아닙니다.**

## 근거 자료

- BYB USB Communication Guide: https://docs.backyardbrains.com/software/spike-recorder/classic/usb-communication-guide/
- 원본 Spike Recorder: https://github.com/BackyardBrains/Spike-Recorder
- 프로토콜/메시지: `src/engine/ArduinoSerial.cpp`, `src/engine/HIDUsbManager.cpp`
- Arduino Servo: https://docs.arduino.cc/libraries/servo/

## 인형뽑기 집게의 종류

현재 제공되는 Arduino 스케치는 **PWM RC 서보모터**를 회전시켜 여닫는 작은 기계식 집게를 기준으로 작성했습니다. 실제 아케이드 인형뽑기 장치는 솔레노이드/전자석이나 DC 모터로 파지력을 제어하는 모델도 있습니다. 그런 집게에는 위 서보 코드를 그대로 연결하면 안 되며, 별도의 모터 드라이버·전원·전류 제한과 기구 안전 설계가 필요합니다. 먼저 별도 저전력 서보 집게로 동작을 검증한 뒤 실제 집게 기구 사양에 맞춰 출력단을 변경하세요.

**검증 순서:** (1) `--dry-run`으로 EMG 수치만 확인 → (2) 집게를 떼고 Arduino의 서보 모터만 제어 → (3) 손이 끼지 않는 낮은 힘의 집게를 연결.

현재 시리얼 입력에서 `BRD:1/4/5` 확장보드 메시지가 검출되면 잘못된 채널을 근전도로 해석하지 않도록 **제어를 중단**합니다. 확장보드 없이 기본 채널을 사용하거나 확장보드 규격에 맞춘 추가 구현이 필요합니다. 구형 HID 입력은 Classic 소스에서 확인한 `BRD`별 2/3/4채널 전환을 구현했습니다.
