#!/usr/bin/env python3
"""Headless Backyard Brains SpikerBox -> Arduino EMG claw controller.

Protocol reference (not copied code):
https://docs.backyardbrains.com/software/spike-recorder/classic/usb-communication-guide/
Source reference: https://github.com/BackyardBrains/Spike-Recorder
Python 3.10+. Only pyserial and (for HID) hidapi are required.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Optional

BYB_VID = 0x2E73
HID_PIDS = (0x0001, 0x0002)
SERIAL_PIDS = (0x0004, 0x0006, 0x0007, 0x0009, 0x000D)
START_MSG = bytes.fromhex("FF FF 01 01 80 FF")
END_MSG = bytes.fromhex("FF FF 01 01 81 FF")
CONFIG_PATH = Path(__file__).resolve().parent / "calibration.json"


@dataclass(frozen=True)
class Hardware:
    name: str
    rate: int  # samples per second, per channel
    channels: int
    center: int  # centered digital value (typically 512 or 8192)
    scale: float


# Based on the classic firmware/software. The exact hardware identifier is
# read from the device; do not infer signal layout from a random serial port.
SERIAL_TYPES = {
    "HUMANSB": Hardware("HUMANSB", 5000, 2, 8192, 1.0),
    "UNIBOX": Hardware("UNIBOX", 42661, 2, 8192, 1.0),
    "NRNSBPRO": Hardware("NRNSBPRO", 10000, 2, 8192, 2.8),
    "MSBPCDC": Hardware("MSBPCDC", 10000, 2, 512, 30.0),
    "NSBPCDC": Hardware("NSBPCDC", 10000, 2, 512, 30.0),
    "HBSBPRO": Hardware("HBSBPRO", 1000, 2, 512, 30.0),
    "HBLEOSB": Hardware("HBLEOSB", 10000, 1, 512, 30.0),
    "HHIBOX": Hardware("HHIBOX", 10000, 1, 512, 30.0),
    "MUSCUSB1": Hardware("MUSCUSB1", 10000, 1, 512, 30.0),
    "MUSCLESS": Hardware("MUSCLESS", 10000, 1, 512, 30.0),
    "HEARTSS": Hardware("HEARTSS", 10000, 1, 512, 30.0),
    "PLANTSS": Hardware("PLANTSS", 10000, 1, 512, 30.0),
    "EXTCLAW": Hardware("EXTCLAW", 896, 3, 512, 30.0),
}


class MessageDemux:
    """Remove inline framed metadata without destroying neighboring samples."""

    def __init__(self) -> None:
        self.pending = bytearray()
        self.in_message = False

    def feed(self, chunk: bytes) -> tuple[bytes, list[str]]:
        self.pending.extend(chunk)
        raw = bytearray()
        messages: list[str] = []
        while self.pending:
            if self.in_message:
                i = self.pending.find(END_MSG)
                if i >= 0:
                    body = self.pending[:i].decode("ascii", "replace")
                    messages.extend(m for m in body.split(";") if ":" in m)
                    del self.pending[: i + len(END_MSG)]
                    self.in_message = False
                elif len(self.pending) > 1024:
                    # No unbounded memory growth on a corrupt USB stream.
                    self.pending.clear()
                    self.in_message = False
                else:
                    break
            else:
                i = self.pending.find(START_MSG)
                if i >= 0:
                    raw.extend(self.pending[:i])
                    del self.pending[: i + len(START_MSG)]
                    self.in_message = True
                else:
                    # A marker can be split across separate serial/HID reads.
                    keep = 0
                    for n in range(1, min(len(START_MSG), len(self.pending) + 1)):
                        if self.pending.endswith(START_MSG[:n]):
                            keep = n
                    end = len(self.pending) - keep
                    raw.extend(self.pending[:end])
                    del self.pending[:end]
                    break
        return bytes(raw), messages


class FrameDecoder:
    """Decode selected channel from 2-byte/14-bit BYB sample frames."""

    def __init__(self, hardware: Hardware, channel: int = 0) -> None:
        if channel < 0 or channel >= hardware.channels:
            raise ValueError(f"채널 {channel}은 이 장치에서 사용할 수 없습니다 (0~{hardware.channels-1})")
        self.hardware = hardware
        self.channel = channel
        self.buf = bytearray()

    def feed(self, payload: bytes) -> list[float]:
        self.buf.extend(payload)
        result: list[float] = []
        frame_size = self.hardware.channels * 2
        while len(self.buf) >= frame_size:
            start = next((i for i, x in enumerate(self.buf) if x & 0x80), -1)
            if start < 0:
                self.buf.clear()
                break
            if start:
                del self.buf[:start]
            if len(self.buf) < frame_size:
                break
            frame = self.buf[:frame_size]
            # Only the very first byte of a frame has bit 7 set.
            bad = next((j for j in range(1, frame_size) if frame[j] & 0x80), None)
            if bad is not None:
                del self.buf[:bad]
                continue
            offset = self.channel * 2
            digital = ((frame[offset] & 0x7F) << 7) | (frame[offset + 1] & 0x7F)
            result.append((digital - self.hardware.center) * self.hardware.scale)
            del self.buf[:frame_size]
        if len(self.buf) > frame_size:
            del self.buf[:-frame_size]
        return result


def decode_hid_report(data: bytes) -> bytes:
    """HID frame = report ID, length, up to 62 bytes of stream data."""
    if len(data) >= 3 and data[0] == 0 and data[1] == 0x3F:
        data = data[1:]  # Some backends expose a leading zero report-id byte.
    if len(data) < 2:
        return b""
    length = data[1]
    if not (0 <= length <= 62 and length <= len(data) - 2):
        return b""
    return data[2 : 2 + length]


def metadata_value(messages: list[str], name: str) -> Optional[str]:
    for message in messages:
        key, _, value = message.partition(":")
        if key.strip().upper() == name:
            return value.strip()
    return None


def find_serial_ports():
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise RuntimeError("pyserial이 필요합니다: python -m pip install pyserial") from exc
    return list(list_ports.comports())


def find_hid_devices():
    try:
        import hid
    except ImportError:
        return []
    return [d for d in hid.enumerate(BYB_VID, 0)
            if d.get("product_id") in HID_PIDS]


def list_devices() -> None:
    print("[Serial / COM]")
    try:
        ports = find_serial_ports()
        for p in ports:
            vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None and p.pid is not None else "unknown"
            print(f"  {p.device:<12} {vidpid}  {p.description}")
        if not ports:
            print("  (검색된 포트 없음)")
    except Exception as exc:
        print(f"  {exc}")
    print("[Backyard Brains HID]")
    devices = find_hid_devices()
    for i, d in enumerate(devices):
        print(f"  HID {i} : {d.get('vendor_id'):04X}:{d.get('product_id'):04X}  {d.get('product_string', '')}")
    if not devices:
        print("  (없음 또는 hidapi 미설치)")


class SerialSpikerBox:
    mode = "serial"

    def __init__(self, port: str, channel: int, baud: Optional[int] = None):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial을 설치해 주세요: pip install pyserial") from exc
        self.serial = None
        self.demux = MessageDemux()
        self.decoder: Optional[FrameDecoder] = None
        self.hardware: Hardware
        self.channel = channel
        bauds = [baud] if baud else [222222, 500000, 230400, 115200]
        errors: list[str] = []
        for speed in bauds:
            try:
                s = serial.Serial(port, speed, timeout=0.05, write_timeout=0.3)
                self.serial = s
                self.demux = MessageDemux()
                # Time for boards that reboot on serial open.
                time.sleep(0.5)
                deadline = time.monotonic() + 2.3
                next_probe = 0.0
                hw_type = None
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    if now >= next_probe:
                        s.write(b"b:;\n?:;\n")
                        next_probe = now + 0.45
                    data = s.read(min(4096, max(1, s.in_waiting)))
                    _, messages = self.demux.feed(data)
                    hw_type = metadata_value(messages, "HWT") or hw_type
                    if hw_type:
                        break
                if not hw_type:
                    errors.append(f"{speed}: HWT 응답 없음")
                    s.close()
                    self.serial = None
                    continue
                device_type = next((k for k in SERIAL_TYPES if k in hw_type.upper()), None)
                if device_type is None:
                    raise RuntimeError(f"지원하지 않는 SpikerBox 장치 식별값: {hw_type}")
                hardware = SERIAL_TYPES[device_type]
                self.hardware = hardware
                self.decoder = FrameDecoder(hardware, channel)
                # Classic app also uses this configuration command.
                s.write(f"conf s:{hardware.rate};c:{hardware.channels};\n".encode("ascii"))
                if device_type in ("MSBPCDC", "NSBPCDC", "NRNSBPRO"):
                    s.write(b"start:;\n")
                s.reset_input_buffer()
                self.demux = MessageDemux()
                print(f"입력 연결: {port} / {speed} baud / {device_type} / {hardware.rate}Hz / {hardware.channels}ch")
                return
            except Exception as exc:
                errors.append(f"{speed}: {exc}")
                if self.serial is not None:
                    self.serial.close()
                self.serial = None
                if "지원하지 않는" in str(exc) or "채널" in str(exc):
                    raise
        raise RuntimeError("SpikerBox 시리얼 장치를 식별할 수 없습니다. " + " | ".join(errors))

    def read_samples(self) -> list[float]:
        assert self.serial is not None and self.decoder is not None
        data = self.serial.read(min(8192, max(1, self.serial.in_waiting)))
        raw, messages = self.demux.feed(data)
        board = metadata_value(messages, "BRD")
        if board in ("1", "4", "5"):
            # An add-on changes the interleaved frame layout. Do not risk
            # interpreting another channel as the chosen EMG input.
            raise RuntimeError("시리얼 SpikerBox 확장보드가 감지됐습니다. "
                               "Lite 버전에서는 확장보드 없이 1~2채널로 사용해 주세요.")
        return self.decoder.feed(raw)

    def clear(self) -> None:
        assert self.serial is not None
        self.serial.reset_input_buffer()
        self.demux = MessageDemux()
        self.decoder = FrameDecoder(self.hardware, self.channel)

    def close(self) -> None:
        if self.serial is not None and self.serial.is_open:
            try:
                if self.hardware.name in ("MSBPCDC", "NSBPCDC", "NRNSBPRO"):
                    self.serial.write(b"h:;\n")
            finally:
                self.serial.close()


class HIDSpikerBox:
    mode = "hid"

    def __init__(self, channel: int, index: int = 0):
        try:
            import hid
        except ImportError as exc:
            raise RuntimeError("HID 입력에는 hidapi가 필요합니다: pip install hidapi") from exc
        devices = find_hid_devices()
        if not devices:
            raise RuntimeError("Backyard Brains 구형 HID 장치를 찾지 못했습니다 (VID 2E73 PID 0001/0002).")
        if not 0 <= index < len(devices):
            raise ValueError(f"--hid-index 는 0~{len(devices)-1}이어야 합니다")
        self.device = hid.device()
        self.device.open_path(devices[index]["path"])
        self.hardware = Hardware("BYB_HID_PRO", 10000, 2, 512, 62.0)
        self.channel = channel
        self.decoder = FrameDecoder(self.hardware, channel)
        self.demux = MessageDemux()
        try:
            # Equivalent to Classic HIDUsbManager startup (no firmware writes).
            for command in (b"?:;\n", b"max:;\n", b"board:;\n", b"start:;\n"):
                self._command(command)
                time.sleep(0.02)
            print(f"입력 연결: HID {index} / {devices[index].get('product_string', '')}")
        except BaseException:
            self.device.close()
            raise

    def _command(self, cmd: bytes) -> None:
        if len(cmd) > 62:
            raise ValueError("HID 메시지가 너무 깁니다")
        report = bytes([0x3F, 0x3E]) + cmd + bytes(62 - len(cmd))
        written = self.device.write(report)
        if written < len(report):
            raise OSError("HID 장치 명령 전송 실패")

    def read_samples(self) -> list[float]:
        report = bytes(self.device.read(64, 50))
        raw, messages = self.demux.feed(decode_hid_report(report))
        board = metadata_value(messages, "BRD")
        if board is not None:
            # Classic code: normal board 2ch/10k, expansion 3/4ch/5k.
            new_channels = 4 if board == "1" else 3 if board in ("4", "5") else 2
            new_rate = 5000 if new_channels > 2 else 10000
            if (new_channels, new_rate) != (self.hardware.channels, self.hardware.rate):
                self.hardware = replace(self.hardware, channels=new_channels, rate=new_rate)
                self.decoder = FrameDecoder(self.hardware, self.channel)
                # Initial board discovery is normal; runtime changes are checked
                # against the active envelope/calibration in the caller.
                return []
        return self.decoder.feed(raw)

    def clear(self) -> None:
        self.decoder = FrameDecoder(self.hardware, self.channel)
        self.demux = MessageDemux()
        # Drop queued reports from the interactive calibration pause.
        # Keep the drain bounded even if new reports continue to arrive.
        deadline = time.monotonic() + 0.2
        for _ in range(200):
            if time.monotonic() >= deadline or not self.device.read(64, 1):
                break

    def close(self) -> None:
        try:
            self._command(b"h:;\n")
        except Exception:
            pass
        self.device.close()


class EmgEnvelope:
    """Streaming 20-Hz high-pass + 80-ms RMS envelope, 50-ms updates."""

    def __init__(self, sample_rate: int, window_ms: int = 80, update_ms: int = 50):
        self.rate = sample_rate
        self.window_size = max(8, round(sample_rate * window_ms / 1000))
        self.update_size = max(1, round(sample_rate * update_ms / 1000))
        rc = 1.0 / (2.0 * math.pi * 20.0)
        dt = 1.0 / sample_rate
        self.hp_alpha = rc / (rc + dt)
        self.reset()

    def reset(self) -> None:
        self.prev_in = 0.0
        self.prev_hp = 0.0
        self.window: deque[float] = deque()
        self.power = 0.0
        self.count = 0

    def feed(self, samples: list[float]) -> list[float]:
        levels = []
        for x in samples:
            hp = self.hp_alpha * (self.prev_hp + x - self.prev_in)
            self.prev_hp, self.prev_in = hp, x
            square = hp * hp
            self.window.append(square)
            self.power += square
            if len(self.window) > self.window_size:
                self.power -= self.window.popleft()
            self.count += 1
            if self.count % self.update_size == 0 and len(self.window) == self.window_size:
                levels.append(math.sqrt(max(0.0, self.power / self.window_size)))
        return levels


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("데이터가 없습니다")
    i = int((len(ordered) - 1) * fraction)
    return ordered[i]


def collect_baseline(reader, processor: EmgEnvelope, seconds: float = 2.5) -> list[float]:
    reader.clear()
    processor.reset()
    levels: list[float] = []
    received = 0
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        samples = reader.read_samples()
        if reader.hardware.rate != processor.rate:
            raise RuntimeError("샘플링 설정이 바뀌었습니다. 프로그램을 다시 시작해 보정하세요.")
        received += len(samples)
        levels.extend(processor.feed(samples))
    if received < processor.rate or len(levels) < 8:
        raise RuntimeError("근전도 샘플이 너무 적습니다. 전극·USB 연결 및 입력 채널을 확인하세요.")
    return levels


def do_calibration(reader, processor: EmgEnvelope) -> tuple[float, float]:
    print("\n[보정] 집게는 아직 움직이지 않습니다.")
    input("1/2 팔과 손의 힘을 완전히 빼고 Enter → ")
    idle = collect_baseline(reader, processor)
    rest = statistics.median(idle)
    print(f"  이완 RMS = {rest:.2f}")
    input("2/2 팔에 편안하지만 뚜렷하게 힘을 주고 Enter → ")
    strong_levels = collect_baseline(reader, processor)
    strong = percentile(strong_levels, 0.75)
    print(f"  수축 RMS = {strong:.2f}")
    if not math.isfinite(rest) or not math.isfinite(strong) or strong - rest < max(1.0, rest * 0.5):
        raise RuntimeError("수축·이완 신호 차이가 부족합니다. 전극 위치와 채널을 점검하고 다시 보정하세요.")
    return rest, strong


def calibration_key(reader) -> dict:
    h = reader.hardware
    return {"mode": reader.mode, "type": h.name, "rate": h.rate, "channels": h.channels,
            "center": h.center, "channel": reader.channel, "version": 1}


def load_calibration(reader) -> Optional[tuple[float, float]]:
    try:
        obj = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        if obj.get("device") != calibration_key(reader):
            return None
        idle, strong = float(obj["idle"]), float(obj["strong"])
        if math.isfinite(idle) and math.isfinite(strong) and strong - idle >= max(1.0, idle * 0.5):
            return idle, strong
    except (FileNotFoundError, ValueError, KeyError, TypeError, OSError):
        pass
    return None


def save_calibration(reader, idle: float, strong: float) -> None:
    obj = {"device": calibration_key(reader), "idle": idle, "strong": strong}
    target = CONFIG_PATH.with_suffix(".tmp")
    target.write_text(json.dumps(obj, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    target.replace(CONFIG_PATH)


class ClawOutput:
    def __init__(self, port: Optional[str], dry_run: bool):
        self.serial = None
        self.dry_run = dry_run
        if dry_run:
            print("[드라이런] Arduino에 명령을 보내지 않습니다.")
            return
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial이 필요합니다: pip install pyserial") from exc
        assert port is not None
        self.serial = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.3)
        time.sleep(2.0)  # Arduino Uno/Nano may reset when COM port opens.
        self.send(0)
        print(f"집게 출력 연결: {port} (115200 baud)")

    def send(self, activation: int) -> None:
        activation = max(0, min(100, int(activation)))
        if self.serial:
            command = f"G,{activation}\n".encode("ascii")
            if self.serial.write(command) != len(command):
                raise OSError("Arduino 전송이 일부만 완료됐습니다")

    def close(self) -> None:
        if self.serial:
            try:
                self.send(0)
                self.serial.flush()
            except Exception:
                pass
            self.serial.close()


def choose_source(args):
    mode = args.input_mode
    port = args.input_port
    if mode == "auto":
        if port:
            mode = "serial"
        else:
            hid_candidates = find_hid_devices()
            port_candidates = [p for p in find_serial_ports()
                               if p.vid == BYB_VID and p.pid in SERIAL_PIDS and p.device != args.output_port]
            if len(hid_candidates) + len(port_candidates) != 1:
                raise RuntimeError("입력 장치를 하나로 결정할 수 없습니다. --list 확인 후 "
                                   "--input-port COMx 또는 --input-mode hid를 지정하세요.")
            if hid_candidates:
                mode = "hid"
            else:
                port = port_candidates[0].device
                mode = "serial"
    if mode == "serial":
        if not port:
            raise RuntimeError("시리얼 장치는 --input-port COMx 지정이 필요합니다.")
        if port.casefold() == str(args.output_port).casefold():
            raise RuntimeError("SpikerBox 입력 포트와 Arduino 출력 포트가 같으면 안 됩니다.")
        return SerialSpikerBox(port, args.channel, args.baud)
    return HIDSpikerBox(args.channel, args.hid_index)


def live_loop(reader, processor: EmgEnvelope, output: ClawOutput, idle: float, strong: float,
              quiet: bool = False) -> None:
    input("\n집게에서 손을 떼고, 준비되면 Enter를 누르세요 → ")
    reader.clear()
    processor.reset()
    output.send(0)
    live_start = time.monotonic()
    last_real_sample = live_start
    last_send = 0.0
    last_log = 0.0
    value = 0.0
    smoothed = None
    print("제어 중... Ctrl+C로 종료 (통신 끊기면 자동 열림)")
    while True:
        samples = reader.read_samples()
        if reader.hardware.rate != processor.rate:
            raise RuntimeError("샘플링 설정이 바뀌었습니다. 재보정이 필요합니다.")
        now = time.monotonic()
        if samples:
            last_real_sample = now
            for level in processor.feed(samples):
                if smoothed is None:
                    smoothed = level
                else:
                    smoothed = 0.65 * smoothed + 0.35 * level
                value = min(1.0, max(0.0, (smoothed - idle) / (strong - idle)))
        stale = now - last_real_sample > 0.25
        if now - last_send >= 0.05:
            # Do not move the claw during the initial DSP settling period.
            command = 0 if stale or smoothed is None or now - live_start < 0.3 else round(value * 100)
            output.send(command)
            last_send = now
        if not quiet and now - last_log >= 0.4:
            display = 0 if stale else round(value * 100)
            print(f"\rEMG RMS {smoothed or 0:8.2f} | 활성도 {display:3d}% | "
                  f"{'신호 대기' if stale else '수신 중'}       ", end="", flush=True)
            last_log = now
        if now - last_real_sample > 3.0:
            raise RuntimeError("3초 동안 유효한 EMG 샘플을 받지 못했습니다. 제어를 중단합니다.")


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="BYB SpikerBox → EMG → Arduino 집게 (UI 없음)")
    parser.add_argument("--list", action="store_true", help="연결된 COM/HID 장치 나열")
    parser.add_argument("--input-mode", choices=("auto", "serial", "hid"), default="auto")
    parser.add_argument("--input-port", help="SpikerBox COM 포트 (예: COM5)")
    parser.add_argument("--output-port", help="Arduino COM 포트 (예: COM7)")
    parser.add_argument("--hid-index", type=int, default=0, help="HID 장치 목록 번호")
    parser.add_argument("--baud", type=int, help="SpikerBox 시리얼 통신 속도 고정 (기본: 자동 시도)")
    parser.add_argument("--channel", type=int, default=0, help="EMG 채널 번호 (0부터 시작)")
    parser.add_argument("--calibrate", action="store_true", help="저장된 보정을 무시하고 다시 측정")
    parser.add_argument("--dry-run", action="store_true", help="Arduino 없이 EMG 입력만 모니터링")
    parser.add_argument("--servo-test", action="store_true", help="SpikerBox 없이 서보 연결만 테스트")
    parser.add_argument("--quiet", action="store_true", help="실시간 수치 표시 숨기기")
    args = parser.parse_args(argv)
    if args.list:
        list_devices()
        return 0
    if args.servo_test:
        if not args.output_port:
            parser.error("서보 테스트에는 --output-port COMx가 필요합니다")
        port = None
        try:
            input("서보모터를 집게에서 분리하고 안전한 상태로 두었나요? Enter → ")
            port = ClawOutput(args.output_port, False)
            print("서보 닫기 2초 → 열기")
            for _ in range(40):
                port.send(80)
                time.sleep(0.05)
            port.send(0)
            time.sleep(0.7)
            return 0
        except (KeyboardInterrupt, Exception) as exc:
            print(f"서보 테스트 중단: {exc}", file=sys.stderr)
            return 1
        finally:
            if port is not None:
                port.close()
    if not args.dry_run and not args.output_port:
        parser.error("제어 시 --output-port COMx가 필요합니다. 테스트만 하려면 --dry-run을 사용하세요.")
    if args.channel < 0:
        parser.error("--channel은 0 이상이어야 합니다")
    reader = output = None
    try:
        # Open Arduino first. Keep it safely open throughout calibration.
        output = ClawOutput(args.output_port, args.dry_run)
        reader = choose_source(args)
        # HID expansion metadata may arrive after streaming starts.
        warm_deadline = time.monotonic() + 0.6
        while time.monotonic() < warm_deadline:
            reader.read_samples()
        processor = EmgEnvelope(reader.hardware.rate)
        calibration = None if args.calibrate else load_calibration(reader)
        if calibration is None:
            idle, strong = do_calibration(reader, processor)
            save_calibration(reader, idle, strong)
            print(f"보정 저장: 이완 {idle:.2f}, 수축 {strong:.2f}")
        else:
            idle, strong = calibration
            print(f"저장된 보정 사용: 이완 {idle:.2f}, 수축 {strong:.2f}")
        live_loop(reader, processor, output, idle, strong, args.quiet)
    except KeyboardInterrupt:
        print("\n사용자 요청으로 제어 종료")
    except Exception as exc:
        print(f"\n오류: {exc}", file=sys.stderr)
        return 1
    finally:
        # Best effort release; Arduino's independent 300ms watchdog is the fallback.
        if output is not None:
            output.close()
        if reader is not None:
            reader.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
