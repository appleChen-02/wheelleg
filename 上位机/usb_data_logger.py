#!/usr/bin/env python3
"""
USB data logger for StandardRobotpp.

Features:
- Read CDC serial data from MCU.
- Parse frame format: [SOF=0x5A][len][id][crc8][payload][crc16].
- Verify CRC8/CRC16 exactly as firmware.
- Decode known packet IDs:
  - 0x02: IMU
  - 0x08: RobotMotion
    - 0x0B: RobotTarget
- Save CSV files directly readable by MATLAB.
- Optional MAT export on exit (requires scipy).

实时打印说明：
    使用 `--realtime` 开关后，脚本在每帧解码完成时会把该帧的关键字段打印到终端，便于调试和观察：
    - IMU: 时间、tick、yaw/pitch/roll（rad）、yaw_vel/pitch_vel/roll_vel（rad/s）。
    - MOTION: 时间、tick、vx、vy、wz、body_roll/pitch/yaw 等主要运动量。
    - TARGET: 与 MOTION 格式相同，为目标快照。
    使用 `--realtime-mode` 选择输出方式：
    - append: 逐行追加输出（默认）。
    - latest: 在终端底部维护“最新快照区”，每次只更新该区域，不清空历史输出。

示例（中文）：
    #通常使用
    py usb_data_logger.py --port COM10 --baud 115200 --realtime --realtime-mode latest --save-mat --outdir ./log
    # 自动选择串口并实时在终端打印解码后的帧
    python usb_data_logger.py --realtime --baud 115200 --outdir ./log

    # 更新“最新快照区”，不清空历史输出
    python usb_data_logger.py --realtime --realtime-mode latest --baud 115200 --outdir ./log

    # 指定端口（Linux 示例）
    python usb_data_logger.py --port /dev/ttyACM0 --baud 115200 --outdir ./log

    # 指定端口（Windows 示例），运行 60 秒并在退出时导出 MAT 文件
    python usb_data_logger.py --port COM3 --baud 115200 --duration 60 --save-mat

    # 列出可用串口并退出
    python usb_data_logger.py --list-ports

    # 增大每次读取缓冲并减少磁盘刷新频率
    python usb_data_logger.py --port COM3 --baud 115200 --read-size 1024 --flush-interval 5

    # 快速捕获（自动选择串口，使用默认波特率）
    python usb_data_logger.py --outdir ./quick_log

Examples (English):
    # Auto-pick a serial port and print decoded frames to terminal in realtime
    python usb_data_logger.py --realtime --baud 115200 --outdir ./log

    # Update the latest snapshot block without clearing history
    python usb_data_logger.py --realtime --realtime-mode latest --baud 115200 --outdir ./log

    # Specify a port explicitly (Linux example)
    python usb_data_logger.py --port /dev/ttyACM0 --baud 115200 --outdir ./log

    # Specify a port explicitly (Windows example) and run for 60s, export MAT on exit
    python usb_data_logger.py --port COM3 --baud 115200 --duration 60 --save-mat

    # List available serial ports and exit
    python usb_data_logger.py --list-ports

    # Increase read buffer and reduce flush frequency
    python usb_data_logger.py --port COM3 --baud 115200 --read-size 1024 --flush-interval 5

    # Minimal quick capture (auto-port, default baud)
    python usb_data_logger.py --outdir ./quick_log
"""

from __future__ import annotations

import argparse
import csv
import os
import struct
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except Exception as exc:
    raise SystemExit(
        "pyserial is required. Install with: pip install pyserial\n"
        f"Import error: {exc}"
    )

SOF = 0x5A

# 下位机当前常用的上行数据包 ID。
# 如果下位机协议以后增加新包，只需要在这里补一个分支并同步 MATLAB 侧即可。
ID_IMU = 0x02
ID_ROBOT_MOTION = 0x08
ID_ROBOT_TARGET = 0x0B
ID_TORQUE = 0x0C

# 机器人运动/目标快照 payload: <I21f> (tick_ms + 21 floats)
ROBOT_SNAPSHOT_STRUCT = struct.Struct("<I21f")
ROBOT_SNAPSHOT_COLUMNS = [
    "tick_ms",
    "vx",
    "vy",
    "wz",
    "tail_beta",
    "tail_beta_dot",
    "body_roll",
    "body_pitch",
    "body_yaw",
    "body_x",
    "leg0_phi",
    "leg0_phi_dot",
    "leg0_legx",
    "leg0_legx_dot",
    "leg0_theta",
    "leg0_theta_dot",
    "leg1_phi",
    "leg1_phi_dot",
    "leg1_legx",
    "leg1_legx_dot",
    "leg1_theta",
    "leg1_theta_dot",
]

# 控制力矩 payload: <I5f> (tick_ms + 5 floats)
TORQUE_STRUCT = struct.Struct("<I5f")
TORQUE_COLUMNS = [
    "tick_ms",
    "T_r_to_b",
    "T_l_to_b",
    "T_wr_to_r",
    "T_wl_to_l",
    "T_t_to_b",
]

CRC8_INIT = 0xFF
# CRC8 表直接对齐下位机固件中的查表法实现，保证 Python 端和 MCU 端校验结果一致。
CRC8_TABLE = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
]


@dataclass
class Stats:
    ok_frames: int = 0
    bad_crc8: int = 0
    bad_crc16: int = 0
    unknown_id: int = 0


def crc8(data: bytes, init: int = CRC8_INIT) -> int:
    crc = init
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc & 0xFF


def crc16(data: bytes, init: int = 0xFFFF) -> int:
    # 与固件使用同一类 CRC16/IBM-SDLC 变体，最终字节序为低字节在前。
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def verify_header_crc8(header4: bytes) -> bool:
    return len(header4) == 4 and crc8(header4[:3]) == header4[3]


def verify_frame_crc16(frame: bytes) -> bool:
    if len(frame) < 6:
        return False
    expected = frame[-2] | (frame[-1] << 8)
    actual = crc16(frame[:-2])
    return actual == expected


class CsvSink:
    def __init__(self, outdir: str) -> None:
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir

        # 每个数据包单独落盘，MATLAB 读取时最省事，也便于单独查看某一路数据。
        self.imu_file = open(os.path.join(outdir, "imu.csv"), "w", newline="", encoding="utf-8")
        self.motion_file = open(
            os.path.join(outdir, "robot_motion.csv"), "w", newline="", encoding="utf-8"
        )
        self.target_file = open(
            os.path.join(outdir, "robot_target.csv"), "w", newline="", encoding="utf-8"
        )
        self.torque_file = open(
            os.path.join(outdir, "torque.csv"), "w", newline="", encoding="utf-8"
        )
        self.unknown_file = open(
            os.path.join(outdir, "unknown_frames.csv"), "w", newline="", encoding="utf-8"
        )

        self.imu_writer = csv.writer(self.imu_file)
        self.motion_writer = csv.writer(self.motion_file)
        self.target_writer = csv.writer(self.target_file)
        self.torque_writer = csv.writer(self.torque_file)
        self.unknown_writer = csv.writer(self.unknown_file)

        self.imu_writer.writerow(
            ["host_time_s", "tick_ms", "yaw", "pitch", "roll", "yaw_vel", "pitch_vel", "roll_vel"]
        )
        self.motion_writer.writerow(["host_time_s"] + ROBOT_SNAPSHOT_COLUMNS)
        self.target_writer.writerow(["host_time_s"] + ROBOT_SNAPSHOT_COLUMNS)
        self.torque_writer.writerow(["host_time_s"] + TORQUE_COLUMNS)
        self.unknown_writer.writerow(["host_time_s", "id_hex", "payload_len", "frame_hex"])

        self.imu_rows: List[List[float]] = []
        self.motion_rows: List[List[float]] = []
        self.target_rows: List[List[float]] = []
        self.torque_rows: List[List[float]] = []

    def write_imu(self, host_t: float, row: List[float]) -> None:
        line = [host_t] + row
        self.imu_writer.writerow(line)
        self.imu_rows.append(line)

    def write_motion(self, host_t: float, row: List[float]) -> None:
        line = [host_t] + row
        self.motion_writer.writerow(line)
        self.motion_rows.append(line)

    def write_target(self, host_t: float, row: List[float]) -> None:
        line = [host_t] + row
        self.target_writer.writerow(line)
        self.target_rows.append(line)

    def write_torque(self, host_t: float, row: List[float]) -> None:
        line = [host_t] + row
        self.torque_writer.writerow(line)
        self.torque_rows.append(line)

    def write_unknown(self, host_t: float, pkt_id: int, payload_len: int, frame_hex: str) -> None:
        self.unknown_writer.writerow([host_t, f"0x{pkt_id:02X}", payload_len, frame_hex])

    def flush(self) -> None:
        self.imu_file.flush()
        self.motion_file.flush()
        self.target_file.flush()
        self.torque_file.flush()
        self.unknown_file.flush()

    def close(self) -> None:
        self.flush()
        self.imu_file.close()
        self.motion_file.close()
        self.target_file.close()
        self.torque_file.close()
        self.unknown_file.close()


class FrameParser:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.stats = Stats()

    def feed(self, data: bytes) -> List[bytes]:
        # 累积串口分片数据，支持半包、粘包以及噪声字节。
        self.buf.extend(data)
        out: List[bytes] = []

        while True:
            sof_idx = self.buf.find(bytes([SOF]))
            if sof_idx < 0:
                self.buf.clear()
                break
            if sof_idx > 0:
                del self.buf[:sof_idx]

            if len(self.buf) < 4:
                break

            # 先校验帧头 CRC8，再根据 len 判断整帧是否到齐。
            header = bytes(self.buf[:4])
            if not verify_header_crc8(header):
                self.stats.bad_crc8 += 1
                del self.buf[0]
                continue

            payload_len = header[1]
            frame_len = 4 + payload_len + 2

            # 防御性上限，避免错误 len 把解析器卡住。
            if payload_len > 200:
                self.stats.bad_crc8 += 1
                del self.buf[0]
                continue

            if len(self.buf) < frame_len:
                break

            frame = bytes(self.buf[:frame_len])
            if not verify_frame_crc16(frame):
                self.stats.bad_crc16 += 1
                del self.buf[0]
                continue

            out.append(frame)
            self.stats.ok_frames += 1
            del self.buf[:frame_len]

        return out


def decode_frame(frame: bytes) -> Optional[Dict[str, object]]:
    host_t = time.time()
    pkt_id = frame[2]
    payload = frame[4:-2]

    # 下面的 struct 格式要和 usb_typdef.h 里的 packed 结构严格对应。
    if pkt_id == ID_IMU and len(payload) == 28:
        tick_ms, yaw, pitch, roll, yaw_vel, pitch_vel, roll_vel = struct.unpack("<I6f", payload)
        return {
            "type": "imu",
            "host_time": host_t,
            "row": [tick_ms, yaw, pitch, roll, yaw_vel, pitch_vel, roll_vel],
        }

    if pkt_id == ID_ROBOT_MOTION and len(payload) == ROBOT_SNAPSHOT_STRUCT.size:
        row = list(ROBOT_SNAPSHOT_STRUCT.unpack(payload))
        return {
            "type": "motion",
            "host_time": host_t,
            "row": row,
        }

    if pkt_id == ID_ROBOT_TARGET and len(payload) == ROBOT_SNAPSHOT_STRUCT.size:
        row = list(ROBOT_SNAPSHOT_STRUCT.unpack(payload))
        return {
            "type": "target",
            "host_time": host_t,
            "row": row,
        }

    if pkt_id == ID_TORQUE and len(payload) == TORQUE_STRUCT.size:
        row = list(TORQUE_STRUCT.unpack(payload))
        return {
            "type": "torque",
            "host_time": host_t,
            "row": row,
        }

    return {
        "type": "unknown",
        "host_time": host_t,
        "id": pkt_id,
        "payload_len": len(payload),
        "frame_hex": frame.hex(),
    }


def ansi_supported() -> bool:
    if os.name != "nt":
        return True
    try:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(-11)
        mode = ctypes.c_uint32()
        if kernel32.GetConsoleMode(handle, ctypes.byref(mode)) == 0:
            return False
        new_mode = mode.value | 0x0004
        if kernel32.SetConsoleMode(handle, new_mode) == 0:
            return False
        return True
    except Exception:
        return False


def realtime_emit_append(message: str, args: argparse.Namespace) -> None:
    if not getattr(args, "realtime", False):
        return
    if getattr(args, "realtime_mode", "append") != "append":
        return
    print(message, flush=True)


def format_imu_line(row: Optional[List[float]], host_t: Optional[float]) -> str:
    if not row or host_t is None:
        return "IMU   : <waiting>"
    ts = time.strftime("%H:%M:%S", time.localtime(host_t))
    return (
        f"IMU   : {ts} tick_ms={int(row[0])} "
        f"yaw={row[1]:.3f} pitch={row[2]:.3f} roll={row[3]:.3f} "
        f"yaw_vel={row[4]:.3f} pitch_vel={row[5]:.3f} roll_vel={row[6]:.3f}"
    )


def format_robot_line(prefix: str, row: Optional[List[float]], host_t: Optional[float]) -> str:
    if not row or host_t is None:
        return f"{prefix}: <waiting>"
    ts = time.strftime("%H:%M:%S", time.localtime(host_t))
    parts = [f"tick_ms={int(row[0])}"]
    for name, val in zip(ROBOT_SNAPSHOT_COLUMNS[1:], row[1:]):
        parts.append(f"{name}={val:.3f}")
    return f"{prefix}: {ts} " + " ".join(parts)


def format_torque_line(row: Optional[List[float]], host_t: Optional[float]) -> str:
    if not row or host_t is None:
        return "TORQUE: <waiting>"
    ts = time.strftime("%H:%M:%S", time.localtime(host_t))
    return (
        f"TORQUE: {ts} tick_ms={int(row[0])} "
        f"Tr_to_b={row[1]:.3f} Tl_to_b={row[2]:.3f} "
        f"Twr_to_r={row[3]:.3f} Twl_to_l={row[4]:.3f} "
        f"Tt_to_b={row[5]:.3f}"
    )


class LatestSnapshotDisplay:
    def __init__(self, args: argparse.Namespace) -> None:
        self.enabled = bool(getattr(args, "realtime", False)) and (
            getattr(args, "realtime_mode", "append") == "latest"
        )
        self.ansi_ok = ansi_supported()
        self.lines_rendered = 0
        self.imu_row: Optional[List[float]] = None
        self.imu_host_t: Optional[float] = None
        self.motion_row: Optional[List[float]] = None
        self.motion_host_t: Optional[float] = None
        self.target_row: Optional[List[float]] = None
        self.target_host_t: Optional[float] = None
        self.torque_row: Optional[List[float]] = None
        self.torque_host_t: Optional[float] = None

    def update(self, kind: str, row: List[float], host_t: float) -> None:
        if not self.enabled:
            return
        if kind == "imu":
            self.imu_row = row
            self.imu_host_t = host_t
        elif kind == "motion":
            self.motion_row = row
            self.motion_host_t = host_t
        elif kind == "target":
            self.target_row = row
            self.target_host_t = host_t
        elif kind == "torque":
            self.torque_row = row
            self.torque_host_t = host_t
        else:
            return
        self.render()

    def render(self) -> None:
        lines = [
            "[LATEST] Latest snapshot (updates on new frames)",
            format_imu_line(self.imu_row, self.imu_host_t),
            format_robot_line("MOTION", self.motion_row, self.motion_host_t),
            format_robot_line("TARGET", self.target_row, self.target_host_t),
            format_torque_line(self.torque_row, self.torque_host_t),
        ]

        if not self.ansi_ok or self.lines_rendered == 0:
            for line in lines:
                print(line)
            self.lines_rendered = len(lines)
            return

        sys.stdout.write(f"\x1b[{self.lines_rendered}F")
        for line in lines:
            sys.stdout.write("\x1b[2K" + line + "\n")
        sys.stdout.flush()
        self.lines_rendered = len(lines)


def export_mat(outdir: str, sink: CsvSink) -> None:
    try:
        import numpy as np  # type: ignore
        from scipy.io import savemat  # type: ignore
    except Exception:
        print("[WARN] scipy or numpy not available, skip MAT export.")
        return

    # 这里导出的是“数值矩阵”，MATLAB 读取后可以直接用于绘图和滤波处理。
    mat_data = {
        "imu": np.array(sink.imu_rows, dtype=float) if sink.imu_rows else np.empty((0, 8), dtype=float),
        "robot_motion": np.array(sink.motion_rows, dtype=float)
        if sink.motion_rows
        else np.empty((0, 23), dtype=float),
        "robot_target": np.array(sink.target_rows, dtype=float)
        if sink.target_rows
        else np.empty((0, 23), dtype=float),
        "torque": np.array(sink.torque_rows, dtype=float)
        if sink.torque_rows
        else np.empty((0, 7), dtype=float),
    }

    mat_path = os.path.join(outdir, "usb_log.mat")
    savemat(mat_path, mat_data)
    print(f"[INFO] Saved MAT: {mat_path}")


def list_serial_port_names() -> List[str]:
    names = [p.device for p in list_ports.comports()]
    names.sort()
    return names


def auto_pick_port() -> str:
    ports = list_serial_port_names()
    if not ports:
        raise SystemExit(
            "No serial port found. On Ubuntu, check USB cable and permission (dialout group)."
        )

    preferred_patterns = ["/dev/ttyACM", "/dev/ttyUSB", "COM"]
    for pattern in preferred_patterns:
        for p in ports:
            if p.startswith(pattern):
                print(f"[INFO] Auto selected port: {p}")
                return p

    print(f"[INFO] Auto selected port: {ports[0]}")
    return ports[0]


def run(args: argparse.Namespace) -> int:
    sink = CsvSink(args.outdir)
    parser = FrameParser()
    latest_display = LatestSnapshotDisplay(args)

    print(f"[INFO] Open serial: {args.port}, baud={args.baud}")
    print(f"[INFO] Output dir : {args.outdir}")

    t0 = time.time()
    next_flush_t = t0 + max(float(args.flush_interval), 0.0)

    try:
        with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
            while True:
                # 每轮先读取一段字节，再交给帧解析器做状态机式拆包。
                chunk = ser.read(args.read_size)
                if chunk:
                    frames = parser.feed(chunk)
                    for fr in frames:
                        decoded = decode_frame(fr)
                        if decoded is None:
                            continue

                        kind = decoded["type"]
                        host_t = float(decoded["host_time"])
                        if kind == "imu":
                            row = list(decoded["row"])
                            sink.write_imu(host_t, row)
                            if getattr(args, "realtime", False):
                                ts = time.strftime("%H:%M:%S", time.localtime(host_t))
                                message = (
                                    f"[IMU] {ts} tick={int(row[0])} "
                                    f"yaw={row[1]:.3f} pitch={row[2]:.3f} roll={row[3]:.3f} "
                                    f"yaw_vel={row[4]:.3f} pitch_vel={row[5]:.3f} roll_vel={row[6]:.3f}"
                                )
                                realtime_emit_append(message, args)
                                latest_display.update("imu", row, host_t)
                        elif kind == "motion":
                            row = list(decoded["row"])
                            sink.write_motion(host_t, row)
                            if getattr(args, "realtime", False):
                                ts = time.strftime("%H:%M:%S", time.localtime(host_t))
                                message = (
                                    f"[MOTION] {ts} tick={int(row[0])} vx={row[1]:.3f} vy={row[2]:.3f} wz={row[3]:.3f} "
                                    f"roll={row[6]:.3f} pitch={row[7]:.3f} yaw={row[8]:.3f}"
                                )
                                realtime_emit_append(message, args)
                                latest_display.update("motion", row, host_t)
                        elif kind == "target":
                            row = list(decoded["row"])
                            sink.write_target(host_t, row)
                            if getattr(args, "realtime", False):
                                ts = time.strftime("%H:%M:%S", time.localtime(host_t))
                                message = (
                                    f"[TARGET] {ts} tick={int(row[0])} vx={row[1]:.3f} vy={row[2]:.3f} wz={row[3]:.3f} "
                                    f"roll={row[6]:.3f} pitch={row[7]:.3f} yaw={row[8]:.3f}"
                                )
                                realtime_emit_append(message, args)
                                latest_display.update("target", row, host_t)
                        elif kind == "torque":
                            row = list(decoded["row"])
                            sink.write_torque(host_t, row)
                            if getattr(args, "realtime", False):
                                ts = time.strftime("%H:%M:%S", time.localtime(host_t))
                                message = (
                                    f"[TORQUE] {ts} tick={int(row[0])} "
                                    f"Tr={row[1]:.3f} Tl={row[2]:.3f} "
                                    f"Twr={row[3]:.3f} Twl={row[4]:.3f} "
                                    f"Tt={row[5]:.3f}"
                                )
                                realtime_emit_append(message, args)
                                latest_display.update("torque", row, host_t)
                        else:
                            parser.stats.unknown_id += 1
                            sink.write_unknown(
                                host_t,
                                int(decoded["id"]),
                                int(decoded["payload_len"]),
                                str(decoded["frame_hex"]),
                            )

                # 定时刷新到磁盘，避免每秒边界时高频重复 flush。
                if args.flush_interval > 0 and time.time() >= next_flush_t:
                    sink.flush()
                    next_flush_t += float(args.flush_interval)

                if args.duration > 0 and (time.time() - t0) >= args.duration:
                    print("[INFO] Reached duration limit, stopping.")
                    break

    except serial.SerialException as exc:
        print(f"[ERROR] Serial open/read failed: {exc}")
        if sys.platform.startswith("linux"):
            print("[HINT] Ubuntu可能需要串口权限: sudo usermod -a -G dialout $USER")
            print("[HINT] 重新登录后生效，或临时使用 sudo 运行。")
        return 1
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        sink.close()

    print(
        "[INFO] Stats: "
        f"ok={parser.stats.ok_frames}, "
        f"crc8_err={parser.stats.bad_crc8}, "
        f"crc16_err={parser.stats.bad_crc16}, "
        f"unknown_id={parser.stats.unknown_id}"
    )

    if args.save_mat:
        export_mat(args.outdir, sink)

    return 0


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="USB data logger for MCU frames")
    ap.add_argument(
        "--port",
        default=None,
        help="Serial port, e.g. /dev/ttyACM0 (Linux) or COM6 (Windows). If omitted, auto-detect.",
    )
    ap.add_argument("--list-ports", action="store_true", help="List local serial ports and exit")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    ap.add_argument("--timeout", type=float, default=0.02, help="Serial read timeout in seconds")
    ap.add_argument("--read-size", type=int, default=512, help="Bytes read each cycle")
    ap.add_argument("--outdir", default="usb_log", help="Output directory")
    ap.add_argument("--duration", type=float, default=0.0, help="Stop after N seconds, 0 means run forever")
    ap.add_argument("--flush-interval", type=int, default=1, help="CSV flush interval in seconds")
    ap.add_argument(
        "--realtime",
        action="store_true",
        help="Print decoded frames to terminal in real-time",
    )
    ap.add_argument(
        "--realtime-mode",
        choices=["append", "latest"],
        default="append",
        help="Realtime output mode: append (default) or latest (update snapshot block)",
    )
    ap.add_argument("--save-mat", action="store_true", help="Export MAT file on exit (needs scipy)")
    return ap


if __name__ == "__main__":
    parser = build_argparser()
    args = parser.parse_args()

    if args.list_ports:
        ports = list_serial_port_names()
        if ports:
            print("[INFO] Available serial ports:")
            for p in ports:
                print(f"  {p}")
        else:
            print("[INFO] No serial ports found.")
        sys.exit(0)

    if args.port is None:
        args.port = auto_pick_port()

    sys.exit(run(args))
