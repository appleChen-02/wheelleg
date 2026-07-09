#!/usr/bin/env python3
"""
SBUS raw data reader — 读取 MCU 发出的遥控器 16 通道原始 SBUS 值。

用法（中文）:
    # 自动搜索串口，实时打印 16 通道原始值到终端
    python sbus_raw_reader.py --realtime

    # 指定端口，最新快照模式（终端底部固定区域刷新）
    python sbus_raw_reader.py --port COM10 --baud 115200 --realtime --realtime-mode latest

    # 同时保存 CSV 文件供 MATLAB 分析
    python sbus_raw_reader.py --realtime --outdir ./sbus_log

    # 列出可用串口后退出
    python sbus_raw_reader.py --list-ports

    # 运行 30 秒后自动停止
    python sbus_raw_reader.py --realtime --duration 30

Examples (English):
    python sbus_raw_reader.py --port COM3 --baud 115200 --realtime
    python sbus_raw_reader.py --realtime --outdir ./sbus_log
"""

from __future__ import annotations

import argparse
import csv
import os
import struct
import sys
import time
from typing import List, Optional, Tuple

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except Exception as exc:
    raise SystemExit(
        "pyserial is required. Install with: pip install pyserial\n"
        f"Import error: {exc}"
    )

# ========================================================================
# 协议常量（与下位机保持一致）
# ========================================================================
SOF = 0x5A
ID_SBUS_RAW = 0x0D

# SBUS 原始数据包 payload: <I16HBB> = 4 + 32 + 1 + 1 = 38 bytes
SBUS_RAW_STRUCT = struct.Struct("<I16HBB")

# 16 通道的列名（CH0 ~ CH15）
SBUS_CH_COLUMNS = [f"CH{i}" for i in range(16)]
SBUS_CSV_COLUMNS = ["host_time_s", "tick_ms"] + SBUS_CH_COLUMNS + ["connect_flag"]

# CRC8 表（与下位机固件一致）
CRC8_INIT = 0xFF
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


# ========================================================================
# CRC 校验函数
# ========================================================================
def crc8(data: bytes, init: int = CRC8_INIT) -> int:
    crc = init
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc & 0xFF


def crc16(data: bytes, init: int = 0xFFFF) -> int:
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
    return crc16(frame[:-2]) == expected


# ========================================================================
# 帧解析器
# ========================================================================
class FrameParser:
    """状态机式帧解析器，处理半包/粘包/噪声字节。"""

    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> List[bytes]:
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

            header = bytes(self.buf[:4])
            if not verify_header_crc8(header):
                del self.buf[0]
                continue

            payload_len = header[1]
            frame_len = 4 + payload_len + 2

            if payload_len > 200:
                del self.buf[0]
                continue

            if len(self.buf) < frame_len:
                break

            frame = bytes(self.buf[:frame_len])
            if not verify_frame_crc16(frame):
                del self.buf[0]
                continue

            out.append(frame)
            del self.buf[:frame_len]

        return out


# ========================================================================
# 辅助函数
# ========================================================================
def ansi_supported() -> bool:
    """检测终端是否支持 ANSI 转义序列。"""
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


# 参考偏移值（供偏差显示用，实际值以后续实测为准）
# HT8A  CH0/1/3 中值 992；CH2/4/7 中值 992
SBUS_MID = 992


def format_sbus_line(tick_ms: int, ch: List[int], connect_flag: int, show_deviation: bool = True) -> str:
    """将一帧 SBUS 数据格式化为终端输出字符串。"""
    now_str = time.strftime("%H:%M:%S")

    # 通道行：第一行显示 CH0-7，第二行显示 CH8-15
    def ch_str(i: int) -> str:
        val = ch[i]
        if show_deviation:
            dev = val - SBUS_MID
            sign = "+" if dev >= 0 else ""
            return f"CH{i:2d}:{val:4d}({sign}{dev})"
        else:
            return f"CH{i:2d}:{val:4d}"

    line0 = " ".join(ch_str(i) for i in range(8))
    line1 = " ".join(ch_str(i) for i in range(8, 16))
    header = f"[SBUS] {now_str} tick={tick_ms:5d} conn={connect_flag:3d}"

    return f"{header}\n  {line0}\n  {line1}"


def list_serial_port_names() -> List[str]:
    names = [p.device for p in list_ports.comports()]
    names.sort()
    return names


def auto_pick_port() -> str:
    ports = list_serial_port_names()
    if not ports:
        raise SystemExit(
            "No serial port found. Check USB cable and permissions."
        )
    preferred = ["/dev/ttyACM", "/dev/ttyUSB", "COM"]
    for pattern in preferred:
        for p in ports:
            if p.startswith(pattern):
                print(f"[INFO] Auto selected port: {p}")
                return p
    print(f"[INFO] Auto selected port: {ports[0]}")
    return ports[0]


# ========================================================================
# 主运行逻辑
# ========================================================================
def run(args: argparse.Namespace) -> int:
    # ---------- 初始化 CSV 输出 ----------
    csv_file = None
    csv_writer = None
    csv_rows: List[List] = []
    if args.outdir:
        os.makedirs(args.outdir, exist_ok=True)
        csv_path = os.path.join(args.outdir, "sbus_raw.csv")
        csv_file = open(csv_path, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(SBUS_CSV_COLUMNS)
        print(f"[INFO] CSV output: {csv_path}")

    parser = FrameParser()
    ansi_ok = ansi_supported()
    latest_lines_rendered = 0

    # Latest 快照缓存
    latest_tick: int = 0
    latest_ch: List[int] = [0] * 16
    latest_conn: int = 0
    latest_has_data: bool = False

    print(f"[INFO] Open serial: {args.port}, baud={args.baud}")
    if args.outdir:
        print(f"[INFO] Output dir : {args.outdir}")
    print(
        f"[INFO] Waiting for SBUS raw data (ID=0x{ID_SBUS_RAW:02X})...\n"
        f"       Reference mid-value = {SBUS_MID}\n"
        f"       Press Ctrl+C to stop.\n"
    )

    t0 = time.time()

    try:
        with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
            while True:
                chunk = ser.read(args.read_size)
                if chunk:
                    frames = parser.feed(chunk)
                    for fr in frames:
                        pkt_id = fr[2]
                        if pkt_id != ID_SBUS_RAW:
                            continue

                        # 校验 payload 长度
                        expected_len = SBUS_RAW_STRUCT.size
                        actual_len = len(fr) - 6  # 减去 header(4) + crc16(2)
                        if actual_len != expected_len:
                            continue

                        payload = fr[4:-2]
                        tick_ms, *ch16, connect_flag, _reserved = SBUS_RAW_STRUCT.unpack(payload)

                        host_t = time.time()
                        latest_tick = tick_ms
                        latest_ch = list(ch16)
                        latest_conn = connect_flag
                        latest_has_data = True

                        # ---------- 写入 CSV ----------
                        if csv_writer is not None:
                            row = [host_t, tick_ms] + latest_ch + [connect_flag]
                            csv_writer.writerow(row)
                            csv_rows.append(row)

                        # ---------- 命令行实时打印 ----------
                        if getattr(args, "realtime", False):
                            mode = getattr(args, "realtime_mode", "append")
                            if mode == "append":
                                print(format_sbus_line(tick_ms, latest_ch, connect_flag))
                            elif mode == "latest":
                                if ansi_ok and latest_lines_rendered > 0:
                                    sys.stdout.write(f"\x1b[{latest_lines_rendered}F")
                                lines = format_sbus_line(tick_ms, latest_ch, connect_flag).split("\n")
                                for line in lines:
                                    sys.stdout.write("\x1b[2K" + line + "\n")
                                sys.stdout.flush()
                                latest_lines_rendered = len(lines)

                # ---------- 定时检查退出条件 ----------
                if args.duration > 0 and (time.time() - t0) >= args.duration:
                    print("[INFO] Reached duration limit, stopping.")
                    break

    except serial.SerialException as exc:
        print(f"[ERROR] Serial error: {exc}")
        if sys.platform.startswith("linux"):
            print("[HINT] Ubuntu may need: sudo usermod -a -G dialout $USER (then re-login)")
        return 1
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        if csv_file is not None:
            csv_file.close()

    # ---------- 最终统计 ----------
    print(f"[INFO] Total SBUS raw frames captured: {len(csv_rows)}")
    if latest_has_data:
        print(f"[INFO] Final snapshot (tick={latest_tick}, conn={latest_conn}):")
        for i in range(0, 16, 8):
            row_str = "  ".join(
                f"CH{i+j:2d}: {latest_ch[i+j]:5d} (dev={latest_ch[i+j] - SBUS_MID:+5d})"
                for j in range(8)
            )
            print(f"  {row_str}")

        # 自动计算各通道量程
        if len(csv_rows) > 0:
            print("\n[INFO] Per-channel range analysis (from captured data):")
            print(f"  {'CH':<6} {'Min':>6} {'Max':>6} {'Range':>8} {'Mid':>6} {'Deviation':>10}")
            print(f"  {'-'*6} {'-'*6} {'-'*6} {'-'*8} {'-'*6} {'-'*10}")
            for i in range(16):
                vals = [r[2 + i] for r in csv_rows]  # r[0]=host_t, r[1]=tick_ms, r[2:]=ch[0..15]
                vmin, vmax = min(vals), max(vals)
                vrange = vmax - vmin
                vmid = (vmax + vmin) // 2
                dev = max(abs(vmax - SBUS_MID), abs(vmin - SBUS_MID))
                marker = ""
                if vrange > 0:
                    if dev > 400:
                        marker = "  <-- likely joystick/analog channel"
                    elif dev > 200:
                        marker = "  <-- likely switch channel"
                print(f"  {'CH'+str(i):<6} {vmin:>6} {vmax:>6} {vrange:>8} {vmid:>6} {dev:>10}{marker}")

    return 0


# ========================================================================
# 命令行解析
# ========================================================================
def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="SBUS raw data reader — read 16-channel raw SBUS values from MCU via USB CDC."
    )
    ap.add_argument(
        "--port", default=None,
        help="Serial port, e.g. COM6 (Windows) or /dev/ttyACM0 (Linux). Auto-detect if omitted.",
    )
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    ap.add_argument("--timeout", type=float, default=0.02, help="Serial read timeout in seconds")
    ap.add_argument("--read-size", type=int, default=512, help="Bytes read per cycle")
    ap.add_argument(
        "--outdir", default=None,
        help="Output directory for CSV. If omitted, no CSV is saved.",
    )
    ap.add_argument(
        "--duration", type=float, default=0.0,
        help="Stop after N seconds (0 = run forever)",
    )
    ap.add_argument(
        "--realtime", action="store_true",
        help="Print decoded frames to terminal in real-time",
    )
    ap.add_argument(
        "--realtime-mode", choices=["append", "latest"], default="latest",
        help="Realtime display mode: 'append' (scroll) or 'latest' (fixed snapshot area, default)",
    )
    ap.add_argument(
        "--list-ports", action="store_true",
        help="List available serial ports and exit",
    )
    return ap


if __name__ == "__main__":
    ap = build_argparser()
    args = ap.parse_args()

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

    # 默认开启 realtime，方便查看
    if not args.realtime:
        args.realtime = True

    sys.exit(run(args))
