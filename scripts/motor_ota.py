"""Linux 板载 CANFD 固件烧录模块
协议层 + socketcan 传输层合并于一个文件，仅依赖 Python 标准库。
使用方法
========

单台设备（最简）::

    from motor_ota import update_firmware

    update_firmware("app.bin", can_id=0x1009, interface="can_rdk")

多台设备循环烧录（套接字只打开一次，推荐）::

    from motor_ota import LinuxFlasher, FlashError

    with LinuxFlasher(interface="can_rdk") as flasher:
        for can_id in (0x1009, 0x100B):
            try:
                flasher.flash("app.bin", can_id=can_id)
            except FlashError as e:
                print(f"CAN 0x{can_id:X} 升级失败: {e}")

进度回调::

    def on_progress(written, total):
        print(f"{written}/{total} ({written * 100 // total}%)")

    update_firmware("app.bin", can_id=0x1009, interface="can_rdk",
                    on_progress=on_progress)

命令行直接运行::

    python3 -m motor_ota --firmware app.bin --can-id 2 --interface can_rdk
    --can-id 指定电机 CAN ID，支持十进制和 0x 十六进制
    app.bin 改为需要烧录的固件路径

板端环境要求
============

- Linux，内核支持 CANFD，且接口已配置并启用（每次上电配置一次）::

    sudo ip link set can_rdk type can bitrate 1000000 dbitrate 5000000 fd on
    sudo ip link set can_rdk up

- 仲裁段 1 Mbps / 数据段 5 Mbps 为设备出厂约定，不可改
- 实烧注意事项（X5 板实测）：电机必须供电、与板子**共地**、
  每次烧录前设备**断电重上电**（一次连接后进入升级态，断电重上电才能再次连接）
- 固件文件为原始 BIN，原样写入，不做任何转换
- 本模块为同步阻塞模型；需要异步请自行放入后台线程
"""

import argparse
import logging
import socket
import struct
import sys
import time
from dataclasses import dataclass

__all__ = [
    'update_firmware', 'LinuxFlasher', 'SocketcanAdapter', 'FlashTimeouts',
    'FlashError', 'AdapterError', 'ConnectError', 'CommandError',
    'TransportError', 'FlashTimeoutError', 'VerifyError', 'FirmwareFileError',
    'FIRMWARE_BASE_ADDRESS',
]

# ---- 固定参数（与设备端约定一致，不对外暴露修改） ----
FIRMWARE_BASE_ADDRESS = 0x60000000  # 设备固件区起始地址
BLOCK_SIZE = 256                    # 单次写入块大小（字节）
RESPONSE_ID_OFFSET = 1              # 设备应答 CAN ID = 请求 ID + 1


# ---- 异常体系 ----
class FlashError(Exception):
    """固件升级错误基类。"""


class AdapterError(FlashError):
    """适配器打开或初始化失败（请检查 CAN 接口配置与启用状态）。"""


class ConnectError(FlashError):
    """设备连接失败（请将设备断电后重新上电再试）。"""


class CommandError(FlashError):
    """设备返回错误（附错误码中文说明）。"""


class TransportError(FlashError):
    """总线传输异常（多帧序列或流控错误）。"""


class FlashTimeoutError(FlashError):
    """等待设备响应超时。"""


class VerifyError(FlashError):
    """设备自校验未通过。"""


class FirmwareFileError(FlashError):
    """固件文件读取失败。"""


# ---- 超时配置 ----
@dataclass
class FlashTimeouts:
    """各阶段超时（毫秒）。默认值即实机验证的推荐值。"""
    connect_ms: int = 500          # 连接响应
    connect_retries: int = 3       # 连接重试次数
    command_ms: int = 1000         # 常规命令
    program_start_ms: int = 2000   # 进入升级模式
    erase_ms: int = 10000          # 擦除（较慢）
    verify_ms: int = 10000         # 自校验
    cf_gap_ms: int = 1000          # 多帧/流控帧间隔


# ---- 设备协议命令码（协议约定值） ----
CMD_CONNECT = 0xFF        # 连接
CMD_DISCONNECT = 0xFE
CMD_GET_STATUS = 0xFD
CMD_SET_MTA = 0xF6        # 设置访问地址
CMD_PROGRAM_START = 0xD2  # 进入写入模式
CMD_PROGRAM_CLEAR = 0xD1  # 擦除
CMD_PROGRAM = 0xD0        # 写入（短包）
CMD_PROGRAM_BULK = 0xC8   # 写入（数据块）
CMD_PROGRAM_RESET = 0xCF  # 写入完成复位
CMD_USER = 0xF1           # 自定义命令（自校验）
PID_POSITIVE = 0xFF       # 正响应
PID_NEGATIVE = 0xFE       # 负响应


LOG_TRACE = 5   # 逐帧字节日志级别（低于 DEBUG）


def make_logger():
    """模块 logger：默认静默，客户自行挂 handler 接管。"""
    logging.addLevelName(LOG_TRACE, 'TRACE')
    logger = logging.getLogger('firmware_flasher')
    logger.addHandler(logging.NullHandler())
    return logger


logger = make_logger()


def load_firmware(path):
    """读取固件 BIN 文件（原样字节，不做转换）。"""
    try:
        with open(path, 'rb') as f:
            data = f.read()
    except OSError as e:
        raise FirmwareFileError(f'固件文件读取失败: {path} ({e})') from e
    if not data:
        raise FirmwareFileError(f'固件文件为空: {path}')
    return data


def split_blocks(data):
    """按 BLOCK_SIZE 切块，末块取余。"""
    return [data[i:i + BLOCK_SIZE] for i in range(0, len(data), BLOCK_SIZE)]


# ---- 总线传输层：多帧拆包编码（ISO 15765-2 经典 8 字节帧） ----
_SF_MAX = 7      # 单帧最大载荷
_FF_PAYLOAD = 6  # 首帧载荷
_CF_PAYLOAD = 7  # 连续帧载荷


def _pad8(frame_head, payload):
    """帧头 + 载荷补 0 到 8 字节。"""
    return bytes(frame_head + list(payload)).ljust(8, b'\x00')


def encode_sf(payload):
    """单帧：0x0L + 载荷（≤7 字节）。"""
    if len(payload) > _SF_MAX:
        raise ValueError('单帧载荷超限')
    return _pad8([0x00 | len(payload)], payload)


def encode_ff(payload):
    """首帧：1L LL（12 位总长）+ 前 6 字节。"""
    if len(payload) < 8:
        raise ValueError('短载荷不应拆多帧')
    head = [0x10 | (len(payload) >> 8), len(payload) & 0xFF]
    return _pad8(head, payload[:_FF_PAYLOAD])


def encode_cfs(payload):
    """首帧之后的全部连续帧：2N（SN=1..15 循环跳 0）+ 7 字节；末帧只载剩余字节，不补 0。"""
    frames = []
    body = payload[_FF_PAYLOAD:]
    sn = 0
    for i in range(0, len(body), _CF_PAYLOAD):
        sn = (sn + 1) & 0x0F
        if sn == 0:
            sn = 1
        frames.append(bytes([0x20 | sn]) + body[i:i + _CF_PAYLOAD])
    return frames


def encode_message(payload):
    """拆帧总入口：返回 (首帧列表, 连续帧列表)。SF 时连续帧为空。"""
    if len(payload) <= _SF_MAX:
        return [encode_sf(payload)], []
    return [encode_ff(payload)], encode_cfs(payload)


# ---- 总线传输层：多帧接收重组（与 encode 层互逆） ----
_FC_CTS = bytes([0x30, 0x00, 0x00, 0, 0, 0, 0, 0])  


class IsoTpReceiver:
    """总线多帧接收重组状态机（纯逻辑，不碰硬件）。

    feed() 每收到一帧调用一次，返回 (完成的消息, 需回发的帧)。
    收到首帧自动产出流控帧；序号错乱则丢弃当前消息并重置。
    流控帧与非本层帧静默忽略。
    """

    def __init__(self):
        self._buffer = bytearray()
        self._total = 0
        self._expected_sn = 0

    def feed(self, frame):
        if len(frame) < 1:
            return None, None
        nibble = frame[0] >> 4
        if nibble == 0 and 0 < frame[0] & 0x0F <= _SF_MAX:
            length = frame[0] & 0x0F
            return bytes(frame[1:1 + length]), None
        if nibble == 1:
            self._total = ((frame[0] & 0x0F) << 8) | frame[1]
            self._buffer = bytearray(frame[2:8])
            self._expected_sn = 0
            return None, _FC_CTS
        if nibble == 2:
            sn = frame[0] & 0x0F
            expected = (self._expected_sn + 1) & 0x0F or 1
            if sn != expected or self._total == 0:
                self.__init__()   # 序号错乱：丢弃重置
                return None, None
            self._expected_sn = sn
            need = self._total - len(self._buffer)
            self._buffer += frame[1:1 + min(7, need)]
            if len(self._buffer) >= self._total:
                msg = bytes(self._buffer[:self._total])
                self.__init__()
                return msg, None
        return None, None   

class IsoTpLink:
    """总线帧与会话级消息之间的传输层。

    send_and_receive(): 发一条完整命令、等一条完整应答。
    多帧命令在首帧后等待设备流控（CTS 才发后续帧，WAIT 继续等，
    OVFLW 报错）；等应答期间收到设备首帧会自动回流控帧。
    """

    def __init__(self, send_raw, recv_raw, cf_gap_ms=None):
        self._send_raw = send_raw
        self._recv_raw = recv_raw
        self._cf_gap_ms = cf_gap_ms or 1000

    def _poll_one_frame(self, deadline):
        """取一帧；超时抛 FlashTimeoutError。"""
        while True:
            frame = self._recv_raw()
            if frame is not None:
                return frame
            if time.monotonic() >= deadline:
                raise FlashTimeoutError('等待设备应答超时')
            time.sleep(0.001)

    def send_and_receive(self, payload, timeout_ms):
        first, cfs = encode_message(payload)
        for frame in first:
            self._send_raw(frame)
        if cfs:
            self._await_flow_control_then_send(cfs)
        return self._receive_message(timeout_ms)

    def _await_flow_control_then_send(self, cfs):
        deadline = time.monotonic() + (self._cf_gap_ms / 1000.0)
        while True:
            frame = self._poll_one_frame(deadline)
            nibble = frame[0] >> 4
            if nibble != 3:
                continue                       
            status = frame[0] & 0x0F
            if status == 0:                    # CTS：一次性连发（BS=0）
                for cf in cfs:
                    self._send_raw(cf)
                return
            if status == 1:                    # WAIT：重置时限继续等
                deadline = time.monotonic() + (self._cf_gap_ms / 1000.0)
                continue
            raise TransportError(f'设备流控溢出 (0x{frame[0]:02X})')

    def _receive_message(self, timeout_ms):
        receiver = IsoTpReceiver()
        deadline = time.monotonic() + (timeout_ms / 1000.0)
        while True:
            frame = self._poll_one_frame(deadline)
            msg, reply = receiver.feed(frame)
            if reply is not None:
                self._send_raw(reply)
            if msg is not None:
                return msg

    def send_and_receive_no_wait(self, payload):
        first, cfs = encode_message(payload)
        for frame in first:
            self._send_raw(frame)
        if cfs:
            # 复位命令只有 1 字节，实际不会走这里；保留通用性
            for cf in cfs:
                self._send_raw(cf)


# ---- 设备命令层（组包/判答，纯函数） ----
ERROR_MESSAGES = {
    0x00: '命令同步错误',
    0x10: '设备忙',
    0x20: '未知命令',
    0x22: '参数超范围',
    0x25: '访问被锁定',
    0x26: '页无效',
    0x29: '命令序列错误',
    0x31: '设备写入失败',
}


def _le32(value):
    return value.to_bytes(4, 'little')


def build_connect():
    return bytes([CMD_CONNECT, 0x00])


def build_get_status():
    return bytes([CMD_GET_STATUS])


def build_program_start():
    return bytes([CMD_PROGRAM_START])


def build_set_mta(address):
    return bytes([CMD_SET_MTA, 0, 0, 0]) + _le32(address)


def build_program_clear(total_bytes):
    return bytes([CMD_PROGRAM_CLEAR, 0, 0, 0]) + _le32(total_bytes)


def build_program_bulk(data):
    return bytes([CMD_PROGRAM_BULK]) + len(data).to_bytes(2, 'little') + data


def build_program(data):
    if len(data) > 255:
        raise ValueError('短包载荷超限')
    return bytes([CMD_PROGRAM, len(data)]) + data


def build_selfcheck(total_bytes):
    return bytes([CMD_USER, 0x02]) + _le32(total_bytes) + b'\x00\x00'


def build_program_reset():
    return bytes([CMD_PROGRAM_RESET])


def check_positive(resp):
    """判定正响应；负响应抛 CommandError（含错误码中文说明）。"""
    if not resp or resp[0] not in (PID_POSITIVE, PID_NEGATIVE):
        raise CommandError(f'设备应答异常: {resp.hex() if resp else "空"}')
    if resp[0] == PID_NEGATIVE:
        code = resp[1] if len(resp) > 1 else 0xFF
        reason = ERROR_MESSAGES.get(code, f'未知错误 0x{code:02X}')
        raise CommandError(f'设备返回错误: {reason} (0x{code:02X})')


def parse_connect(resp):
    check_positive(resp)
    if len(resp) < 4:
        raise CommandError(f'连接应答长度不足: {resp.hex()}')
    return resp[3]


def parse_selfcheck(resp):
    """自校验应答：0x01 通过；0x02 设备端暂定放行（与上位机既有行为一致）。"""
    check_positive(resp)
    if len(resp) < 3 or resp[1] != 0x02:
        raise CommandError(f'自校验应答异常: {resp.hex() if resp else "空"}')
    if resp[2] not in (0x01, 0x02):
        raise VerifyError(f'设备自校验未通过 (result=0x{resp[2]:02X})')


# ---- 设备升级会话编排 ----
class DeviceSession:
    """单台设备的升级会话：连接、写入、自校验、复位。"""

    def __init__(self, link, timeouts):
        self._link = link
        self._t = timeouts

    def _command(self, payload, timeout_ms):
        resp = self._link.send_and_receive(payload, timeout_ms)
        check_positive(resp)
        return resp

    def connect(self):
        """连接设备（带重试；失败提示断电重上电）。"""
        last_error = None
        for _ in range(self._t.connect_retries):
            try:
                resp = self._link.send_and_receive(
                    build_connect(), self._t.connect_ms)
                parse_connect(resp)
                logger.debug('设备连接成功 (单包上限=%d)', resp[3])
                return
            except (FlashTimeoutError, CommandError) as e:
                last_error = e
        raise ConnectError(
            f'设备连接失败（已重试 {self._t.connect_retries} 次）：{last_error}。'
            '请将设备断电后重新上电再试（设备在一次连接后进入升级态，'
            '断电重上电才能再次连接）') from last_error

    def get_status(self):
        self._command(build_get_status(), self._t.command_ms)

    def program_start(self):
        self._command(build_program_start(), self._t.program_start_ms)

    def set_mta(self, address):
        self._command(build_set_mta(address), self._t.command_ms)

    def program_clear(self, total_bytes):
        logger.debug('擦除 %d 字节...', total_bytes)
        self._command(build_program_clear(total_bytes), self._t.erase_ms)

    def _write_block(self, block):
        if len(block) == BLOCK_SIZE:
            msg = build_program_bulk(block)
        else:
            msg = build_program(block)
        self._command(msg, self._t.command_ms)

    def selfcheck(self, total_bytes):
        logger.debug('设备自校验 %d 字节...', total_bytes)
        resp = self._link.send_and_receive(
            build_selfcheck(total_bytes), self._t.verify_ms)
        parse_selfcheck(resp)

    def program_reset(self):
        """复位命令不期待应答（设备收到即重启）。"""
        self._link.send_and_receive_no_wait(build_program_reset())

    def flash(self, firmware, on_progress=None):
        """完整升级流程：握手→擦除→逐块写入→自校验→复位。"""
        blocks = split_blocks(firmware)
        total = len(firmware)
        self.get_status()
        self.program_start()
        self.set_mta(FIRMWARE_BASE_ADDRESS)
        self.program_clear(total)
        written = 0
        for block in blocks:
            self._write_block(block)
            written += len(block)
            logger.debug('写入进度 %d/%d', written, total)
            if on_progress is not None:
                on_progress(written, total)
        self.set_mta(FIRMWARE_BASE_ADDRESS)
        self.selfcheck(total)
        self.program_reset()
        logger.info('升级完成: %d 字节', total)


# ---- socketcan 传输层（板载 CANFD，无第三方依赖） ----
CAN_EFF_FLAG = 0x80000000   # 扩展帧（29 位 ID）
CANFD_BRS = 0x01            # 数据段波特率切换（5 Mbps）
DLC = 8                     # 协议帧固定 8 字节载荷

# 内核帧布局:
#   struct can_frame     can_id(4) len(1) 保留(3) data(8)  = 16 字节（经典帧）
#   struct canfd_frame   can_id(4) len(1) flags(1) 保留(2) data(64) = 72 字节（CANFD 帧）
_CAN_FRAME = struct.Struct('<IB3x8s')
_CANFD_FRAME = struct.Struct('<IBB2x64s')

# socket 模块常量随 Python 版本有差异，缺失时用内核头文件值兜底
_SOL_CAN_RAW = getattr(socket, 'SOL_CAN_RAW', 101)
_CAN_RAW_FD_FRAMES = getattr(socket, 'CAN_RAW_FD_FRAMES', 5)


def frame_id_matches(frame_id, expected):
    return (frame_id & 0x1FFFFFFF) == (expected & 0x1FFFFFFF)


class SocketcanAdapter:

    def __init__(self, interface='can0'):
        self._ifname = interface
        self._sock = None

    def open(self):
        try:
            self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW,
                                       socket.CAN_RAW)
            # 不设 CAN_RAW_FD_FRAMES 时，CANFD 帧收发都会被内核静默丢弃/拒绝
            self._sock.setsockopt(_SOL_CAN_RAW, _CAN_RAW_FD_FRAMES, 1)
            self._sock.bind((self._ifname,))
            self._sock.setblocking(False)
        except OSError as e:
            if self._sock is not None:
                self._sock.close()
                self._sock = None
            raise AdapterError(
                f'socketcan 打开失败: {e}。请确认接口已配置并启用: '
                f'sudo ip link set {self._ifname} type can bitrate 1000000 '
                f'dbitrate 5000000 fd on && sudo ip link set {self._ifname} up') from e

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def send_frame(self, can_id, data):
        if not 0 <= can_id <= 0x1FFFFFFE:
            raise AdapterError(f'扩展 CAN ID 超出范围或无应答 ID 空间: 0x{can_id:X}')
        if self._sock is None:
            raise AdapterError('socketcan 尚未打开')
        if len(data) > DLC:
            raise TransportError(f'单帧数据超过 {DLC} 字节')
        padded = bytes(data).ljust(DLC, b'\x00')
        frame = _CANFD_FRAME.pack(can_id | CAN_EFF_FLAG, DLC, CANFD_BRS, padded)
        try:
            self._sock.send(frame)
        except OSError as e:
            raise TransportError(f'socketcan 发送失败: {e}') from e

    def recv_frame(self):
        """非阻塞收一帧；无数据返回 None。"""
        try:
            frame = self._sock.recv(_CANFD_FRAME.size)
        except BlockingIOError:
            return None
        except OSError as e:
            raise TransportError(f'socketcan 接收失败: {e}') from e
        if len(frame) == _CAN_FRAME.size:
            can_id, dlc, data = _CAN_FRAME.unpack(frame)
            return can_id, bytes(data[:dlc if dlc else 8])
        if len(frame) == _CANFD_FRAME.size:
            can_id, dlc, _flags, data = _CANFD_FRAME.unpack(frame)
            return can_id, bytes(data[:DLC])
        return None   # 未知帧长，忽略


class LinuxFlasher:
    def __init__(self, interface='can0', timeouts=None):
        self._interface = interface
        self._timeouts = timeouts or FlashTimeouts()
        self._adapter = None

    def _make_io(self, can_id):
        """返回 (send_raw, recv_raw)：发请求帧、只认本机应答 ID。"""
        adapter = self._adapter

        def send_raw(frame, _a=adapter, _id=can_id):
            _a.send_frame(_id, frame)

        response_id = can_id + RESPONSE_ID_OFFSET

        def recv_raw(_a=adapter, _rid=response_id):
            got = _a.recv_frame()
            if got is None:
                return None
            frame_id, data = got
            return data if frame_id_matches(frame_id, _rid) else None

        return send_raw, recv_raw

    def __enter__(self):
        self._adapter = SocketcanAdapter(self._interface)
        self._adapter.open()
        return self

    def __exit__(self, *exc):
        if self._adapter is not None:
            self._adapter.close()
            self._adapter = None

    def flash(self, firmware_path, can_id, on_progress=None):
        """烧录一台设备：读文件 → 连接 → 写入 → 自校验 → 复位。"""
        firmware = load_firmware(firmware_path)
        send_raw, recv_raw = self._make_io(can_id)
        link = IsoTpLink(send_raw, recv_raw, cf_gap_ms=self._timeouts.cf_gap_ms)
        session = DeviceSession(link, self._timeouts)
        session.connect()
        session.flash(firmware, on_progress)


def update_firmware(firmware_path, can_id, interface='can0',
                    on_progress=None, timeouts=None):
    """单台设备一键升级（便捷入口；内部开关一次套接字）。"""
    with LinuxFlasher(interface, timeouts) as flasher:
        flasher.flash(firmware_path, can_id, on_progress)


def _parse_can_id(value):
    try:
        can_id = int(value, 0)
    except ValueError as e:
        raise argparse.ArgumentTypeError(f'无效的 CAN ID: {value}') from e
    if not 0 <= can_id <= 0x1FFFFFFE:
        raise argparse.ArgumentTypeError('CAN ID 必须在 0x0 到 0x1FFFFFFE 之间')
    return can_id


# ---- 命令行入口 ----
def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description='固件升级工具（Linux 板载 CANFD）：向设备烧录固件 BIN')
    parser.add_argument('--firmware', required=True, help='固件 BIN 文件路径')
    parser.add_argument('--can-id', required=True, type=_parse_can_id,
                        help='目标设备 CAN ID（支持 0x 十六进制，如 0x1009）')
    parser.add_argument('--interface', default='can0',
                        help='CAN 接口名（默认 can0）')
    return parser.parse_args(argv)


def _main(argv=None):
    args = _parse_args(argv)
    last_percent = -1

    def on_progress(written, total):
        nonlocal last_percent
        percent = written * 100 // total
        if percent != last_percent:
            last_percent = percent
            print(f'\r写入进度: {percent}%', end='', flush=True)

    try:
        update_firmware(args.firmware, args.can_id,
                        interface=args.interface, on_progress=on_progress)
    except FlashError as e:
        print()
        print(f'升级失败: {e}', file=sys.stderr)
        return 1
    print()
    print('升级完成')
    return 0


if __name__ == '__main__':
    sys.exit(_main())
