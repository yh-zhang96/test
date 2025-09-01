import pysoem


master = pysoem.Master()
pysoem.find_adapters()
adapter_name = '\\Device\\NPF_{E65C214B-4E36-4148-9B34-D8DCE81F25C0}'
master.open(adapter_name)

if master.config_init():
    print("成功扫描到 EtherCAT 从站")
else:
    print("未找到 EtherCAT 从站")
#print(pysoem.find_adapters())

if master.config_init() <= 0:
    raise RuntimeError('no slaves found')

# 找到 EP7342 从站（按名字或 Vendor/Product ID）
drive = next(s for s in master.slaves if 'EP7342' in s.name)

# 先写 SDO：选“位置控制”模式
drive.sdo_write(0x8022, 1, bytes([3]))   # 通道1 Position controller

# 建立 PDO 映射并进入 OP
master.config_map()
master.state = pysoem.OP_STATE
master.write_state()
master.state_check(pysoem.OP_STATE, 5_000)
*****************************************************************************************************

#PDO的读写语法
drive.output[target_offset:target_offset+4] = value.to_bytes(4, 'little', signed=True)
pos = int.from_bytes(drive.input[actual_offset:actual_offset+4], 'little', signed=True)

******************************************************************************************************
#怎么得到 target_offset（步骤说明 + 示例代码）

# 说明：假设 'slave' 是 pysoem 中的某个从站对象（master.slaves[i]）
# 这个函数返回一个 dict，包含 bit_offset 和 byte_offset，如果没找到则返回 None。

def sdo_read_uint32_as_int(slave, index, subindex):
    # 读取 4 字节 SDO（按设备实现可以读取更短），返回 int
    data = slave.sdo_read(index, subindex, 4)
    return int.from_bytes(bytes(data), 'little', signed=False)

def sdo_read_uint8(slave, index, subindex=0):
    data = slave.sdo_read(index, subindex, 1)
    return int.from_bytes(bytes(data), 'little', signed=False)

def read_pdo_assign_list(slave, assign_index):
    # 读 0x1C12 (RxPDO Assign) 或 0x1C13 (TxPDO Assign)
    count = sdo_read_uint8(slave, assign_index, 0)
    pdo_list = []
    for i in range(1, count+1):
        val = sdo_read_uint32_as_int(slave, assign_index, i)
        pdo_index = val & 0xFFFF
        pdo_list.append(pdo_index)
    return pdo_list

def read_pdo_mapping_entries(slave, pdo_index):
    # 读取某个 PDO（如 0x160A）的映射条目
    cnt = sdo_read_uint8(slave, pdo_index, 0)
    entries = []
    for i in range(1, cnt+1):
        raw = sdo_read_uint32_as_int(slave, pdo_index, i)
        entry_index = (raw >> 16) & 0xFFFF
        entry_sub   = (raw >> 8)  & 0xFF
        entry_bits  = raw & 0xFF
        entries.append({'index': entry_index, 'sub': entry_sub, 'bits': entry_bits})
    return entries

def find_field_byte_offset_in_rxpdo(slave, field_index, field_sub, rxpdo_assign_index=0x1C12):
    bit_cursor = 0
    pdo_list = read_pdo_assign_list(slave, rxpdo_assign_index)
    for pdo in pdo_list:
        mapping = read_pdo_mapping_entries(slave, pdo)
        for entry in mapping:
            if entry['index'] == field_index and entry['sub'] == field_sub:
                return {'pdo': pdo, 'bit_offset': bit_cursor, 'byte_offset': bit_cursor // 8, 'bits': entry['bits']}
            bit_cursor += entry['bits']
        # 有些实现会在一个 PDO 结束后向上对齐到下一个整字节
        if bit_cursor % 8 != 0:
            bit_cursor += (8 - (bit_cursor % 8))
    return None

#调用示例：
# 假设 target field 是 0x7040:11（EP7342 常见的 Target Position）
res = find_field_byte_offset_in_rxpdo(drive, 0x7040, 0x11)
print(res)
# 可能输出: {'pdo': 0x160A, 'bit_offset': 32, 'byte_offset': 4, 'bits': 32}
# 说明目标位置从 drive.output 的第 4 字节开始，占 32 位（4 字节）
*****************************************************************************************************
# 保存为ep7342_lib.py

import time
import struct
import pysoem


class EP7342Axis:
    def __init__(self, net_if: str, channel: int = 1,
                 encoder_cpr: int = 4096, gear_ratio: float = 1.0, unit: str = "mm"):
        """
        EP7342 Axis wrapper for one channel (Ch1 or Ch2).

        :param net_if: EtherCAT network interface (e.g. 'eth0')
        :param channel: 1 or 2 (EP7342 has two channels)
        :param encoder_cpr: Encoder counts per revolution
        :param gear_ratio: Gear ratio (motor turns : output turns)
        :param unit: 'mm' or 'deg'
        """
        self.net_if = net_if
        self.channel = channel
        self.encoder_cpr = encoder_cpr
        self.gear_ratio = gear_ratio
        self.unit = unit

        self.master = pysoem.Master()
        self.master.open(net_if)

        if self.master.config_init() <= 0:
            raise RuntimeError("No EtherCAT slaves found")

        # Find EP7342
        self.drive = None
        for s in self.master.slaves:
            if "EP7342" in s.name:
                self.drive = s
                break
        if not self.drive:
            self.drive = self.master.slaves[0]
            print("Warning: using first slave instead of EP7342")

        # Select position control mode
        mode_index = 0x8022 if channel == 1 else 0x8032
        self.drive.sdo_write(mode_index, 1, bytes([3]))  # 3 = Position controller

        # Map PDOs
        self.master.config_map()
        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        self.master.state_check(pysoem.OP_STATE, 5000)

        # Locate fields
        self._map_fields()

        print(f"EP7342 Axis (Ch{channel}) ready on {net_if}, unit={unit}")

    # ------------------ Helpers ------------------
    def _sdo_read(self, index, subindex, size=4):
        return self.drive.sdo_read(index, subindex, size)

    def _sdo_read_int(self, index, subindex, size, signed=False):
        data = self.drive.sdo_read(index, subindex, size)
        return int.from_bytes(bytes(data), "little", signed=signed)

    def _map_fields(self):
        """Parse PDO mapping and locate important fields."""
        # For simplicity, assume default mapping of EP7342:
        # Enable = 0x7020:01, Reset = 0x7020:02, Execute=0x7040:01, TargetPos=0x7040:11
        # Actual position often in 0x7041:11 (TxPDO)
        self.enable_off = 0  # byte offset in output (to be adapted by PDO parsing)
        self.reset_off = 0
        self.exec_off = 1
        self.targetpos_off = 4
        self.actualpos_off = 0

        # NOTE: 这里为了演示，简化成假定偏移。正式版应读取 0x1C12/0x1C13 解析映射。

    def _set_bit(self, byte_off, bit, val):
        b = self.drive.output[byte_off]
        if val:
            b |= (1 << bit)
        else:
            b &= ~(1 << bit)
        self.drive.output[byte_off] = b

    def _write_int32(self, byte_off, val):
        self.drive.output[byte_off:byte_off+4] = int(val).to_bytes(4, "little", signed=True)

    def _read_int32(self, byte_off):
        return int.from_bytes(bytes(self.drive.input[byte_off:byte_off+4]), "little", signed=True)

    # ------------------ Status / Errors ------------------
    def get_status(self):
        """
        解析状态字 (0x6020 for Ch1, 0x6030 for Ch2).
        返回 dict: {Ready, Active, Error, ...}
        """
        idx = 0x6020 if self.channel == 1 else 0x6030
        status = self._sdo_read_int(idx, 1, 2, signed=False)
        return {
            "Ready": bool(status & (1 << 0)),
            "Active": bool(status & (1 << 1)),
            "Error": bool(status & (1 << 2)),
            "Raw": status
        }

    # ------------------ High-level API ------------------
    def enable(self):
        self._set_bit(self.enable_off, 0, True)
        self._cycle()
        print("Axis enabled")

    def disable(self):
        self._set_bit(self.enable_off, 0, False)
        self._cycle()
        print("Axis disabled")

    def reset_fault(self):
        self._set_bit(self.reset_off, 1, True)
        self._cycle()
        time.sleep(0.05)
        self._set_bit(self.reset_off, 1, False)
        self._cycle()
        print("Fault reset sent")

    def move_to(self, pos_units: float):
        """
        Move to target position in user units (mm or deg).
        """
        pulses = self._units_to_pulses(pos_units)
        self._write_int32(self.targetpos_off, pulses)
        # Toggle Execute
        self._set_bit(self.exec_off, 0, True)
        self._cycle()
        time.sleep(0.02)
        self._set_bit(self.exec_off, 0, False)
        self._cycle()
        print(f"Move_to {pos_units}{self.unit} -> {pulses} pulses")

    def home(self):
        """简单的 home：把目标位置设为 0"""
        self.move_to(0)

    def stop(self):
        """停止（简单实现：清除 Execute，实际可拓展为快速停止）"""
        self._set_bit(self.exec_off, 0, False)
        self._cycle()
        print("Axis stopped")

    def get_actual_position(self):
        pulses = self._read_int32(self.actualpos_off)
        return self._pulses_to_units(pulses)

    # ------------------ Unit conversion ------------------
    def _units_to_pulses(self, val):
        if self.unit == "deg":
            return int(val / 360.0 * self.encoder_cpr * self.gear_ratio)
        elif self.unit == "mm":
            # 假设一圈 = 1 mm（需用户定义实际换算关系）
            return int(val / 1.0 * self.encoder_cpr * self.gear_ratio)
        else:
            return int(val)

    def _pulses_to_units(self, pulses):
        if self.unit == "deg":
            return pulses / (self.encoder_cpr * self.gear_ratio) * 360.0
        elif self.unit == "mm":
            return pulses / (self.encoder_cpr * self.gear_ratio) * 1.0
        else:
            return pulses

    # ------------------ Cyclic communication ------------------
    def _cycle(self):
        self.master.send_processdata()
        self.master.receive_processdata(2000)
        time.sleep(0.01)
************
#使用示例
from ep7342_lib import EP7342Axis

# 初始化：网口 eth0, 通道1, 编码器4096线, 齿轮比1:1, 单位=deg
axis = EP7342Axis("eth0", channel=1, encoder_cpr=4096, gear_ratio=1.0, unit="deg")

# 清故障 + 使能
axis.reset_fault()
axis.enable()

# 移动到 90 度
axis.move_to(90.0)

# 打印当前位置
print("Actual pos:", axis.get_actual_position(), "deg")

# 回零
axis.home()

# 停机并下电
axis.stop()
axis.disable()
**********************************************************************************************************
#保守测试版脚本（核心片段，带注释）

import pysoem
import struct
import time

class EPMotor:
    def __init__(self, ifname="eth0"):
        self.master = pysoem.Master()
        self.master.open(ifname)
        self.slaves = self.master.config_init()
        if self.slaves < 1:
            raise RuntimeError("Kein EtherCAT-Slave gefunden")
        self.drive = self.master.slaves[0]

        # 进入 PRE-OP 以便配置 SDO
        self.master.state = pysoem.PRE_OP_STATE
        self.master.write_state()

        # --- 硬件参数 ---
        self.encoder_cpr = 2500          # 编码器标称 2500 CPR
        self.ratio = 69                  # 齿轮比 69:1
        self.lead_mm = 5.0               # 假设丝杆导程 5 mm/转（需实测确认！）
        self.counts_per_rev = self.encoder_cpr * 4 * self.ratio  # 四倍频后总count
        self.counts_per_mm = self.counts_per_rev / self.lead_mm

        # 自动 PDO 映射
        self.master.config_map()
        self._map_fields()

        # 切到 OP
        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        self.master.state_check(pysoem.OP_STATE, 5000)

    # --- PDO映射解析 ---
    def _map_fields(self):
        # 简化：假设 Beckhoff 默认映射（可扩展为自动解析0x1C12/0x160x）
        self.targetpos_off = 0  # 实际应用必须根据 PDO 映射调整
        self.actualpos_off = 4

    # --- 读写辅助 ---
    def _write_int32(self, off, val):
        struct.pack_into('<i', self.drive.output, off, int(val))

    def _read_int32(self, off):
        return struct.unpack_from('<i', self.drive.input, off)[0]

    # --- 移动命令 ---
    def move_mm(self, mm):
        target_counts = int(mm * self.counts_per_mm)
        self._write_int32(self.targetpos_off, target_counts)
        self.master.send_processdata()
        self.master.receive_processdata(2000)

    def move_deg(self, deg):
        counts_per_deg = self.counts_per_rev / 360.0
        target_counts = int(deg * counts_per_deg)
        self._write_int32(self.targetpos_off, target_counts)
        self.master.send_processdata()
        self.master.receive_processdata(2000)

    def get_position_mm(self):
        counts = self._read_int32(self.actualpos_off)
        return counts / self.counts_per_mm

    def shutdown(self):
        self.master.state = pysoem.INIT_STATE
        self.master.write_state()
        self.master.close()

*****************************************************************************************************************
#电流写入
# —— 写入新的电流值（示例：更保守的限制） ——
new_imax_mA = 2000                             # 2.0 A 的持续电流上限
new_inom_mA = 1800                             # 1.8 A 的额定电流（示例）
drive.sdo_write(0x8020, 0x01, new_imax_mA.to_bytes(2, 'little', signed=False))
drive.sdo_write(0x8020, 0x02, new_inom_mA.to_bytes(2, 'little', signed=False))

# 写完再读回校验
imax_mA_verify = int.from_bytes(bytes(drive.sdo_read(0x8020, 0x01, 2)), 'little', False)
inom_mA_verify = int.from_bytes(bytes(drive.sdo_read(0x8020, 0x02, 2)), 'little', False)
print("Written Maximal current (mA) =", imax_mA_verify)
print("Written Nominal  current (mA) =", inom_mA_verify)
********************************************************************************************************************
#基本流程
# (1) 写操作模式 = CSP (0x08)
drive.sdo_write(0x6060, 0, (8).to_bytes(1, 'little'))

#使能
# 直接访问 output 数组的第 0 个字节
# 置 1：启用电机
self.drive.output[0] |= (1 << 0)

# 置 0：关闭电机
self.drive.output[0] &= ~(1 << 0)

# 别忘了同步过程数据
self.master.send_processdata()
self.master.receive_processdata()


# (2) 驱动状态机: Shutdown -> Switch On -> Enable Operation
# 写 0x6040 (Controlword)，通常按顺序写入 0x06, 0x07, 0x0F
drive.sdo_write(0x6040, 0, (6).to_bytes(2, 'little'))   # Shutdown
drive.sdo_write(0x6040, 0, (7).to_bytes(2, 'little'))   # Switch On
drive.sdo_write(0x6040, 0, (15).to_bytes(2, 'little'))  # Enable Operation

# (3) 设置目标位置 (例如 10000 counts)
drive.output[target_offset:target_offset+4] = (10000).to_bytes(4, 'little', signed=True)

# (4) 刷新过程数据
master.send_processdata()
master.receive_processdata(2000)

# (5) 读取实际位置
pos_counts = int.from_bytes(drive.input[actual_offset:actual_offset+4], 'little', signed=True)
********************************************************************************************************************
