import pysoem
import time

class MotorDemo:
    def __init__(self, iface='eth0'):
        self.master = pysoem.Master()
        self.master.open(iface)
        if self.master.config_init() == 0:
            raise Exception("No EtherCAT slaves found")
        self.drive = self.master.slaves[0]

        # 配置 PDO
        self.master.config_map()
        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        self.master.state_check(pysoem.OP_STATE, 5000)

    def enable_motor(self):
        # Enable = output[0], bit0
        self.drive.output[0] |= (1 << 0)
        self.master.send_processdata()
        self.master.receive_processdata()

    def disable_motor(self):
        self.drive.output[0] &= ~(1 << 0)
        self.master.send_processdata()
        self.master.receive_processdata()

    def move_to(self, target_counts):
        # 目标位置写到 output[4:8] (int32)
        self.drive.output[4:8] = int(target_counts).to_bytes(4, 'little', signed=True)
        self.master.send_processdata()
        self.master.receive_processdata()

    def get_position(self):
        # 实际位置从 input[4:8] (int32)
        self.master.send_processdata()
        self.master.receive_processdata()
        return int.from_bytes(bytes(self.drive.input[4:8]), 'little', signed=True)


if __name__ == "__main__":
    motor = MotorDemo('eth0')  # 替换成你的网卡接口名
    motor.enable_motor()
    print("Motor enabled")

    # 移动到某个位置 (例如 10000 counts)
    motor.move_to(10000)
    time.sleep(2)

    # 读取当前位置
    pos = motor.get_position()
    print(f"Actual Position: {pos}")

    # 关闭电机
    motor.disable_motor()
    print("Motor disabled")
