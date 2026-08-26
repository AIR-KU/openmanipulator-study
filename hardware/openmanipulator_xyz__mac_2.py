#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
from pynput import keyboard
from dynamixel_sdk import *


# ==========================================================
# 0. macOS 키 입력
# ==========================================================
_pressed_keys = set()


def _key_name(key):
    try:
        if key.char is not None:
            return key.char.lower()
    except AttributeError:
        pass

    special_keys = {
        keyboard.Key.esc: "esc",
    }
    return special_keys.get(key)


def _on_press(key):
    name = _key_name(key)
    if name:
        _pressed_keys.add(name)


def _on_release(key):
    name = _key_name(key)
    if name:
        _pressed_keys.discard(name)


def is_pressed(key_name):
    return key_name.lower() in _pressed_keys


_keyboard_listener = keyboard.Listener(
    on_press=_on_press,
    on_release=_on_release,
)
_keyboard_listener.start()


# ==========================================================
# 1. 하드웨어 / 제어 설정
# ==========================================================
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
LEN_GOAL_POSITION = 4

PROFILE_VELOCITY_VALUE = 80
PROFILE_ACCELERATION_VALUE = 0
GRIPPER_VELOCITY_VALUE = 30
GRIPPER_ACCELERATION_VALUE = 10
GRAB_FORCE = 100

XYZ_STEP = 1.0
MAX_IK_PULSE_JUMP = 180

BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DEVICENAME = "/dev/cu.usbserial-FTAK8896"

DXL_ID_LIST = [11, 12, 13, 14, 15]
ARM_ID_LIST = [11, 12, 13, 14]
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

MIN_POS = 0
MAX_POS = 4095


# ==========================================================
# 2. 로봇 기구학 파라미터 (mm)
#    ROBOTIS OpenManipulator-X 공식 기구 정의 기준
# ==========================================================
# world -> joint1 : (12, 0, 17) mm
# joint1 -> joint2: (0, 0, 59.5) mm
#
# 중요:
# WORLD_X_OFFSET(12 mm)는 Joint 1과 함께 회전하는 링크 길이가 아니라
# world 좌표계에서 Joint 1 회전축 자체가 +X로 12 mm 떨어져 있다는 뜻이다.
WORLD_X_OFFSET = 12.0
SHOULDER_Z = 17.0 + 59.5      # world -> joint2 높이 = 76.5 mm

# joint2 -> joint3: (24, 0, 128) mm
LINK2_X = 24.0
LINK2_Z = 128.0
L2 = math.hypot(LINK2_X, LINK2_Z)

# joint3 -> joint4: (124, 0, 0) mm
L3 = 124.0

# joint4 -> TCP: (126, 0, 0) mm
L4 = 126.0

# joint2 -> joint3 벡터가 +r 수평축과 이루는 기본 각도
LINK2_ANGLE = math.atan2(LINK2_Z, LINK2_X)

# XYZ 모드에서는 TCP pitch를 world 기준 수평으로 유지
# FK에서 tool_pitch = -(theta2 + theta3 + theta4)
TOOL_PITCH_TARGET = 0.0


# ==========================================================
# 3. Dynamixel pulse <-> 관절각 θ
# ==========================================================
PULSE_CENTER = {
    11: 2048,
    12: 2048,
    13: 2048,
    14: 2048,
}

JOINT_DIRECTION = {
    11: +1.0,
    12: +1.0,
    13: +1.0,
    14: +1.0,
}

PULSE_TO_RAD = 2.0 * math.pi / 4096.0
RAD_TO_PULSE = 4096.0 / (2.0 * math.pi)

# ROBOTIS 공식 예제 ready pose:
# joint1 = 0 deg, joint2 = -60 deg, joint3 = +20 deg, joint4 = +40 deg
# 현재 코드의 2048 pulse = 0 deg 기준으로 환산
INITIAL_POSITIONS = {
    11: 2048,
    12: 1365,
    13: 2276,
    14: 2503,
    15: 1024,
}

FINAL_POSITIONS = {
    11: 0,
    12: 720,
    13: 3030,
    14: 840,
    15: 512,
}

current_goals = INITIAL_POSITIONS.copy()

GRIPPER_OPEN = 1024
GRIPPER_CLOSED = 3500

# ROBOTIS 공식 OpenManipulator-X 관절 제한 (rad)
JOINT_LIMITS = {
    11: (-math.pi, math.pi),
    12: (-2.05, math.pi / 2.0),
    13: (-math.pi / 2.0, 1.53),
    14: (-1.8, 2.0),
}


def pulse_to_joint_angle(dxl_id, pulse):
    return (
        JOINT_DIRECTION[dxl_id]
        * (pulse - PULSE_CENTER[dxl_id])
        * PULSE_TO_RAD
    )


def joint_angle_to_pulse(dxl_id, theta):
    return int(round(
        PULSE_CENTER[dxl_id]
        + JOINT_DIRECTION[dxl_id] * theta * RAD_TO_PULSE
    ))


def pulses_to_joint_angles(pulses):
    return tuple(
        pulse_to_joint_angle(dxl_id, pulses[dxl_id])
        for dxl_id in ARM_ID_LIST
    )


def joint_angles_to_pulses(q):
    return {
        dxl_id: joint_angle_to_pulse(dxl_id, theta)
        for dxl_id, theta in zip(ARM_ID_LIST, q)
    }


# ==========================================================
# 4. Forward Kinematics: θ -> XYZ
# ==========================================================
def forward_kinematics_from_q(q):
    theta1, theta2, theta3, theta4 = q

    # joint2 -> joint3 링크의 world/local 수평면 기준 방향
    phi2 = LINK2_ANGLE - theta2

    # joint3 -> joint4, joint4 -> TCP 방향
    phi3 = -(theta2 + theta3)
    phi4 = -(theta2 + theta3 + theta4)

    # Joint 1 회전축을 기준으로 한 수평 반경
    r_from_joint1 = (
        L2 * math.cos(phi2)
        + L3 * math.cos(phi3)
        + L4 * math.cos(phi4)
    )

    # world 좌표계 Z
    z = (
        SHOULDER_Z
        + L2 * math.sin(phi2)
        + L3 * math.sin(phi3)
        + L4 * math.sin(phi4)
    )

    # Joint 1 회전축은 world 원점에서 X=12 mm에 고정되어 있다.
    # 12 mm 오프셋 자체는 theta1과 함께 회전시키지 않는다.
    x = WORLD_X_OFFSET + r_from_joint1 * math.cos(theta1)
    y = r_from_joint1 * math.sin(theta1)

    return x, y, z, phi4


def forward_kinematics_from_pulses(pulses):
    return forward_kinematics_from_q(pulses_to_joint_angles(pulses))


# ==========================================================
# 5. Analytic Inverse Kinematics: XYZ -> θ
# ==========================================================
def _clamp_acos_arg(value):
    return max(-1.0, min(1.0, value))


def analytic_inverse_kinematics_candidates(x, y, z):
    """
    목표 TCP (x, y, z)에 대한 가능한 IK 해들을 반환한다.
    OpenManipulator-X의 elbow branch가 둘 존재할 수 있으므로
    둘 다 계산한 뒤 xyz_to_safe_pulses()에서 현재 자세에 가까운 해를 선택한다.
    """

    # ------------------------------------------------------
    # 1) world 원점이 아니라 Joint 1 회전축 기준으로 XY를 변환
    # ------------------------------------------------------
    x_from_joint1 = x - WORLD_X_OFFSET
    y_from_joint1 = y

    theta1 = math.atan2(y_from_joint1, x_from_joint1)
    r_from_joint1 = math.hypot(x_from_joint1, y_from_joint1)

    # ------------------------------------------------------
    # 2) TCP에서 L4를 제거하여 Joint 4(손목) 목표점 계산
    # ------------------------------------------------------
    wrist_r = (
        r_from_joint1
        - L4 * math.cos(TOOL_PITCH_TARGET)
    )
    wrist_z = (
        z
        - SHOULDER_Z
        - L4 * math.sin(TOOL_PITCH_TARGET)
    )

    D2 = wrist_r**2 + wrist_z**2
    D = math.sqrt(D2)

    if D < 1e-9:
        return []

    min_reach = abs(L2 - L3)
    max_reach = L2 + L3

    if not (min_reach <= D <= max_reach):
        print(
            f"[경고] 도달 불가능 좌표: D={D:.2f} mm, "
            f"허용범위={min_reach:.2f}~{max_reach:.2f} mm"
        )
        return []

    # ------------------------------------------------------
    # 3) 표준 2-link planar IK
    #
    # phi2 : L2의 실제 방향
    # phi3 : L3의 실제 방향
    #
    # q_rel = phi3 - phi2 에 대해 두 branch를 모두 계산
    # ------------------------------------------------------
    cos_q_rel = (
        D2 - L2**2 - L3**2
    ) / (2.0 * L2 * L3)

    q_rel_abs = math.acos(_clamp_acos_arg(cos_q_rel))

    candidates = []

    for q_rel in (q_rel_abs, -q_rel_abs):
        phi2 = (
            math.atan2(wrist_z, wrist_r)
            - math.atan2(
                L3 * math.sin(q_rel),
                L2 + L3 * math.cos(q_rel),
            )
        )
        phi3 = phi2 + q_rel

        # FK 정의를 역으로 풀기
        # phi2 = LINK2_ANGLE - theta2
        theta2 = LINK2_ANGLE - phi2

        # phi3 = -(theta2 + theta3)
        theta3 = -phi3 - theta2

        # TOOL_PITCH_TARGET = -(theta2 + theta3 + theta4)
        theta4 = (
            -TOOL_PITCH_TARGET
            - theta2
            - theta3
        )

        candidates.append(
            (theta1, theta2, theta3, theta4)
        )

    return candidates


def _within_joint_limits(q):
    for dxl_id, theta in zip(ARM_ID_LIST, q):
        theta_min, theta_max = JOINT_LIMITS[dxl_id]
        if not (theta_min <= theta <= theta_max):
            return False
    return True


def xyz_to_safe_pulses(x, y, z, current_pulses):
    q_candidates = analytic_inverse_kinematics_candidates(x, y, z)

    if not q_candidates:
        return None

    valid_candidates = []

    for q_target in q_candidates:
        # 공식 관절 범위 검사
        if not _within_joint_limits(q_target):
            continue

        # FK/IK 수식 일치 검사
        fk_x, fk_y, fk_z, _ = forward_kinematics_from_q(q_target)
        formula_error = math.sqrt(
            (fk_x - x) ** 2
            + (fk_y - y) ** 2
            + (fk_z - z) ** 2
        )

        if formula_error > 1e-6:
            continue

        pulse_targets = joint_angles_to_pulses(q_target)

        # pulse 범위 검사
        if any(
            not (MIN_POS <= pulse <= MAX_POS)
            for pulse in pulse_targets.values()
        ):
            continue

        # pulse 반올림 이후 XYZ 오차 검사
        q_quantized = pulses_to_joint_angles(pulse_targets)
        qx, qy, qz, _ = forward_kinematics_from_q(q_quantized)

        quantized_error = math.sqrt(
            (qx - x) ** 2
            + (qy - y) ** 2
            + (qz - z) ** 2
        )

        if quantized_error > 1.0:
            continue

        # 현재 자세와의 pulse 차이를 비용으로 사용
        jump_cost = sum(
            abs(pulse_targets[dxl_id] - current_pulses[dxl_id])
            for dxl_id in ARM_ID_LIST
        )

        max_jump = max(
            abs(pulse_targets[dxl_id] - current_pulses[dxl_id])
            for dxl_id in ARM_ID_LIST
        )

        valid_candidates.append(
            (jump_cost, max_jump, pulse_targets, q_target)
        )

    if not valid_candidates:
        print("[경고] 현재 XYZ에서 관절 범위 안에 들어오는 IK 해가 없습니다.")
        return None

    # 현재 자세와 가장 가까운 IK branch 선택
    valid_candidates.sort(key=lambda item: item[0])
    _, max_jump, pulse_targets, q_target = valid_candidates[0]

    # 한 제어 step에서 지나치게 큰 관절 이동 차단
    if max_jump > MAX_IK_PULSE_JUMP:
        for dxl_id in ARM_ID_LIST:
            jump = abs(
                pulse_targets[dxl_id]
                - current_pulses[dxl_id]
            )
            if jump == max_jump:
                print(
                    f"[경고] IK 이동량이 너무 큽니다. "
                    f"ID={dxl_id}, Δ={jump} pulse"
                )
                break
        return None

    return pulse_targets


# ==========================================================
# 6. DYNAMIXEL 통신
# ==========================================================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(
    portHandler,
    packetHandler,
    ADDR_GOAL_POSITION,
    LEN_GOAL_POSITION,
)


def add_sync_write_param(motor_id, target_pos):
    target_pos = max(MIN_POS, min(MAX_POS, int(target_pos)))

    param_goal_position = [
        DXL_LOBYTE(DXL_LOWORD(target_pos)),
        DXL_HIBYTE(DXL_LOWORD(target_pos)),
        DXL_LOBYTE(DXL_HIWORD(target_pos)),
        DXL_HIBYTE(DXL_HIWORD(target_pos)),
    ]

    groupSyncWrite.addParam(motor_id, param_goal_position)


def send_arm_positions(pulse_targets):
    for dxl_id in ARM_ID_LIST:
        current_goals[dxl_id] = pulse_targets[dxl_id]
        add_sync_write_param(dxl_id, pulse_targets[dxl_id])

    groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()


def move_to_pose_sequentially(position_dict, motor_order, delay=1.0):
    for dxl_id in motor_order:
        add_sync_write_param(dxl_id, position_dict[dxl_id])
        groupSyncWrite.txPacket()
        groupSyncWrite.clearParam()
        time.sleep(delay)


# ==========================================================
# 7. 초기화
# ==========================================================
if not portHandler.openPort():
    raise RuntimeError("DYNAMIXEL 통신 포트를 열지 못했습니다.")

if not portHandler.setBaudRate(BAUDRATE):
    portHandler.closePort()
    raise RuntimeError("DYNAMIXEL Baudrate 설정에 실패했습니다.")

print("[시스템] 통신 포트 연결 성공!")

# 11~14번: 위치 제어
for dxl_id in ARM_ID_LIST:
    packetHandler.write4ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_PROFILE_VELOCITY,
        PROFILE_VELOCITY_VALUE,
    )
    packetHandler.write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        TORQUE_ENABLE,
    )

# 15번: 전류 기반 위치 제어 그리퍼
packetHandler.write1ByteTxRx(
    portHandler, 15, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
)
packetHandler.write1ByteTxRx(
    portHandler, 15, ADDR_OPERATING_MODE, 5
)
packetHandler.write1ByteTxRx(
    portHandler, 15, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
)
packetHandler.write2ByteTxRx(
    portHandler, 15, ADDR_GOAL_CURRENT, GRAB_FORCE
)
packetHandler.write4ByteTxRx(
    portHandler, 15, ADDR_PROFILE_ACCELERATION, GRIPPER_ACCELERATION_VALUE
)
packetHandler.write4ByteTxRx(
    portHandler, 15, ADDR_PROFILE_VELOCITY, GRIPPER_VELOCITY_VALUE
)

print("\n[시스템] 초기 자세로 이동합니다...")
move_to_pose_sequentially(
    INITIAL_POSITIONS,
    [11, 12, 13, 14, 15],
    delay=1.0,
)

# 초기 자세의 XYZ를 현재 Cartesian 목표로 사용
target_x, target_y, target_z, target_pitch = forward_kinematics_from_pulses(
    current_goals
)

print("\n================== 🤖 XYZ 조종기 ==================")
print(" W / S : X축 +/-")
print(" A / D : Y축 +/-")
print(" Q / E : Z축 +/-")
print(" C     : 그리퍼 열기 / 닫기")
print(" ESC   : 안전 복귀 후 종료")
print("===================================================")
print(
    f"현재 TCP 계산값: X={target_x:.1f}, Y={target_y:.1f}, "
    f"Z={target_z:.1f} mm"
)
print(
    f"[TCP 측정 기준] world 원점 기준. "
    f"Joint1 회전축은 X={WORLD_X_OFFSET:.1f} mm, "
    f"Joint2 높이는 Z={SHOULDER_Z:.1f} mm"
)


# ==========================================================
# 8. XYZ 메인 제어 루프
# ==========================================================
c_pressed_last = False
gripper_is_closed = False

try:
    while True:
        # --------------------------------------------------
        # ESC: 안전 복귀 후 종료
        # --------------------------------------------------
        if is_pressed("esc"):
            print("\n[시스템] 마지막 자세로 복귀합니다...")
            move_to_pose_sequentially(
                FINAL_POSITIONS,
                [15, 14, 13, 12, 11],
                delay=1.0,
            )
            break

        # --------------------------------------------------
        # C: 그리퍼 토글
        # --------------------------------------------------
        c_pressed = is_pressed("c")

        if c_pressed and not c_pressed_last:
            if not gripper_is_closed:
                packetHandler.write4ByteTxRx(
                    portHandler,
                    15,
                    ADDR_GOAL_POSITION,
                    GRIPPER_CLOSED,
                )
                gripper_is_closed = True
                print(">> ✊ [그리퍼] 닫기")
            else:
                packetHandler.write4ByteTxRx(
                    portHandler,
                    15,
                    ADDR_GOAL_POSITION,
                    GRIPPER_OPEN,
                )
                gripper_is_closed = False
                print(">> 🖐️ [그리퍼] 열기")

        c_pressed_last = c_pressed

        # --------------------------------------------------
        # XYZ 키 입력
        # --------------------------------------------------
        dx = 0.0
        dy = 0.0
        dz = 0.0

        if is_pressed("w"):
            dx += XYZ_STEP
        elif is_pressed("s"):
            dx -= XYZ_STEP

        if is_pressed("a"):
            dy += XYZ_STEP
        elif is_pressed("d"):
            dy -= XYZ_STEP

        if is_pressed("q"):
            dz += XYZ_STEP
        elif is_pressed("e"):
            dz -= XYZ_STEP

        # --------------------------------------------------
        # 목표 XYZ -> IK -> θ -> pulse -> 모터 전송
        # --------------------------------------------------
        if dx != 0.0 or dy != 0.0 or dz != 0.0:
            next_x = target_x + dx
            next_y = target_y + dy
            next_z = target_z + dz

            pulse_targets = xyz_to_safe_pulses(
                next_x,
                next_y,
                next_z,
                current_goals,
            )

            if pulse_targets is not None:
                target_x = next_x
                target_y = next_y
                target_z = next_z

                send_arm_positions(pulse_targets)

        time.sleep(0.03)

finally:
    for dxl_id in DXL_ID_LIST:
        packetHandler.write1ByteTxRx(
            portHandler,
            dxl_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_DISABLE,
        )

    portHandler.closePort()
    print("\n[시스템] 안전하게 종료되었습니다.")
