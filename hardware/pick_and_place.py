#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from dynamixel_sdk import *


# ============================================================
# DYNAMIXEL 기본 설정
# ============================================================

DEVICENAME = '/dev/cu.usbserial-FTAK8896'

BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

ARM_IDS = [11, 12, 13, 14]
GRIPPER_ID = 15
ALL_IDS = ARM_IDS + [GRIPPER_ID]

ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4

TORQUE_ENABLE = 1

PROFILE_VELOCITY_VALUE = 80

POSITION_THRESHOLD = 20
MOVE_TIMEOUT = 10.0


# ============================================================
# 2. ★★★ 여기만 직접 값 넣기 ★★★
# ============================================================

# 예:
# HOME = {
#     11: 2048,
#     12: 2048,
#     13: 2048,
#     14: 2048
# }

HOME = {
    11: None,
    12: None,
    13: None,
    14: None
}

# 물체 바로 위
PICK_ABOVE = {
    11: None,
    12: None,
    13: None,
    14: None
}

# 실제로 물체를 집는 위치
PICK = {
    11: None,
    12: None,
    13: None,
    14: None
}

# 내려놓을 장소 바로 위
PLACE_ABOVE = {
    11: None,
    12: None,
    13: None,
    14: None
}

# 실제로 물체를 내려놓는 위치
PLACE = {
    11: None,
    12: None,
    13: None,
    14: None
}

# Gripper
GRIPPER_OPEN = None
GRIPPER_CLOSE = None


# ============================================================
# 3. Ubuntu에서 U2D2 포트 자동 검색
# ============================================================

def find_device():

    candidates = (
        glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyACM*")
    )

    if len(candidates) == 0:
        print("[ERROR] U2D2 포트를 찾을 수 없습니다.")
        print("U2D2가 연결되어 있는지 확인하세요.")
        quit()

    print("발견된 포트:", candidates)

    # 보통 하나만 연결되어 있으므로 첫 번째 사용
    return candidates[0]


DEVICENAME = find_device()

print(f"사용 포트: {DEVICENAME}")


# ============================================================
# 4. PortHandler / PacketHandler
# ============================================================

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncWrite = GroupSyncWrite(
    portHandler,
    packetHandler,
    ADDR_GOAL_POSITION,
    LEN_GOAL_POSITION
)


# ============================================================
# 5. 입력값 확인
# ============================================================

def check_pose_values():

    poses = {
        "HOME": HOME,
        "PICK_ABOVE": PICK_ABOVE,
        "PICK": PICK,
        "PLACE_ABOVE": PLACE_ABOVE,
        "PLACE": PLACE
    }

    for pose_name, pose in poses.items():

        for motor_id, value in pose.items():

            if value is None:
                print(
                    f"\n[ERROR] {pose_name}의 "
                    f"ID {motor_id} 위치값이 입력되지 않았습니다."
                )
                quit()

    if GRIPPER_OPEN is None:
        print("\n[ERROR] GRIPPER_OPEN 값을 입력하세요.")
        quit()

    if GRIPPER_CLOSE is None:
        print("\n[ERROR] GRIPPER_CLOSE 값을 입력하세요.")
        quit()


# ============================================================
# 6. 포트 열기
# ============================================================

def open_port():

    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        quit()

    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        quit()


# ============================================================
# 7. Torque ON
# ============================================================

def enable_torque():

    print("\n===== TORQUE ON =====")

    for motor_id in ALL_IDS:

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler,
            motor_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE
        )

        if dxl_comm_result != COMM_SUCCESS:
            print(
                f"[ID {motor_id}]",
                packetHandler.getTxRxResult(dxl_comm_result)
            )

        elif dxl_error != 0:
            print(
                f"[ID {motor_id}]",
                packetHandler.getRxPacketError(dxl_error)
            )

        else:
            print(f"ID {motor_id}: Torque ON")


# ============================================================
# 8. Profile Velocity 설정
# ============================================================

def set_profile_velocity():

    print("\n===== PROFILE VELOCITY =====")

    for motor_id in ARM_IDS:

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
            portHandler,
            motor_id,
            ADDR_PROFILE_VELOCITY,
            PROFILE_VELOCITY_VALUE
        )

        if dxl_comm_result != COMM_SUCCESS:
            print(
                f"[ID {motor_id}]",
                packetHandler.getTxRxResult(dxl_comm_result)
            )

        elif dxl_error != 0:
            print(
                f"[ID {motor_id}]",
                packetHandler.getRxPacketError(dxl_error)
            )

        else:
            print(
                f"ID {motor_id}: "
                f"Profile Velocity = {PROFILE_VELOCITY_VALUE}"
            )


# ============================================================
# 9. 현재 위치 읽기
# ============================================================

def read_present_position(motor_id):

    present_position, dxl_comm_result, dxl_error = \
        packetHandler.read4ByteTxRx(
            portHandler,
            motor_id,
            ADDR_PRESENT_POSITION
        )

    if dxl_comm_result != COMM_SUCCESS:

        print(
            f"[ID {motor_id}] "
            f"{packetHandler.getTxRxResult(dxl_comm_result)}"
        )

        return None

    if dxl_error != 0:

        print(
            f"[ID {motor_id}] "
            f"{packetHandler.getRxPacketError(dxl_error)}"
        )

        return None

    return present_position


# ============================================================
# 10. 모든 모터 현재 위치 출력
# ============================================================

def print_present_positions():

    print("\n===== PRESENT POSITION =====")

    for motor_id in ALL_IDS:

        position = read_present_position(motor_id)

        if position is not None:
            print(f"ID {motor_id}: {position}")

    print("============================")


# ============================================================
# 11. Arm 목표 위치 전송
# ============================================================

def move_arm(target_positions):

    groupSyncWrite.clearParam()

    for motor_id, target_pos in target_positions.items():

        # Goal Position은 4 Byte이므로
        # 1 Byte씩 나누어서 전송
        param_goal_position = [
            DXL_LOBYTE(DXL_LOWORD(target_pos)),
            DXL_HIBYTE(DXL_LOWORD(target_pos)),
            DXL_LOBYTE(DXL_HIWORD(target_pos)),
            DXL_HIBYTE(DXL_HIWORD(target_pos))
        ]

        success = groupSyncWrite.addParam(
            motor_id,
            param_goal_position
        )

        if not success:
            print(
                f"[ERROR] ID {motor_id}: "
                "groupSyncWrite addParam 실패"
            )

    # 11~14번에게 한 번에 전송
    dxl_comm_result = groupSyncWrite.txPacket()

    if dxl_comm_result != COMM_SUCCESS:
        print(
            "[SyncWrite ERROR]",
            packetHandler.getTxRxResult(dxl_comm_result)
        )

    groupSyncWrite.clearParam()


# ============================================================
# 12. 목표 위치까지 도착했는지 기다리기
# ============================================================

def wait_until_arrived(target_positions):

    start_time = time.time()

    while True:

        all_arrived = True

        for motor_id, target_pos in target_positions.items():

            present_pos = read_present_position(motor_id)

            if present_pos is None:
                all_arrived = False
                continue

            position_error = abs(target_pos - present_pos)

            if position_error > POSITION_THRESHOLD:
                all_arrived = False

        # 모든 관절이 목표값 근처까지 도착
        if all_arrived:
            return True

        # 무한루프 방지
        if time.time() - start_time > MOVE_TIMEOUT:
            print("[WARNING] 목표 위치 도착 대기 시간 초과")
            print_present_positions()
            return False

        time.sleep(0.05)


# ============================================================
# 13. 특정 자세로 이동
# ============================================================

def go_to(name, pose):

    print(f"\n>>> {name} 이동")

    move_arm(pose)

    arrived = wait_until_arrived(pose)

    if arrived:
        print(f">>> {name} 도착")

    time.sleep(0.3)


# ============================================================
# 14. Gripper 이동
# ============================================================

def move_gripper(position):

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler,
        GRIPPER_ID,
        ADDR_GOAL_POSITION,
        position
    )

    if dxl_comm_result != COMM_SUCCESS:

        print(
            "[GRIPPER ERROR]",
            packetHandler.getTxRxResult(dxl_comm_result)
        )

        return

    if dxl_error != 0:

        print(
            "[GRIPPER ERROR]",
            packetHandler.getRxPacketError(dxl_error)
        )

        return


# ============================================================
# 15. Gripper 열기
# ============================================================

def gripper_open():

    print("\n>>> GRIPPER OPEN")

    move_gripper(GRIPPER_OPEN)

    time.sleep(1)


# ============================================================
# 16. Gripper 닫기
# ============================================================

def gripper_close():

    print("\n>>> GRIPPER CLOSE")

    move_gripper(GRIPPER_CLOSE)

    time.sleep(1)


# ============================================================
# 17. Pick & Place
# ============================================================

def pick_and_place():

    print("\n")
    print("==============================")
    print("     PICK & PLACE START")
    print("==============================")

    # --------------------------------
    # HOME
    # --------------------------------

    go_to(
        "HOME",
        HOME
    )

    # 물체를 집기 전에 그리퍼 열기
    gripper_open()

    # --------------------------------
    # PICK
    # --------------------------------

    go_to(
        "PICK ABOVE",
        PICK_ABOVE
    )

    go_to(
        "PICK",
        PICK
    )

    # 물체 집기
    gripper_close()

    # 다시 위로
    go_to(
        "PICK ABOVE",
        PICK_ABOVE
    )

    # --------------------------------
    # PLACE
    # --------------------------------

    go_to(
        "PLACE ABOVE",
        PLACE_ABOVE
    )

    go_to(
        "PLACE",
        PLACE
    )

    # 물체 놓기
    gripper_open()

    # 다시 위로
    go_to(
        "PLACE ABOVE",
        PLACE_ABOVE
    )

    # --------------------------------
    # HOME
    # --------------------------------

    go_to(
        "HOME",
        HOME
    )

    print("\n")
    print("==============================")
    print("     PICK & PLACE COMPLETE")
    print("==============================")


# ============================================================
# 18. MAIN
# ============================================================

if __name__ == "__main__":

    # 포즈 값 입력 여부부터 확인
    check_pose_values()

    # U2D2 연결
    open_port()

    try:

        # Torque ON
        enable_torque()

        # 움직이는 속도 설정
        set_profile_velocity()

        # 현재 모터 위치 확인
        print_present_positions()

        print("\n주의:")
        print("처음 테스트는 물체 없이 진행하세요.")
        print("비상 시 바로 전원을 끌 수 있게 준비하세요.")

        input(
            "\nEnter를 누르면 Pick & Place를 시작합니다..."
        )

        # 실제 동작
        pick_and_place()

    except KeyboardInterrupt:

        print("\n\n사용자가 프로그램을 중지했습니다.")

    finally:

        # 통신 포트만 닫음
        # Torque를 여기서 강제로 OFF하지 않는 이유:
        # 팔이 갑자기 아래로 떨어질 수 있기 때문
        portHandler.closePort()

        print("\nPort closed.")