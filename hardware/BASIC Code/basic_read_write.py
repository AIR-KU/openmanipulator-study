#!/usr/bin/env python3

from dynamixel_sdk import *

# 포트와 Protocol 설정
portHandler = PortHandler("/dev/cu.usbserial-FTAK8896")
packetHandler = PacketHandler(2.0)

# 포트 열기
if portHandler.openPort():
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    exit()

# 통신속도 설정
if portHandler.setBaudRate(1000000):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    exit()

# 제어할 다이나믹셀 설정
dxl_id = 15

# Torque Enable 주소
torque_on_address = 64

# Torque ON
data = 1

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler,
    dxl_id,
    torque_on_address,
    data
)

if dxl_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print(packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

# 목표 위치를 반복해서 입력
while True:
    try:
        target_position = int(
            input("Enter target position (0 ~ 4095, -1 to exit): ")
        )
    except ValueError:
        print("Please enter an integer.")
        continue

    # -1을 입력하면 종료
    if target_position == -1:
        break

    # 위치 범위 확인
    elif target_position < 0 or target_position > 4095:
        print("Position must be between 0 and 4095.")
        continue

    # Goal Position 주소
    goal_position_address = 116

    # 목표 위치 쓰기
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler,
        dxl_id,
        goal_position_address,
        target_position
    )

    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))

    # 목표 위치에 도착할 때까지 현재 위치 읽기
    while True:
        present_position_address = 132

        present_position, dxl_comm_result, dxl_error = (
            packetHandler.read4ByteTxRx(
                portHandler,
                dxl_id,
                present_position_address
            )
        )

        if dxl_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(packetHandler.getRxPacketError(dxl_error))

        print(f"Current Position: {present_position}")

        # 목표 위치와 현재 위치의 차이가 10 이 10 이하면 도착
        if abs(target_position - present_position) <= 10:
            break

# Torque OFF
data = 0

packetHandler.write1ByteTxRx(
    portHandler,
    dxl_id,
    torque_on_address,
    data
)

# 포트 닫기
portHandler.closePort() 
