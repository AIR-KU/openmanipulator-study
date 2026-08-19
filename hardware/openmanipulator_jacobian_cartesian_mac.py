#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
import numpy as np
from pynput import keyboard

from dynamixel_sdk import *

# --- macOS 키 입력 처리 (pynput) ---
_pressed_keys = set()

def _key_name(key):
    """pynput Key/KeyCode를 기존 is_pressed()와 비슷한 문자열로 변환"""
    try:
        if key.char is not None:
            return key.char.lower()
    except AttributeError:
        pass

    special_keys = {
        keyboard.Key.space: 'space',
        keyboard.Key.esc: 'esc',
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
    on_release=_on_release
)
_keyboard_listener.start()

# ==========================================
# 1. 하드웨어 및 기본 설정
# ==========================================
# --- 다이나믹셀 컨트롤 테이블 주소 ---
ADDR_OPERATING_MODE         = 11    # 동작 모드 (1 Byte)
ADDR_TORQUE_ENABLE          = 64    # 토크 온/오프 (1 Byte)
ADDR_GOAL_CURRENT           = 102   # 목표 전류 / 쥐는 힘 (2 Byte)
ADDR_PROFILE_ACCELERATION   = 108   # 가속도 프로파일 (4 Byte)
ADDR_PROFILE_VELOCITY       = 112   # 속도 프로파일 (4 Byte)
ADDR_GOAL_POSITION          = 116   # 목표 위치 (4 Byte)
ADDR_PRESENT_POSITION       = 132   # 현재 위치 (4 Byte)
LEN_GOAL_POSITION           = 4     # 목표 위치 데이터 길이 (4 Byte)

# --- 제어 튜닝 파라미터 ---
PROFILE_VELOCITY_VALUE      = 80   # 최고 이동 속도 (부드러운 제어를 위해 상향)
PROFILE_ACCELERATION_VALUE  = 0    # 가속도 (작을수록 스무스하게 출발/정지)
GRIPPER_VELOCITY_VALUE      = 30    # 집게 전용 속도 (아주 천천히 스르륵)
GRIPPER_ACCELERATION_VALUE  = 10    # 집게 전용 가속도 (더욱 부드럽게 출발)
GRAB_FORCE                  = 100   # 집게 파지력 (0~1193, 150은 연약한 물체용)

JOINT_STEP = 20      # 관절 모드: 1회 누름당 펄스 변화량
XYZ_STEP   = 2.0     # Cartesian 모드: 제어 루프 1회당 이동 거리 (mm)

# --- Jacobian Differential IK 설정 ---
PITCH_SCALE = 100.0                 # pitch(rad)를 mm 수준으로 스케일링
PINV_RCOND = 1e-3                   # pseudo-inverse 특이값 컷오프
MAX_DQ_RAD = math.radians(2.0)      # 한 루프에서 관절 최대 변화량

# --- 통신 설정 ---
BAUDRATE            = 1000000   
PROTOCOL_VERSION    = 2.0
DEVICENAME          = '/dev/cu.usbserial-FTAK8896'    

DXL_ID_LIST         = [11, 12, 13, 14, 15]
TORQUE_ENABLE       = 1
TORQUE_DISABLE      = 0

# --- 로봇 기구학 링크 길이 (단위: mm) ---
L1 = 130.23  
L2 = 124.0  
L3 = 126.0  

INITIAL_POSITIONS = {11: 2048, 12: 720, 13: 3030, 14: 2390, 15: 1024}
FINAL_POSITIONS   = {11: 0, 12: 720, 13: 3030, 14: 840, 15: 512}

current_goals = INITIAL_POSITIONS.copy()
MIN_POS, MAX_POS = 0, 4095

# --- 스마트 그리퍼 설정 ---
GRIPPER_OPEN   = 1024   # 집게 열림 위치
GRIPPER_CLOSED = 3500   # 집게 닫힘 목표 위치 (닿으면 멈춤)

# ==========================================
# 2. 기구학 (Kinematics) + Jacobian 엔진
# ==========================================

def pulses_to_joint_angles(p11, p12, p13, p14):
    """
    Dynamixel pulse -> 기구학 관절각 q(rad)

    기존 코드의 FK와 동일한 zero/부호 규칙을 사용한다.
    """
    q1 = (p11 - 2048) / 4096.0 * 2.0 * math.pi
    q2 = -(p12 - 2048) / 4096.0 * 2.0 * math.pi
    q3 = -(p13 - 2048) / 4096.0 * 2.0 * math.pi
    q4 = -(p14 - 2048) / 4096.0 * 2.0 * math.pi

    return np.array([q1, q2, q3, q4], dtype=float)


def joint_angles_to_pulses(q):
    """
    기구학 관절각 q(rad) -> Dynamixel pulse
    """
    q1, q2, q3, q4 = q

    return {
        11: int(round(q1 * 4096.0 / (2.0 * math.pi))) + 2048,
        12: int(round(-q2 * 4096.0 / (2.0 * math.pi))) + 2048,
        13: int(round(-q3 * 4096.0 / (2.0 * math.pi))) + 2048,
        14: int(round(-q4 * 4096.0 / (2.0 * math.pi))) + 2048,
    }


def forward_kinematics_from_q(q):
    """
    q(rad) -> [X, Y, Z, Pitch]

    X, Y, Z : mm
    Pitch   : rad
    """
    q1, q2, q3, q4 = q

    q23 = q2 + q3
    q234 = q2 + q3 + q4

    # base 축에서 End Effector까지의 수평 반경
    r = (
        L1 * math.cos(q2)
        + L2 * math.cos(q23)
        + L3 * math.cos(q234)
    )

    z = (
        L1 * math.sin(q2)
        + L2 * math.sin(q23)
        + L3 * math.sin(q234)
    )

    x = r * math.cos(q1)
    y = r * math.sin(q1)
    pitch = q234

    return np.array([x, y, z, pitch], dtype=float)


def forward_kinematics(p11, p12, p13, p14):
    """
    기존 코드와 같은 형태의 FK 인터페이스.
    pulse -> X, Y, Z, Pitch
    """
    q = pulses_to_joint_angles(p11, p12, p13, p14)
    return tuple(forward_kinematics_from_q(q))


def calculate_jacobian(q):
    """
    현재 자세 q에서의 Jacobian.

    task vector:
        [X, Y, Z, PITCH_SCALE * Pitch]

    즉:
        delta_task = J(q) @ delta_q

    4번째 행까지 넣는 이유:
        OpenManipulator-X arm joint가 4개이고
        XYZ만 제어하면 남는 1자유도로 손목 자세가 변할 수 있기 때문.
        여기서는 ΔPitch = 0으로 두어 XYZ 이동 중 현재 gripper pitch를 유지한다.
    """
    q1, q2, q3, q4 = q

    q23 = q2 + q3
    q234 = q2 + q3 + q4

    c1 = math.cos(q1)
    s1 = math.sin(q1)

    c2 = math.cos(q2)
    s2 = math.sin(q2)

    c23 = math.cos(q23)
    s23 = math.sin(q23)

    c234 = math.cos(q234)
    s234 = math.sin(q234)

    r = L1 * c2 + L2 * c23 + L3 * c234

    # dr / dq
    dr_dq2 = -L1 * s2 - L2 * s23 - L3 * s234
    dr_dq3 = -L2 * s23 - L3 * s234
    dr_dq4 = -L3 * s234

    # dz / dq
    dz_dq2 = L1 * c2 + L2 * c23 + L3 * c234
    dz_dq3 = L2 * c23 + L3 * c234
    dz_dq4 = L3 * c234

    J = np.array([
        #               q1              q2               q3               q4
        [-r * s1, dr_dq2 * c1, dr_dq3 * c1, dr_dq4 * c1],  # dx
        [ r * c1, dr_dq2 * s1, dr_dq3 * s1, dr_dq4 * s1],  # dy
        [      0.0,      dz_dq2,      dz_dq3,      dz_dq4],  # dz
        [      0.0,  PITCH_SCALE,  PITCH_SCALE,  PITCH_SCALE],  # dPitch
    ], dtype=float)

    return J


def jacobian_cartesian_step(current_pulses, dx, dy, dz):
    """
    End Effector의 작은 Cartesian 이동량을
    Jacobian pseudo-inverse를 이용해 관절 변화량으로 변환.

        Δq = J(q)^+ Δx

    여기서는 gripper의 현재 pitch를 유지하므로
    ΔPitch = 0 으로 둔다.
    """
    q = pulses_to_joint_angles(
        current_pulses[11],
        current_pulses[12],
        current_pulses[13],
        current_pulses[14],
    )

    J = calculate_jacobian(q)

    delta_task = np.array([
        dx,
        dy,
        dz,
        0.0,   # pitch 변화 없음
    ], dtype=float)

    # Moore-Penrose pseudo-inverse
    J_pinv = np.linalg.pinv(J, rcond=PINV_RCOND)

    # Differential IK 핵심식
    dq = J_pinv @ delta_task

    # 특이점 근처에서 갑자기 큰 관절 명령이 나가는 것을 제한
    max_abs_dq = float(np.max(np.abs(dq)))
    if max_abs_dq > MAX_DQ_RAD:
        dq = dq * (MAX_DQ_RAD / max_abs_dq)

    q_target = q + dq
    pulse_targets = joint_angles_to_pulses(q_target)

    # 안전 범위를 벗어난 경우 해당 step은 취소
    for dxl_id, pulse in pulse_targets.items():
        if not (MIN_POS <= pulse <= MAX_POS):
            print(
                f"[경고] Jacobian 결과가 위치 범위를 벗어났습니다. "
                f"ID={dxl_id}, target={pulse}"
            )
            return None

    return pulse_targets


# ==========================================
# 3. 통신 및 스마트 초기화 설정
# ==========================================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
    print("[시스템] 통신 포트 연결 성공!")
else: quit()

# --- 1~4축 모터 초기화 (위치 제어 모드) ---
for dxl_id in [11, 12, 13, 14]:
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY_VALUE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# --- 5축 집게(15번) 스마트 그리퍼 세팅 ---
packetHandler.write1ByteTxRx(portHandler, 15, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)      
packetHandler.write1ByteTxRx(portHandler, 15, ADDR_OPERATING_MODE, 5)                  # 전류 기반 위치 제어 모드(5)
packetHandler.write1ByteTxRx(portHandler, 15, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)       
packetHandler.write2ByteTxRx(portHandler, 15, ADDR_GOAL_CURRENT, GRAB_FORCE)           # 쥐는 힘 설정
packetHandler.write4ByteTxRx(portHandler, 15, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION_VALUE)
packetHandler.write4ByteTxRx(portHandler, 15, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY_VALUE)

def add_sync_write_param(motor_id, target_pos):
    target_pos = max(MIN_POS, min(MAX_POS, target_pos))
    param_goal_position = [
        DXL_LOBYTE(DXL_LOWORD(target_pos)), DXL_HIBYTE(DXL_LOWORD(target_pos)),
        DXL_LOBYTE(DXL_HIWORD(target_pos)), DXL_HIBYTE(DXL_HIWORD(target_pos))
    ]
    groupSyncWrite.addParam(motor_id, param_goal_position)

print("\n[시스템] 초기 자세로 순차 이동합니다...")
for dxl_id in DXL_ID_LIST:
    add_sync_write_param(dxl_id, INITIAL_POSITIONS[dxl_id])
    groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()
    time.sleep(1.0)

# ==========================================
# 4. 메인 제어 루프
# ==========================================
control_mode = "JOINT"
space_pressed_last = False
c_pressed_last = False
gripper_is_closed = False

target_x, target_y, target_z, target_pitch = 0, 0, 0, 0

print("\n================== 🤖 모드 전환형 조종기 ==================")
print(" [Spacebar] 모드 전환 (관절 제어 <-> Cartesian/Jacobian 제어)")
print("\n [로봇 팔 이동 제어]")
print("  관절 모드 : 11~14번 모터 (Q/A, W/S, E/D, R/F)")
print("  Cartesian : X축(W/S) / Y축(A/D) / Z축(Q/E), Pitch 자동 유지")
print("\n [스마트 그리퍼 제어]")
print("  [ C ] 키 : 집게 열기 / 닫기 (토글 방식, 스마트 파지)")
print("\n  [ ESC ] 키 : 안전 복귀 및 프로그램 종료")
print("==========================================================\n")

try:
    while True:
        # --- 종료 시퀀스 ---
        if is_pressed('esc'):
            print("\n[시스템] 마지막 자세로 복귀합니다...")
            for dxl_id in [15, 14, 13, 12, 11]:
                add_sync_write_param(dxl_id, FINAL_POSITIONS[dxl_id])
                groupSyncWrite.txPacket()
                groupSyncWrite.clearParam()
                
                if dxl_id == 11:
                    stuck_count, last_position = 0, -1
                    for _ in range(4):
                        time.sleep(0.5)
                        pos, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
                        if abs(pos - FINAL_POSITIONS[dxl_id]) < 20: break
                        if last_position != -1 and abs(pos - last_position) <= 5:
                            stuck_count += 1
                            if stuck_count >= 2: break
                        else: stuck_count = 0 
                        last_position = pos
                else:
                    time.sleep(1.0) 
            break  
            
        # --- 1. 모드 전환 토글 (Spacebar) ---
        is_space_pressed = is_pressed('space')
        if is_space_pressed and not space_pressed_last:
            if control_mode == "JOINT":
                control_mode = "XYZ"
                target_x, target_y, target_z, target_pitch = forward_kinematics(
                    current_goals[11], current_goals[12], current_goals[13], current_goals[14]
                )
                print(f"\n🔄 [Cartesian/Jacobian 모드 ON] 현재 위치: "f"X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}, "f"Pitch={math.degrees(target_pitch):.1f}°")
            else:
                control_mode = "JOINT"
                print("\n🔄 [관절 모드 ON] 개별 모터 제어 활성화")
        space_pressed_last = is_space_pressed

        # --- 2. 스마트 그리퍼 토글 (C 키) ---
        c_pressed = is_pressed('c')
        if c_pressed and not c_pressed_last:
            if not gripper_is_closed:
                # 닫기 (Catch)
                packetHandler.write4ByteTxRx(portHandler, 15, ADDR_GOAL_POSITION, GRIPPER_CLOSED)
                gripper_is_closed = True
                print(">> ✊ [그리퍼] 물체를 부드럽게 잡습니다.")
            else:
                # 열기 (Clear)
                packetHandler.write4ByteTxRx(portHandler, 15, ADDR_GOAL_POSITION, GRIPPER_OPEN)
                gripper_is_closed = False
                print(">> 🖐️ [그리퍼] 물체를 놓습니다.")
        c_pressed_last = c_pressed

        updated_motors = []
        
        # --- 3. 로봇 팔 이동 연산 (XYZ 또는 JOINT) ---
        if control_mode == "XYZ":
            # --------------------------------------------------
            # Cartesian Differential IK
            #
            # 키보드 입력 -> End Effector의 작은 이동량 Δx
            # Jacobian pseudo-inverse -> 관절 변화량 Δq
            #
            #     Δq = J(q)^+ Δx
            #
            # W/S : ±X
            # A/D : ±Y
            # Q/E : ±Z
            #
            # 현재 gripper pitch는 자동으로 유지한다.
            # --------------------------------------------------
            dx = 0.0
            dy = 0.0
            dz = 0.0

            if is_pressed('w'):
                dx += XYZ_STEP
            elif is_pressed('s'):
                dx -= XYZ_STEP

            if is_pressed('a'):
                dy += XYZ_STEP
            elif is_pressed('d'):
                dy -= XYZ_STEP

            if is_pressed('q'):
                dz += XYZ_STEP
            elif is_pressed('e'):
                dz -= XYZ_STEP

            xyz_updated = (dx != 0.0 or dy != 0.0 or dz != 0.0)

            if xyz_updated:
                jacobian_result = jacobian_cartesian_step(
                    current_goals,
                    dx,
                    dy,
                    dz
                )

                if jacobian_result is not None:
                    # Jacobian이 계산한 4개 관절 목표값을 모두 갱신
                    for dxl_id in [11, 12, 13, 14]:
                        current_goals[dxl_id] = jacobian_result[dxl_id]
                        updated_motors.append(dxl_id)

                    # 상태 확인용 FK
                    target_x, target_y, target_z, target_pitch = forward_kinematics(
                        current_goals[11],
                        current_goals[12],
                        current_goals[13],
                        current_goals[14]
                    )

        elif control_mode == "JOINT":
            if is_pressed('q'): current_goals[11] += JOINT_STEP; updated_motors.append(11)
            elif is_pressed('a'): current_goals[11] -= JOINT_STEP; updated_motors.append(11)
            
            if is_pressed('w'): current_goals[12] += JOINT_STEP; updated_motors.append(12)
            elif is_pressed('s'): current_goals[12] -= JOINT_STEP; updated_motors.append(12)
            
            if is_pressed('e'): current_goals[13] += JOINT_STEP; updated_motors.append(13)
            elif is_pressed('d'): current_goals[13] -= JOINT_STEP; updated_motors.append(13)
            
            if is_pressed('r'): current_goals[14] += JOINT_STEP; updated_motors.append(14)
            elif is_pressed('f'): current_goals[14] -= JOINT_STEP; updated_motors.append(14)

        # --- 4. 데이터 동기화 전송 (11~14번 모터만) ---
        if updated_motors:
            updated_motors = list(set(updated_motors))
            for dxl_id in updated_motors:
                add_sync_write_param(dxl_id, current_goals[dxl_id])
            groupSyncWrite.txPacket()
            groupSyncWrite.clearParam()

        time.sleep(0.03)

finally:
    for dxl_id in DXL_ID_LIST:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
    print("\n시스템이 안전하게 종료되었습니다.")