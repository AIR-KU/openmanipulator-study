# 시뮬레이션: 궤적 생성 (MuJoCo)

OpenManipulator-X 를 MuJoCo 에서 움직이며 궤적 생성과 스무딩을 공부한 기록.
코드는 팀 환경 저장소 **[AIR-KU/omx-mujoco](https://github.com/AIR-KU/omx-mujoco)** 에 있다
(비공개. 접근 권한이 없으면 조직 관리자에게 요청).

| 날짜 | 주제 | 코드 |
|---|---|---|
| 2026-09-27 | Modern Robotics 9.1~9.2: 직선 경로 + 5차 다항식 time scaling | [`songmiyung/quintic-p2p`](https://github.com/AIR-KU/omx-mujoco/tree/songmiyung/quintic-p2p) 브랜치 |

---

## 2026-09-27 — point-to-point (MR 9.2)

### 공부한 것
- 궤적 = 경로 θ(s) + time scaling s(t). s 는 경로 진행률 (0 → 1)
- 직선 경로: 관절공간 직선 vs 직교공간 직선
  - 관절공간: 관절 제한이 볼록(직육면체)이라 양 끝만 괜찮으면 안전. 대신 공구 끝이 휜다
  - 직교공간: 공구 끝이 곧게 간다. 대신 도달 영역·특이점을 확인해야 한다
- time scaling: 3차 / 5차 다항식, 사다리꼴, S-커브
  - 5차: 양 끝 가속도 0 → 저크 유한. 최대 속도 1.875/T, 최대 가속도 5.774/T²
  - 느리게 하면(T 증가) 속도 ∝ 1/T, 가속도 ∝ 1/T²

### 구현한 것 (omx-mujoco 브랜치)
| 파일 | 내용 |
|---|---|
| `omx/traj/profiles.py` | 5차 time scaling `quintic(t, T)` → s, ṡ, s̈, 저크. 한계로 최소 T 계산 |
| `omx/traj/interp.py` | `joint_line`, `cartesian_line`. 직교공간은 매 샘플 IK, 관절 한계 넘으면 T 늘림 |
| `omx/log.py`, `omx/metrics.py` | 실행 기록(.npz) / 지표 |
| `scripts/compare_p2p.py` | 창 없이 두 경로 비교 |
| `scripts/plot_log.py` | 기록 겹쳐 그리기 |
| `sim/01_point_to_point.py` | 뷰어: 마커로 목표 → J(관절)/C(직교) 계획 → Space 재생 |

### 결과 (HOME → (0.15, 0.15, 0.05) m, pitch −0.8 rad)
| | 관절공간 직선 | 직교공간 직선 |
|---|---|---|
| 이동 시간 T | 0.98 s (관절 가속도 한계) | 2.42 s (공구 끝 속도 한계) |
| 직선 대비 공구 끝 이탈 | **26.1 mm** | **0.8 mm** |
| 추종 오차 최대 | 3.5° | 1.5° |

한계: 관절 속도 2 rad/s, 관절 가속도 5 rad/s², 공구 끝 0.15 m/s · 0.5 m/s².

### 실행 (omx-mujoco 에서)
```bash
git checkout songmiyung/quintic-p2p
source .venv/bin/activate
python scripts/compare_p2p.py
python scripts/plot_log.py
mjpython sim/01_point_to_point.py      # macOS. Home = fn + ←
```

### 다음
- MR 9.3 via point 궤적 (숙제: 9.3 읽기, Exercise 9.18, 9.19)
- 3차 / 사다리꼴 / S-커브 추가 후 비교
- pick & place 한 사이클
