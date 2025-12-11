# E0509 Manipulation Task

E0509 로봇을 위한 강화학습 기반 조작 태스크 환경입니다. 6자유도 로봇 팔과 4개의 손가락을 가진 mimic 그리퍼로 구성된 E0509 로봇이 목표 자세(pre-grasp pose)에 도달하도록 학습합니다.

## 개요

이 태스크는 Franka Panda의 Lift 태스크를 기반으로 E0509 로봇에 맞게 포팅 및 수정되었습니다. 물체를 집는 대신, 로봇이 특정 목표 자세에 정확하게 도달하는 것을 학습합니다.

### 주요 특징

- **로봇**: E0509 6-DOF 로봇 팔 + 4-finger mimic 그리퍼
- **태스크**: 목표 자세(pose) 도달 (위치 + 방향)
- **학습 알고리즘**: RSL-RL PPO (기본), Stable-Baselines3, RL-Games, SKRL 지원
- **환경 개수**: 최대 4096개의 병렬 환경
- **에피소드 길이**: 5초 (500 스텝 @ 100Hz)

## 파일 구조

```
e0509/
├── README.md                    # 이 문서
├── __init__.py                  # 패키지 초기화
├── e0509.py                     # E0509 로봇 구성 정의
├── lift_env_cfg.py              # 환경 기본 설정 (Scene, Rewards, Commands 등)
├── joint_pos_env_cfg.py         # E0509 특화 설정 (실제 실행 환경)
├── mdp/                         # MDP 컴포넌트
│   ├── __init__.py
│   ├── commands.py              # 커스텀 명령 생성기 (DiscretePoseCommand, HybridPoseCommand)
│   ├── commands_cfg.py          # 명령 생성기 설정
│   ├── observations.py          # 관찰 함수
│   ├── rewards.py               # 보상 함수 (position_exp, multi_axis_alignment 등)
│   └── terminations.py          # 종료 조건
├── agents/                      # RL 알고리즘 설정
│   ├── rsl_rl_ppo_cfg.py       # RSL-RL PPO 설정 (기본)
│   ├── sb3_ppo_cfg.yaml        # Stable-Baselines3 PPO 설정
│   ├── rl_games_ppo_cfg.yaml   # RL-Games PPO 설정
│   └── skrl_ppo_cfg.yaml       # SKRL PPO 설정
└── model/                       # 로봇 3D 모델 (USD 파일)
    ├── e0509_model.usda         # E0509 전체 모델
    ├── arm.usda                 # 6-DOF 로봇 팔
    └── gripper.usda             # 4-finger mimic 그리퍼
```

## 환경 구성

### 로봇 구성 (e0509.py)

```python
E0509_CFG = ArticulationCfg(
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint_1": 0.0,
            "joint_2": 0.0,
            "joint_3": 1.5708,  # 90도
            "joint_4": 0.0,
            "joint_5": 1.5708,  # 90도
            "joint_6": 0.0,
            "rh_.*": 0.0,       # 그리퍼
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-6]"],
            stiffness=800.0,
            damping=40.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["rh_.*"],
            stiffness=2000.0,
            damping=100.0,
        ),
    },
)
```

### 시뮬레이션 장면 (lift_env_cfg.py)

- **로봇**: E0509 (base_link 기준)
- **테이블**: Seattle Lab Table (높이: 0m)
- **바닥**: Ground Plane (높이: -1.05m)
- **마커**: Frame markers (xyz 축 시각화, scale=0.1)

### 명령 생성 (Commands)

**HybridPoseCommand**를 사용하여 목표 자세를 생성합니다:

- **위치**: 균등 분포 샘플링
  - x: 0.4 ~ 0.6m
  - y: -0.25 ~ 0.25m
  - z: 0.1 ~ 0.2m
- **방향**: 사전 정의된 discrete 방향 중 선택
  - roll=90°, pitch=0°, yaw=90° (그리퍼 손가락이 측면을 향함)

```python
object_pose = mdp.HybridPoseCommandCfg(
    body_name="end",  # 그리퍼 end effector
    resampling_time_range=(5.0, 5.0),  # 에피소드마다 새로운 목표
    ranges=mdp.HybridPoseCommandCfg.Ranges(
        pos_x=(0.4, 0.6),
        pos_y=(-0.25, 0.25),
        pos_z=(0.1, 0.2),
    ),
    predefined_orientations=[...],
)
```

### 관찰 (Observations)

PolicyCfg에서 정의된 관찰 벡터:

1. `joint_pos`: 관절 위치 (6 arm + 4 gripper = 10 values)
2. `joint_vel`: 관절 속도 (10 values)
3. `target_pose`: 목표 자세 (position 3 + orientation 4 = 7 values)
4. `actions`: 이전 액션 (10 values)

**총 관찰 차원**: 37

### 액션 (Actions)

- **arm_action**: Joint position control (6 DOF)
  - `joint_[1-6]`: -1 ~ 1 범위, scale=0.5
- **gripper_action**: Binary position control
  - `rh_r1_joint`: 메인 그리퍼 조인트 (나머지는 mimic)
  - Open: 0.04, Close: 0.0

**총 액션 차원**: 7

### 보상 함수 (Rewards)

#### 1. Position Reward (weight=10.0)
```python
reaching_goal_position = position_command_error_exp(alpha=10.0)
```
- 지수 감쇠 커널 사용: R = exp(-10.0 * distance²)
- 목표에 가까울수록 높은 보상
- Jittering 방지를 위한 부드러운 그래디언트

#### 2. Multi-Axis Alignment Reward (weight=15.0)
```python
multi_axis_alignment = multi_axis_alignment_reward()
```
- X, Y, Z 축을 각각 독립적으로 정렬
- 각 축의 내적(dot product)을 계산: (x_align + y_align + z_align) / 3
- 단일 quaternion error보다 명확한 학습 신호 제공

#### 3. Penalty Rewards
- `action_rate`: 액션 변화율 페널티 (weight=-1e-4)
- `joint_vel`: 관절 속도 페널티 (weight=-1e-4)

## 좌표계 구성

### End Effector Frame

그리퍼의 `end` 링크를 기준으로 설정:

```python
self.scene.ee_frame = FrameTransformerCfg(
    prim_path="{ENV_REGEX_NS}/Robot/e0509/base_link",
    debug_vis=True,  # 빨강=X, 초록=Y, 파랑=Z
    target_frames=[
        FrameTransformerCfg.FrameCfg(
            prim_path="{ENV_REGEX_NS}/Robot/e0509/gripper/gripper/end",
            name="end_effector",
            offset=OffsetCfg(pos=(0.0, 0.0, 0.0)),
        ),
    ],
)
```

- **기준 프레임**: `base_link` (로봇 베이스)
- **목표 프레임**: `gripper/gripper/end` (그리퍼 끝)
- **좌표계**: Z축이 손가락 방향을 가리킴 (Isaac Sim 기준)

### Command Body Name

명령 생성과 end effector frame이 동일한 좌표계를 사용하도록 설정:

```python
self.commands.object_pose.body_name = "end"
```

이를 통해 목표 마커와 로봇 end effector의 위치가 정확히 일치합니다.

## 학습 실행

### 환경 이름

- **Task ID**: `Isaac-Lift-Cube-e0509`

### RSL-RL PPO로 학습 (기본)

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Pre-PickPose-e0509 \
    --num_envs 1024
```

### 학습 재개

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Lift-Cube-e0509 \
    --num_envs 1024
    --resume
```

### 학습된 정책 실행

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Pre-PickPose-e0509 \
    --num_envs 6 \
    --checkpoint /home/user/IsaacLab/logs/rsl_rl/e0509_lift_delta_joint
```

### 다른 RL 프레임워크 사용

#### Stable-Baselines3
```bash
./isaaclab.sh -p scripts/reinforcement_learning/sb3/train.py \
    --task Isaac-Lift-Cube-e0509 \
    --num_envs 512
```

#### RL-Games
```bash
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py \
    --task Isaac-Lift-Cube-e0509
```

#### SKRL
```bash
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py \
    --task Isaac-Lift-Cube-e0509 \
    --ml_framework torch  # or jax
```

## 하이퍼파라미터 튜닝

### RSL-RL PPO 설정 (agents/rsl_rl_ppo_cfg.py)

```python
@configclass
class e0509LiftPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 1500
    
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
    )
```

### 보상 가중치 조정

`lift_env_cfg.py`의 `RewardsCfg`에서 가중치 수정:

```python
@configclass
class RewardsCfg:
    reaching_goal_position = RewTerm(..., weight=10.0)
    multi_axis_alignment = RewTerm(..., weight=15.0)
    action_rate = RewTerm(..., weight=-1e-4)
    joint_vel = RewTerm(..., weight=-1e-4)
```

## 커스텀 명령 생성기

### DiscretePoseCommand

사전 정의된 자세 목록에서 무작위로 선택:

```python
DiscretePoseCommandCfg(
    predefined_poses=[
        [x, y, z, qw, qx, qy, qz],  # 자세 1
        [x, y, z, qw, qx, qy, qz],  # 자세 2
        ...
    ],
)
```

### HybridPoseCommand

위치는 범위에서 샘플링, 방향은 discrete 선택:

```python
HybridPoseCommandCfg(
    ranges=Ranges(
        pos_x=(0.4, 0.6),
        pos_y=(-0.25, 0.25),
        pos_z=(0.1, 0.2),
    ),
    predefined_orientations=[
        quat_from_euler_xyz(...).tolist(),
    ],
)
```

## 디버깅 및 시각화

### 마커 시각화

- **목표 마커** (Goal Pose Marker):
  - Frame axes (RGB = XYZ)
  - Scale: 0.1
  - Command에서 자동 생성 (`debug_vis=True`)

- **로봇 End Effector 마커**:
  - Frame axes (RGB = XYZ)
  - Scale: 0.1
  - ee_frame에서 자동 생성 (`debug_vis=True`)

### 로그 및 텐서보드

학습 로그는 `logs/rsl_rl/` 디렉토리에 저장됩니다:

```bash
# 텐서보드 실행
tensorboard --logdir logs/rsl_rl/franka_lift/
```

주요 모니터링 지표:
- `Episode/rewards/reaching_goal_position`: 위치 도달 보상
- `Episode/rewards/multi_axis_alignment`: 방향 정렬 보상
- `Episode/Curriculum/action_rate`: 액션 페널티 (커리큘럼)
- `Episode/Curriculum/joint_vel`: 속도 페널티 (커리큘럼)

## 문제 해결

### 좌표계 불일치

**증상**: 목표 마커와 로봇 end effector 위치가 일치하지 않음

**해결책**: `body_name`과 `ee_frame.target_frames.prim_path`가 동일한 링크를 참조하는지 확인

```python
# 명령 생성
self.commands.object_pose.body_name = "end"

# End effector frame
target_frames=[
    FrameTransformerCfg.FrameCfg(
        prim_path="{ENV_REGEX_NS}/Robot/e0509/gripper/gripper/end",
        ...
    )
]
```

### 방향 정렬 학습이 느림

**증상**: 위치는 수렴하지만 방향 정렬이 잘 안됨

**해결책**:
1. `multi_axis_alignment` 보상 가중치 증가 (15.0 → 20.0)
2. Alpha 값 조정: `position_command_error_exp(alpha=5.0)` (더 부드러운 그래디언트)
3. 명령 방향이 로봇 자연 자세와 일치하는지 확인

### Jittering (떨림) 현상

**증상**: 목표 근처에서 로봇이 계속 떨림

**해결책**:
- Exponential reward 사용 (`position_command_error_exp`)
- Alpha 값 감소 (10.0 → 5.0)
- Action rate penalty 증가 (`action_rate` weight: -1e-4 → -1e-3)

## 참고 자료

- [IsaacLab Documentation](https://isaac-sim.github.io/IsaacLab/)
- [RSL-RL Repository](https://github.com/leggedrobotics/rsl_rl)
- [Franka Lift Task (Original)](https://github.com/isaac-sim/IsaacLab/tree/main/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift)

## 라이센스

BSD-3-Clause License

## 기여

이 태스크는 Isaac Lab Project의 Franka Lift 태스크를 기반으로 E0509 로봇에 맞게 수정되었습니다.
