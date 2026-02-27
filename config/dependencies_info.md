# ROS 2 Workspace Dependencies & Environment Setup

이 문서는 현재 `ros2_ws` 작업 공간을 빌드하기 위해 필요한 모든 의존성(Dependencies)과 `foundation_stereo_py312` Conda 환경에 대한 정보를 포함하고 있습니다.

## 1. `foundation_stereo_py312` Conda 환경 정보

현재 시스템에 설정된 `foundation_stereo_py312` 환경의 모든 패키지 및 버전 정보는 `foundation_stereo_py312_env.yml` 파일로 추출되었습니다.

**환경 복원 방법:**
새로운 PC나 환경에서 아래 명령어를 실행하면 동일한 Conda 환경을 생성할 수 있습니다.
```bash
conda env create -f config/foundation_stereo_py312_env.yml
```

## 2. ROS 2 빌드 의존성 (rosdep)

작업 공간 내의 `src` 및 `deps/isaac_ros_nvblox` 패키지들이 요구하는 시스템 및 ROS 2 의존성 목록입니다.

**주요 의존성 목록:**
- `libeigen3-dev`
- `pybind11-dev`
- `python3-numpy`
- `cupy-cuda12x`
- `python3-transforms3d`
- `boost`
- `python3-scipy`
- `simple-parsing`
- `gazebo_ros_pkgs`
- `python3-ruamel.yaml`
- `python3-shapely`
- `libopencv-dev`
- `python3-opencv`

**의존성 자동 설치 방법:**
새로운 환경에서 작업 공간을 빌드하기 전에 아래 명령어를 실행하여 누락된 의존성을 한 번에 설치할 수 있습니다.
```bash
# rosdep 초기화 및 업데이트 (처음 한 번만 실행)
sudo rosdep init
rosdep update

# 작업 공간의 모든 의존성 자동 설치
cd ~/WALJU
rosdep install --from-paths src deps/isaac_ros_nvblox --ignore-src -r -y
```

## 3. 빌드 명령어

의존성 설치 및 Conda 환경 설정이 완료되면 아래 명령어로 빌드를 진행합니다.

```bash
# Conda 환경 활성화
conda activate foundation_stereo_py312

# ROS 2 작업 공간 빌드
cd ~/WALJU
colcon build --symlink-install
```
