# 🚀 환경 설정 및 빌드 가이드

이 문서는 **아무것도 없는 새로운 Ubuntu PC**에서 이 프로젝트를 처음부터 구동하기까지의 전체 과정을 안내합니다.

---

## 📋 목차

1. [시스템 요구사항](#1-시스템-요구사항)
2. [프로젝트 클론 및 기본 도구 설치](#2-프로젝트-클론-및-기본-도구-설치)
3. [NVIDIA 드라이버 및 CUDA 설치](#3-nvidia-드라이버-및-cuda-설치)
4. [ROS 2 Jazzy 설치](#4-ros-2-jazzy-설치)
5. [Nav2 및 ROS 2 추가 패키지 설치](#5-nav2-및-ros-2-추가-패키지-설치)
6. [Conda 설치 및 환경 구성](#6-conda-설치-및-환경-구성)
7. [외부 의존성 설치](#7-외부-의존성-설치)
8. [ROS 2 작업 공간 빌드](#8-ros-2-작업-공간-빌드)
9. [Isaac Sim 설정 (시뮬레이션 모드)](#9-isaac-sim-설정-시뮬레이션-모드)
10. [실행 테스트](#10-실행-테스트)
11. [트러블슈팅](#11-트러블슈팅)

---

## 1. 시스템 요구사항

### 하드웨어

| 항목 | 최소 | 권장 |
|------|------|------|
| **GPU** | NVIDIA GPU (CUDA 지원, VRAM 8GB+) | RTX 4070 이상 (VRAM 12GB+) |
| **CPU** | 6코어 이상 | 8코어+ (멀티스레드 ROS 노드 처리) |
| **RAM** | 16GB | 32GB+ |
| **저장 공간** | 100GB 이상 여유 | SSD 권장 |

### 소프트웨어

| 항목 | 버전 |
|------|------|
| **OS** | Ubuntu 24.04 LTS (Noble Numbat) |
| **ROS 2** | Jazzy Jalisco |
| **CUDA Toolkit** | 12.x |
| **NVIDIA Driver** | 550+ |
| **Python** | 3.12 (Conda 환경) |
| **Conda** | Miniconda 또는 Anaconda |

---

## 2. 프로젝트 클론 및 기본 도구 설치

### 2.1 프로젝트 클론

```bash
sudo apt install -y git
cd ~
git clone https://github.com/eugene0429/WALJU.git
cd ~/WALJU
```

> 📌 `src/` 내 ROS 2 패키지들(`orb_slam3_ros2`, `foundation_stereo_ros2`, `nvblox_integration`)은 이미 리포지토리에 포함되어 있습니다.
> `deps/` 폴더는 비어 있으며, [Step 7](#7-외부-의존성-설치)에서 외부 라이브러리를 클론 및 빌드합니다.

### 2.2 기본 도구 설치

```bash
# 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# 필수 빌드 도구 설치
sudo apt install -y \
    build-essential cmake git curl wget \
    pkg-config unzip software-properties-common \
    python3-pip python3-dev python3-venv \
    libeigen3-dev libboost-all-dev libssl-dev \
    libopencv-dev libglew-dev libgl1-mesa-dev \
    libwayland-dev libxkbcommon-dev \
    libpython3-dev pybind11-dev
```

---

## 3. NVIDIA 드라이버 및 CUDA 설치

### 3.1 NVIDIA 드라이버

```bash
# 추천 드라이버 확인
ubuntu-drivers devices

# 드라이버 설치 (번호는 환경에 맞게 조정)
sudo apt install -y nvidia-driver-550-open

# 재부팅
sudo reboot
```

재부팅 후 확인:
```bash
nvidia-smi
# GPU 이름, 드라이버 버전, CUDA 버전이 표시되면 성공
```

### 3.2 CUDA Toolkit 12.x

```bash
# NVIDIA CUDA 저장소 추가 (Ubuntu 24.04용)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# CUDA Toolkit 설치
sudo apt install -y cuda-toolkit-12-0

# 환경 변수 설정 (~/.bashrc에 추가)
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

확인:
```bash
nvcc --version
# cuda_12.0 또는 유사한 버전이 출력되면 성공
```

---

## 4. ROS 2 Jazzy 설치

```bash
# 로케일 설정
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS 2 GPG key 추가
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS 2 저장소 추가
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# ROS 2 Jazzy Desktop 설치 (RViz2 포함)
sudo apt install -y ros-jazzy-desktop

# 환경 설정 (~/.bashrc에 추가)
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc

# colcon 빌드 도구 설치
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

---

## 5. Nav2 및 ROS 2 추가 패키지 설치

```bash
# Nav2 네비게이션 스택
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# 추가 필수 ROS 2 패키지
sudo apt install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-grid-map-core \
    ros-jazzy-grid-map-ros \
    ros-jazzy-grid-map-msgs \
    ros-jazzy-grid-map-cv \
    ros-jazzy-grid-map-filters \
    ros-jazzy-grid-map-rviz-plugin

# rosdep 초기화 (최초 1회)
sudo rosdep init
rosdep update
```

---

## 6. Conda 설치 및 환경 구성

FoundationStereo 깊이 추정 노드는 별도의 Conda 환경(`foundation_stereo_py312`)에서 실행됩니다.

### 6.1 Miniconda 설치

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
# 설치 중 'yes'를 선택하여 conda init을 수행

# 새 터미널을 열거나:
source ~/.bashrc
```

### 6.2 Conda 환경 복원

프로젝트에 포함된 `config/foundation_stereo_py312_env.yml` 파일로 환경을 그대로 복원합니다.

```bash
cd ~/WALJU
conda env create -f config/foundation_stereo_py312_env.yml
```

> ⚠️ 이 과정은 시간이 오래 걸릴 수 있습니다 (10~30분).

환경 확인:
```bash
conda activate foundation_stereo_py312
python --version
# Python 3.12.x가 출력되면 성공
conda deactivate
```

---

## 7. 외부 의존성 설치

`deps/` 디렉토리에 외부 라이브러리를 클론하고 빌드합니다.

### 7.1 Pangolin (ORB-SLAM3의 GUI 의존성)

```bash
cd ~
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### 7.2 ORB-SLAM3

ORB-SLAM3 노드는 사전에 빌드된 라이브러리(`libORB_SLAM3.so`)를 참조합니다.

```bash
cd ~/WALJU/deps
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3

# 커스텀 패치 적용 (필수: 빌드 최적화 및 추가 기능)
git apply ../../patches/ORB-SLAM3.patch

# ORB Vocabulary 압축 해제
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

# 빌드
chmod +x build.sh
./build.sh
```

빌드 후 확인:
```bash
ls ~/WALJU/deps/ORB_SLAM3/lib/libORB_SLAM3.so
ls ~/WALJU/deps/ORB_SLAM3/Vocabulary/ORBvoc.txt
# 두 파일이 모두 존재하면 성공
```

### 7.3 FoundationStereo

```bash
cd ~/WALJU/deps
git clone https://github.com/NVlabs/FoundationStereo.git
cd FoundationStereo

# Pretrained 모델 다운로드
# 공식 GitHub 저장소의 안내를 따라 pretrained_models/ 디렉토리에 체크포인트를 다운로드합니다.
# https://github.com/NVlabs/FoundationStereo#pretrained-models
```

### 7.4 isaac_ros_nvblox

```bash
cd ~/WALJU/deps
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

# 커스텀 수정 사항 적용 (스레드 안전성 패치)
cd isaac_ros_nvblox
git apply ../../patches/isaac_ros_nvblox_custom.patch
cd ..
```

---

## 8. ROS 2 작업 공간 빌드

### 8.1 rosdep으로 빠진 의존성 설치

```bash
cd ~/WALJU
rosdep install --from-paths src deps/isaac_ros_nvblox --ignore-src -r -y
```

### 8.2 빌드

```bash
cd ~/WALJU

# ROS 2 환경 소싱 확인
source /opt/ros/jazzy/setup.bash

# 전체 빌드
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 빌드된 작업 공간 소싱
source install/setup.bash
```

> ⚠️ 빌드 중 에러가 발생하면 [트러블슈팅](#11-트러블슈팅) 섹션을 참고하세요.

### 8.3 환경 소싱 자동화 (~/.bashrc에 추가)

```bash
echo 'source ~/WALJU/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## 9. Isaac Sim 설정 (시뮬레이션 모드)

시뮬레이션 모드(`--sim`)로 실행하려면 NVIDIA Isaac Sim이 설치되어 있어야 합니다.

1. [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) 공식 페이지에서 다운로드 및 설치
2. Isaac Sim에서 Lunar 환경 Scene을 열고 시뮬레이션 실행
3. ROS 2 Bridge가 활성화되어 스테레오 이미지 토픽이 퍼블리시되고 있는지 확인:
   ```bash
   ros2 topic list | grep stereo
   # /stereo/left/rgb, /stereo/right/rgb 등이 표시되면 정상
   ```

> 📌 Isaac Sim 없이도 Dataset 모드(`--dataset`)로 LuSNAR 데이터셋을 사용해 파이프라인을 테스트할 수 있습니다.

---

## 10. 실행 테스트

### 전체 파이프라인 (시뮬레이션 모드)

```bash
cd ~/WALJU
./run_pipeline.sh --all --sim
```

### 전체 파이프라인 (데이터셋 모드)

```bash
./run_pipeline.sh --all --dataset
```

### 개별 컴포넌트 실행

```bash
# SLAM + Depth만 실행
./run_pipeline.sh --slam --depth

# nvblox + RViz 시각화만 실행
./run_pipeline.sh --nvblox --rviz

# 전체 + Nav2 네비게이션 포함
./run_pipeline.sh --all --nav2
```

### 파이프라인 종료

```bash
./kill_pipeline.sh
```

### 사용 가능한 옵션 확인

```bash
./run_pipeline.sh --help
```

---

## 11. 트러블슈팅

### ORB-SLAM3 빌드 에러: `libORB_SLAM3.so not found`

- ORB-SLAM3가 `~/WALJU/deps/ORB_SLAM3/` 경로에 올바르게 빌드되었는지 확인
- 경로가 다르다면 `src/orb_slam3_ros2/CMakeLists.txt`의 `ORB_SLAM3_ROOT` 변수를 수정

### Pangolin 관련 에러

```bash
# Pangolin이 시스템에 설치되어 있는지 확인
pkg-config --modversion pangolin
# 설치되어 있지 않다면 7.1 단계를 다시 수행
```

### Conda 환경 생성 실패 (패키지 충돌)

```bash
# 엄격한 버전 대신 핵심 패키지만으로 환경 생성
conda create -n foundation_stereo_py312 python=3.12
conda activate foundation_stereo_py312
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install opencv-python numpy scipy
# 이후 FoundationStereo의 requirements.txt 설치
cd ~/WALJU/deps/FoundationStereo
pip install -r requirements.txt
```

### colcon build 시 특정 패키지 에러

```bash
# 문제 패키지만 제외하고 빌드
colcon build --symlink-install --packages-skip <패키지명>

# 특정 패키지만 재빌드
colcon build --symlink-install --packages-select <패키지명>
```

### Nav2 실행 시 `controller_server` 크래시

```bash
# Nav2 관련 패키지가 모두 설치되어 있는지 확인
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### `isaac_ros_nvblox` 패치 적용 실패

```bash
cd ~/WALJU/deps/isaac_ros_nvblox
# 패치 내용 미리 확인
git apply --check ../../patches/isaac_ros_nvblox_custom.patch

# 충돌 시 강제 적용 (3-way merge)
git apply --3way ../../patches/isaac_ros_nvblox_custom.patch
```

---

## 📂 주요 경로 요약

| 항목 | 경로 |
|------|------|
| **프로젝트 루트** | `~/WALJU/` |
| **소스 패키지** | `~/WALJU/src/` |
| **외부 라이브러리** | `~/WALJU/deps/` |
| **데이터셋** | `~/WALJU/data/` |
| **isaac_ros_nvblox** | `~/WALJU/deps/isaac_ros_nvblox/` |
| **ORB-SLAM3** | `~/WALJU/deps/ORB_SLAM3/` |
| **ORB Vocabulary** | `~/WALJU/deps/ORB_SLAM3/Vocabulary/ORBvoc.txt` |
| **FoundationStereo** | `~/WALJU/deps/FoundationStereo/` |
| **LuSNAR 데이터셋** | `~/WALJU/data/LuSNAR/` |
| **SLAM 설정 (Sim)** | `~/WALJU/assets/sim_slam_settings.yaml` |
| **카메라 Intrinsics (Sim)** | `~/WALJU/assets/sim_intrinsics.txt` |
| **패치 파일** | `~/WALJU/patches/` |
| **nvblox 커스텀 패치** | `~/WALJU/patches/isaac_ros_nvblox_custom.patch` |
| **ORB-SLAM3 빌드 패치** | `~/WALJU/patches/ORB-SLAM3.patch` |
| **Conda 환경 파일** | `~/WALJU/config/foundation_stereo_py312_env.yml` |
| **의존성 정보** | `~/WALJU/config/dependencies_info.md` |
| **유틸리티 스크립트** | `~/WALJU/scripts/` |
| **웨이포인트** | `~/WALJU/data/waypoints.json` |
| **실험 결과** | `~/WALJU/data/scenario_results/` |
| **파이프라인 실행** | `~/WALJU/run_pipeline.sh` |
| **파이프라인 종료** | `~/WALJU/kill_pipeline.sh` |
