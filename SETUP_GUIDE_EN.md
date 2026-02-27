# 🚀 Environment Setup & Build Guide

This document provides a complete step-by-step guide to setting up and running this project from scratch on a **fresh Ubuntu PC**.

---

## 📋 Table of Contents

1. [System Requirements](#1-system-requirements)
2. [Clone Project & Install Basic Tools](#2-clone-project--install-basic-tools)
3. [NVIDIA Driver & CUDA Installation](#3-nvidia-driver--cuda-installation)
4. [ROS 2 Jazzy Installation](#4-ros-2-jazzy-installation)
5. [Nav2 & Additional ROS 2 Packages](#5-nav2--additional-ros-2-packages)
6. [Conda Installation & Environment Setup](#6-conda-installation--environment-setup)
7. [External Dependencies Installation](#7-external-dependencies-installation)
8. [ROS 2 Workspace Build](#8-ros-2-workspace-build)
9. [Isaac Sim Setup (Simulation Mode)](#9-isaac-sim-setup-simulation-mode)
10. [Run & Test](#10-run--test)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. System Requirements

### Hardware

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **GPU** | NVIDIA GPU (CUDA-capable, VRAM 8GB+) | RTX 4070 or higher (VRAM 12GB+) |
| **CPU** | 6+ cores | 8+ cores (for multi-threaded ROS nodes) |
| **RAM** | 16GB | 32GB+ |
| **Storage** | 100GB+ free space | SSD recommended |

### Software

| Component | Version |
|-----------|---------|
| **OS** | Ubuntu 24.04 LTS (Noble Numbat) |
| **ROS 2** | Jazzy Jalisco |
| **CUDA Toolkit** | 12.x |
| **NVIDIA Driver** | 550+ |
| **Python** | 3.12 (Conda environment) |
| **Conda** | Miniconda or Anaconda |

---

## 2. Clone Project & Install Basic Tools

### 2.1 Clone the Project

```bash
sudo apt install -y git
cd ~
git clone https://github.com/eugene0429/WALJU.git
cd ~/WALJU
```

> 📌 The ROS 2 source packages (`orb_slam3_ros2`, `foundation_stereo_ros2`, `nvblox_integration`) under `src/` are already included in the repository.
> The `deps/` folder is empty and will be populated in [Step 7](#7-external-dependencies-installation) with external libraries.

### 2.2 Install Basic Tools

```bash
# System update
sudo apt update && sudo apt upgrade -y

# Install essential build tools
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

## 3. NVIDIA Driver & CUDA Installation

### 3.1 NVIDIA Driver

```bash
# Check recommended driver
ubuntu-drivers devices

# Install driver (adjust version number as needed)
sudo apt install -y nvidia-driver-550-open

# Reboot
sudo reboot
```

Verify after reboot:
```bash
nvidia-smi
# You should see GPU name, driver version, and CUDA version
```

### 3.2 CUDA Toolkit 12.x

```bash
# Add NVIDIA CUDA repository (for Ubuntu 24.04)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# Install CUDA Toolkit
sudo apt install -y cuda-toolkit-12-0

# Add environment variables to ~/.bashrc
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

Verify:
```bash
nvcc --version
# Should output cuda_12.0 or similar
```

---

## 4. ROS 2 Jazzy Installation

```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 GPG key
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# Install ROS 2 Jazzy Desktop (includes RViz2)
sudo apt install -y ros-jazzy-desktop

# Add environment setup to ~/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc

# Install colcon build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

---

## 5. Nav2 & Additional ROS 2 Packages

```bash
# Nav2 Navigation Stack
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Additional required ROS 2 packages
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

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update
```

---

## 6. Conda Installation & Environment Setup

The FoundationStereo depth estimation node runs in a separate Conda environment (`foundation_stereo_py312`).

### 6.1 Install Miniconda

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
# Select 'yes' during installation to run conda init

# Open a new terminal or:
source ~/.bashrc
```

### 6.2 Restore Conda Environment

Restore the environment using the `config/foundation_stereo_py312_env.yml` file included in the project.

```bash
cd ~/WALJU
conda env create -f config/foundation_stereo_py312_env.yml
```

> ⚠️ This process may take a while (10–30 minutes).

Verify the environment:
```bash
conda activate foundation_stereo_py312
python --version
# Should output Python 3.12.x
conda deactivate
```

---

## 7. External Dependencies Installation

Clone and build external libraries into the `deps/` directory.

### 7.1 Pangolin (GUI dependency for ORB-SLAM3)

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

The ORB-SLAM3 node references the pre-built library (`libORB_SLAM3.so`).

```bash
cd ~/WALJU/deps
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3

# Apply custom patch (required: build optimizations and additional features)
git apply ../../patches/ORB-SLAM3.patch

# Extract ORB Vocabulary
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

# Build
chmod +x build.sh
./build.sh
```

Verify the build:
```bash
ls ~/WALJU/deps/ORB_SLAM3/lib/libORB_SLAM3.so
ls ~/WALJU/deps/ORB_SLAM3/Vocabulary/ORBvoc.txt
# Both files should exist
```

### 7.3 FoundationStereo

```bash
cd ~/WALJU/deps
git clone https://github.com/NVlabs/FoundationStereo.git
cd FoundationStereo

# Download pretrained models
# Follow the official GitHub repository instructions to download checkpoints
# into the pretrained_models/ directory.
# https://github.com/NVlabs/FoundationStereo#pretrained-models
```

### 7.4 isaac_ros_nvblox

```bash
cd ~/WALJU/deps
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

# Apply custom modifications (thread safety patch)
cd isaac_ros_nvblox
git apply ../../patches/isaac_ros_nvblox_custom.patch
cd ..
```

---

## 8. ROS 2 Workspace Build

### 8.1 Install Missing Dependencies via rosdep

```bash
cd ~/WALJU
rosdep install --from-paths src deps/isaac_ros_nvblox --ignore-src -r -y
```

### 8.2 Build

```bash
cd ~/WALJU

# Verify ROS 2 environment is sourced
source /opt/ros/jazzy/setup.bash

# Full build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the built workspace
source install/setup.bash
```

> ⚠️ If you encounter build errors, refer to the [Troubleshooting](#11-troubleshooting) section.

### 8.3 Automate Environment Sourcing (add to ~/.bashrc)

```bash
echo 'source ~/WALJU/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## 9. Isaac Sim Setup (Simulation Mode)

To run in simulation mode (`--sim`), NVIDIA Isaac Sim must be installed.

1. Download and install from the [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) official page
2. Open a Lunar environment scene in Isaac Sim and start the simulation
3. Verify that the ROS 2 Bridge is active and stereo image topics are being published:
   ```bash
   ros2 topic list | grep stereo
   # You should see /stereo/left/rgb, /stereo/right/rgb, etc.
   ```

> 📌 You can test the pipeline without Isaac Sim using Dataset mode (`--dataset`) with the LuSNAR dataset.

---

## 10. Run & Test

### Full Pipeline (Simulation Mode)

```bash
cd ~/WALJU
./run_pipeline.sh --all --sim
```

### Full Pipeline (Dataset Mode)

```bash
./run_pipeline.sh --all --dataset
```

### Run Individual Components

```bash
# SLAM + Depth only
./run_pipeline.sh --slam --depth

# nvblox + RViz visualization only
./run_pipeline.sh --nvblox --rviz

# Full pipeline + Nav2 navigation
./run_pipeline.sh --all --nav2
```

### Stop Pipeline

```bash
./kill_pipeline.sh
```

### View Available Options

```bash
./run_pipeline.sh --help
```

---

## 11. Troubleshooting

### ORB-SLAM3 Build Error: `libORB_SLAM3.so not found`

- Verify that ORB-SLAM3 is correctly built at `~/WALJU/deps/ORB_SLAM3/`
- If the path differs, update `ORB_SLAM3_ROOT` in `src/orb_slam3_ros2/CMakeLists.txt`

### Pangolin-related Errors

```bash
# Check if Pangolin is installed on the system
pkg-config --modversion pangolin
# If not installed, repeat Step 7.1
```

### Conda Environment Creation Failure (Package Conflicts)

```bash
# Create environment with core packages only (instead of strict versions)
conda create -n foundation_stereo_py312 python=3.12
conda activate foundation_stereo_py312
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install opencv-python numpy scipy
# Then install FoundationStereo's requirements
cd ~/WALJU/deps/FoundationStereo
pip install -r requirements.txt
```

### colcon build Errors for Specific Packages

```bash
# Build excluding the problematic package
colcon build --symlink-install --packages-skip <package-name>

# Rebuild a specific package only
colcon build --symlink-install --packages-select <package-name>
```

### Nav2 `controller_server` Crash at Runtime

```bash
# Verify all Nav2 packages are installed
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### `isaac_ros_nvblox` Patch Application Failure

```bash
cd ~/WALJU/deps/isaac_ros_nvblox
# Preview patch before applying
git apply --check ../../patches/isaac_ros_nvblox_custom.patch

# Force apply with 3-way merge if conflicts occur
git apply --3way ../../patches/isaac_ros_nvblox_custom.patch
```

---

## 📂 Key Paths Reference

| Item | Path |
|------|------|
| **Project Root** | `~/WALJU/` |
| **Source Packages** | `~/WALJU/src/` |
| **External Libraries** | `~/WALJU/deps/` |
| **Datasets** | `~/WALJU/data/` |
| **isaac_ros_nvblox** | `~/WALJU/deps/isaac_ros_nvblox/` |
| **ORB-SLAM3** | `~/WALJU/deps/ORB_SLAM3/` |
| **ORB Vocabulary** | `~/WALJU/deps/ORB_SLAM3/Vocabulary/ORBvoc.txt` |
| **FoundationStereo** | `~/WALJU/deps/FoundationStereo/` |
| **LuSNAR Dataset** | `~/WALJU/data/LuSNAR/` |
| **SLAM Settings (Sim)** | `~/WALJU/assets/sim_slam_settings.yaml` |
| **Camera Intrinsics (Sim)** | `~/WALJU/assets/sim_intrinsics.txt` |
| **Patch Files** | `~/WALJU/patches/` |
| **nvblox Custom Patch** | `~/WALJU/patches/isaac_ros_nvblox_custom.patch` |
| **ORB-SLAM3 Build Patch** | `~/WALJU/patches/ORB-SLAM3.patch` |
| **Conda Environment File** | `~/WALJU/config/foundation_stereo_py312_env.yml` |
| **Dependencies Info** | `~/WALJU/config/dependencies_info.md` |
| **Utility Scripts** | `~/WALJU/scripts/` |
| **Waypoints** | `~/WALJU/data/waypoints.json` |
| **Experiment Results** | `~/WALJU/data/scenario_results/` |
| **Pipeline Launch** | `~/WALJU/run_pipeline.sh` |
| **Pipeline Stop** | `~/WALJU/kill_pipeline.sh` |
