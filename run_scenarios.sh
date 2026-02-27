#!/bin/bash
# =============================================================================
# Scenario Runner - s1~s12 자동 반복 실험 스크립트
# =============================================================================
#
# Usage:
#   ./run_scenarios.sh                     # s1~s12 전체 실행
#   ./run_scenarios.sh -s 3 -e 7           # s3~s7만 실행
#   ./run_scenarios.sh -s 5                # s5~s12 실행
#   ./run_scenarios.sh -e 3                # s1~s3 실행
#   ./run_scenarios.sh -o 2,5,11           # s2, s5, s11만 실행
#   ./run_scenarios.sh --manual-sim        # 시뮬레이터 수동 시작 모드
#   ./run_scenarios.sh --dry-run           # 실제 실행 없이 계획만 출력
#
# 시뮬레이터 시작 방법:
#   기본: SSH로 jaewon@localhost 에서 run_simulator.sh 자동 실행
#   --manual-sim: 각 시나리오마다 사용자에게 시뮬레이터 시작을 요청
#
# =============================================================================

set -euo pipefail

# =============================================================================
# Configuration
# =============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}"
WAYPOINTS_FILE="${ROS2_WS}/data/waypoints_test.json"
METRICS_FILE="${ROS2_WS}/data/navigation_metrics.csv"
RESULTS_BASE_DIR="${ROS2_WS}/data/scenario_results"

# Simulator configuration
SIMULATOR_SCRIPT="/home/jaewon/Lproject_sim/start_simulation.sh"
SIMULATOR_USER="jaewon"
SIMULATOR_HOST="localhost"
SSH_CMD="ssh -o BatchMode=yes -o StrictHostKeyChecking=no"

# jaewon의 그래픽 세션 환경 (Isaac Sim GUI 실행에 필요)
# 자동 감지되며, 감지 실패 시 수동 설정 가능
SIM_DISPLAY=""       # 자동 감지 (예: :10.0)
SIM_XAUTHORITY=""   # 자동 감지 (예: /home/jaewon/.Xauthority)

# Pipeline command
PIPELINE_CMD="./run_pipeline.sh --all --sim --image-type noisy --no-depth-filter --metrics"

# Timing configuration
SIM_READY_TIMEOUT=30        # 시뮬레이터 준비 대기 (초)
PIPELINE_READY_TIMEOUT=120   # 파이프라인 준비 대기 (초)
NAV_COMPLETE_TIMEOUT=2400    # 네비게이션 완료 대기 (초, 40분)
WAIT_AFTER_SIM_START=15      # 시뮬레이터 시작 후 대기 (초)
WAIT_AFTER_PIPELINE=20       # 파이프라인 시작 후 대기 (초)
WAIT_BETWEEN_SCENARIOS=10    # 시나리오 사이 대기 (초)
WAIT_AFTER_KILL=8            # 파이프라인 종료 후 대기 (초)

# Sim readiness check topic (Isaac Sim이 퍼블리시하는 토픽)
SIM_CHECK_TOPIC="/tf_gt"

# Nav2 readiness check topic
NAV2_CHECK_TOPIC="/map"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

# =============================================================================
# Default Settings
# =============================================================================
START_SCENARIO=1
END_SCENARIO=2
ONLY_SCENARIOS=""     # comma-separated list for -o flag
MANUAL_SIM=false
DRY_RUN=false
SKIP_SIM_START=false  # --no-sim: 시뮬레이터 시작 스킵 (이미 실행 중일 때)
AUTO_CONFIRM=false    # --yes: 확인 없이 바로 실행

# =============================================================================
# Helper Functions
# =============================================================================

log_header() {
    echo ""
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC} ${BOLD}$1${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════════╝${NC}"
}

log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[  OK]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()    { echo -e "${MAGENTA}[STEP]${NC} $1"; }

timestamp() {
    date '+%Y-%m-%d %H:%M:%S'
}

# Count waypoints from JSON file
count_waypoints() {
    if [ ! -f "${WAYPOINTS_FILE}" ]; then
        echo "0"
        return
    fi
    python3 -c "
import json
with open('${WAYPOINTS_FILE}') as f:
    data = json.load(f)
print(len(data))
" 2>/dev/null || echo "0"
}

print_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -s, --start NUM        시작 시나리오 번호 (default: 1)"
    echo "  -e, --end NUM          종료 시나리오 번호 (default: 12)"
    echo "  -o, --only NUM,NUM,... 특정 시나리오만 실행 (comma-separated)"
    echo "  --manual-sim           시뮬레이터 수동 시작 모드 (매번 확인)"
    echo "  --no-sim               시뮬레이터 시작 스킵 (이미 실행 중일 때)"
    echo "  --dry-run              실제 실행 없이 계획만 출력"
    echo "  -y, --yes              확인 없이 자동 실행"
    echo "  --sim-timeout SEC      시뮬레이터 준비 대기 시간 (default: ${SIM_READY_TIMEOUT}s)"
    echo "  --nav-timeout SEC      네비게이션 완료 대기 시간 (default: ${NAV_COMPLETE_TIMEOUT}s)"
    echo "  --waypoints FILE       웨이포인트 파일 경로 (default: ${WAYPOINTS_FILE})"
    echo "  -h, --help             도움말 표시"
    echo ""
    echo "Examples:"
    echo "  $0                           # s1~s12 전체 자동 실행"
    echo "  $0 -s 3 -e 7                 # s3~s7 실행"
    echo "  $0 -o 1,5,10 --manual-sim    # s1,s5,s10만 (수동 시뮬 시작)"
    echo "  $0 --no-sim -o 3             # 시뮬 이미 실행 중, s3만 실행"
}

# =============================================================================
# Parse Arguments
# =============================================================================
while [[ $# -gt 0 ]]; do
    case $1 in
        -s|--start)
            START_SCENARIO="$2"; shift 2 ;;
        -e|--end)
            END_SCENARIO="$2"; shift 2 ;;
        -o|--only)
            ONLY_SCENARIOS="$2"; shift 2 ;;
        --manual-sim)
            MANUAL_SIM=true; shift ;;
        --no-sim)
            SKIP_SIM_START=true; shift ;;
        --dry-run)
            DRY_RUN=true; shift ;;
        -y|--yes)
            AUTO_CONFIRM=true; shift ;;
        --sim-timeout)
            SIM_READY_TIMEOUT="$2"; shift 2 ;;
        --nav-timeout)
            NAV_COMPLETE_TIMEOUT="$2"; shift 2 ;;
        --waypoints)
            WAYPOINTS_FILE="$2"; shift 2 ;;
        -h|--help)
            print_help; exit 0 ;;
        *)
            log_error "Unknown option: $1"
            print_help; exit 1 ;;
    esac
done

# =============================================================================
# Build Scenario List
# =============================================================================
SCENARIOS=()
if [ -n "${ONLY_SCENARIOS}" ]; then
    IFS=',' read -ra SCENARIOS <<< "${ONLY_SCENARIOS}"
else
    for i in $(seq "${START_SCENARIO}" "${END_SCENARIO}"); do
        SCENARIOS+=("$i")
    done
fi

TOTAL_SCENARIOS=${#SCENARIOS[@]}
TOTAL_WAYPOINTS=$(count_waypoints)

if [ "${TOTAL_SCENARIOS}" -eq 0 ]; then
    log_error "실행할 시나리오가 없습니다."
    exit 1
fi

if [ "${TOTAL_WAYPOINTS}" -eq 0 ]; then
    log_error "웨이포인트를 읽을 수 없습니다: ${WAYPOINTS_FILE}"
    exit 1
fi

# =============================================================================
# Pre-flight Check & Summary
# =============================================================================
log_header "Scenario Runner - 실험 자동화"

echo ""
echo -e "${BOLD}실행 계획:${NC}"
echo -e "  시나리오:    s${SCENARIOS[0]}$([ ${TOTAL_SCENARIOS} -gt 1 ] && echo " ~ s${SCENARIOS[-1]}") (총 ${TOTAL_SCENARIOS}개)"
echo -e "  시나리오 목록: ${SCENARIOS[*]}"
echo -e "  웨이포인트:  ${TOTAL_WAYPOINTS}개 (${WAYPOINTS_FILE})"
echo -e "  시뮬레이터:  $([ "${MANUAL_SIM}" = true ] && echo "수동 시작" || ([ "${SKIP_SIM_START}" = true ] && echo "시작 스킵" || echo "SSH 자동 시작 (${SIMULATOR_USER}@${SIMULATOR_HOST})"))"
echo -e "  결과 저장:   ${RESULTS_BASE_DIR}/"
echo -e "  파이프라인:  ${PIPELINE_CMD}"
echo ""
echo -e "${BOLD}타임아웃 설정:${NC}"
echo -e "  시뮬레이터 준비:   ${SIM_READY_TIMEOUT}s"
echo -e "  파이프라인 준비:   ${PIPELINE_READY_TIMEOUT}s"
echo -e "  네비게이션 완료:   ${NAV_COMPLETE_TIMEOUT}s"
echo ""

if [ "${DRY_RUN}" = true ]; then
    log_warn "DRY-RUN 모드: 실제 실행 없이 종료합니다."
    exit 0
fi

if [ "${AUTO_CONFIRM}" = false ]; then
    echo -e "${YELLOW}위 설정으로 실행하시겠습니까? [Y/n]${NC} "
    read -r confirm
    if [[ "${confirm}" =~ ^[Nn] ]]; then
        log_info "취소됨."
        exit 0
    fi
fi

# Create results directory
mkdir -p "${RESULTS_BASE_DIR}"

# =============================================================================
# Core Functions
# =============================================================================

# jaewon의 활성 그래픽 세션(DISPLAY/XAUTHORITY) 자동 감지
detect_sim_display() {
    if [ -n "${SIM_DISPLAY}" ]; then
        log_info "DISPLAY 수동 설정 사용: ${SIM_DISPLAY}"
        return 0
    fi

    log_info "jaewon 그래픽 세션 DISPLAY 감지 중..."

    # jaewon의 실행 중인 GUI 프로세스에서 DISPLAY 추출
    local detected
    detected=$(${SSH_CMD} ${SIMULATOR_USER}@${SIMULATOR_HOST} \
        'for pid in $(pgrep -u jaewon -x gnome-session 2>/dev/null || pgrep -u jaewon -x xfce4-session 2>/dev/null || pgrep -u jaewon -f "Xorg|Xwayland|dbus-daemon" 2>/dev/null | head -3); do
            cat /proc/$pid/environ 2>/dev/null | tr "\0" "\n" | grep "^DISPLAY=" | head -1 && break
        done' 2>/dev/null || true)

    if [ -n "${detected}" ]; then
        SIM_DISPLAY="${detected#DISPLAY=}"
        log_success "DISPLAY 감지 완료: ${SIM_DISPLAY}"
    else
        SIM_DISPLAY=":10.0"  # fallback
        log_warn "DISPLAY 자동 감지 실패. 기본값 사용: ${SIM_DISPLAY}"
    fi

    if [ -z "${SIM_XAUTHORITY}" ]; then
        SIM_XAUTHORITY="/home/${SIMULATOR_USER}/.Xauthority"
    fi
}

# SSH로 jaewon의 GUI 환경에서 명령 실행하는 헬퍼
ssh_sim_run() {
    local cmd="$1"
    ${SSH_CMD} ${SIMULATOR_USER}@${SIMULATOR_HOST} \
        "export DISPLAY=${SIM_DISPLAY}; export XAUTHORITY=${SIM_XAUTHORITY}; ${cmd}"
}

# Kill simulator process (SSH로 jaewon 프로세스 종료)
kill_simulator() {
    log_info "시뮬레이터 종료 중..."
    # 항상 jaewon의 시뮬레이터 프로세스 종료 시도 (어떤 모드든)
    ${SSH_CMD} ${SIMULATOR_USER}@${SIMULATOR_HOST} bash -s <<'REMOTE_EOF' 2>/dev/null || true
# 1차: SIGTERM으로 정상 종료 시도
pkill -u jaewon -f 'start_simulation.sh' 2>/dev/null
pkill -u jaewon -f 'run_noiser.sh' 2>/dev/null
pkill -u jaewon -f 'python.sh.*main.py' 2>/dev/null
pkill -u jaewon -f 'kit/python/bin/python3.*main.py' 2>/dev/null
sleep 2
# 2차: 아직 살아있으면 SIGKILL 강제 종료
pkill -9 -u jaewon -f 'start_simulation.sh' 2>/dev/null
pkill -9 -u jaewon -f 'run_noiser.sh' 2>/dev/null
pkill -9 -u jaewon -f 'python.sh.*main.py' 2>/dev/null
pkill -9 -u jaewon -f 'kit/python/bin/python3.*main.py' 2>/dev/null
REMOTE_EOF
    sleep 3
    log_success "시뮬레이터 종료 완료."
}

# Start simulator for a given scenario
start_simulator() {
    local scenario_num=$1
    local flag="--s${scenario_num}"

    if [ "${SKIP_SIM_START}" = true ]; then
        log_info "시뮬레이터 시작 스킵 (--no-sim 모드)"
        return 0
    fi

    if [ "${MANUAL_SIM}" = true ]; then
        echo ""
        echo -e "${YELLOW}═══════════════════════════════════════════════════════════════${NC}"
        echo -e "${YELLOW}  시뮬레이터를 시작해주세요:${NC}"
        echo -e "${BOLD}  ${SIMULATOR_SCRIPT} ${flag}${NC}"
        echo -e "${YELLOW}═══════════════════════════════════════════════════════════════${NC}"
        echo ""
        echo -e "${YELLOW}시뮬레이터 시작 후 Enter를 눌러주세요...${NC}"
        read -r
        return 0
    fi

    # SSH auto mode (jaewon의 GUI 세션 환경에서 실행)
    log_step "시뮬레이터 시작: ${SIMULATOR_SCRIPT} ${flag}"
    log_info "DISPLAY=${SIM_DISPLAY}, XAUTHORITY=${SIM_XAUTHORITY}"

    # SSH -f: 백그라운드로 전환, stdin/stdout 분리하여 SSH가 즉시 반환되도록 함
    ${SSH_CMD} -f ${SIMULATOR_USER}@${SIMULATOR_HOST} \
        "export DISPLAY=${SIM_DISPLAY}; export XAUTHORITY=${SIM_XAUTHORITY}; cd /home/${SIMULATOR_USER}/Lproject_sim && nohup bash start_simulation.sh ${flag} </dev/null > /tmp/sim_s${scenario_num}.log 2>&1 &" \
        2>/dev/null

    if [ $? -ne 0 ]; then
        log_error "SSH로 시뮬레이터 시작 실패. SSH 키를 확인해주세요."
        return 1
    fi

    log_info "시뮬레이터 시작 명령 완료. ${WAIT_AFTER_SIM_START}초 대기..."
    sleep "${WAIT_AFTER_SIM_START}"
}

# Wait for simulator to be ready (check for topics)
wait_for_sim_ready() {
    log_step "시뮬레이터 준비 대기 중... (토픽: ${SIM_CHECK_TOPIC})"

    local elapsed=0
    local check_interval=3

    while [ ${elapsed} -lt ${SIM_READY_TIMEOUT} ]; do
        # Check if the sim topic is being published
        local hz_output
        hz_output=$(timeout 4 ros2 topic hz "${SIM_CHECK_TOPIC}" --window 3 2>/dev/null | head -1 || true)

        if echo "${hz_output}" | grep -q "average rate"; then
            log_success "시뮬레이터 준비 완료! (${elapsed}초 경과)"
            return 0
        fi

        elapsed=$((elapsed + check_interval))
        echo -ne "\r  대기 중... ${elapsed}/${SIM_READY_TIMEOUT}s"
        sleep "${check_interval}"
    done

    echo ""
    log_error "시뮬레이터 준비 타임아웃 (${SIM_READY_TIMEOUT}초)"
    return 1
}

# Start the pipeline
start_pipeline() {
    log_step "파이프라인 시작: ${PIPELINE_CMD}"

    # Remove old metrics file to start fresh for this scenario
    rm -f "${METRICS_FILE}"

    cd "${ROS2_WS}"
    # Run pipeline in background
    bash -c "${PIPELINE_CMD}" &
    PIPELINE_PID=$!

    log_info "파이프라인 PID: ${PIPELINE_PID}"
    log_info "파이프라인 초기화 대기 (${WAIT_AFTER_PIPELINE}초)..."
    sleep "${WAIT_AFTER_PIPELINE}"
}

# Wait for Nav2 to be fully ready (RViz + lifecycle active + action server)
# run_pipeline.sh에서 RViz가 마지막에 실행되므로, RViz가 뜨면 모든 노드가 준비된 상태
wait_for_nav2_ready() {
    log_step "파이프라인 준비 대기 중... (RViz → Nav2 action → bt_navigator active)"

    local elapsed=0
    local check_interval=5
    local rviz_ready=false
    local action_ready=false
    local lifecycle_ready=false

    while [ ${elapsed} -lt ${PIPELINE_READY_TIMEOUT} ]; do
        # Check 1: RViz 프로세스가 실행 중인지 (파이프라인의 마지막 컴포넌트)
        if pgrep -f "rviz2" > /dev/null 2>&1; then
            rviz_ready=true
        fi

        # Check 2: navigate_to_pose action server 존재
        if [ "${rviz_ready}" = true ]; then
            local actions
            actions=$(ros2 action list 2>/dev/null || true)
            if echo "${actions}" | grep -q "navigate_to_pose"; then
                action_ready=true
            fi
        fi

        # Check 3: bt_navigator가 active 상태
        if [ "${action_ready}" = true ]; then
            local bt_state
            bt_state=$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)
            if echo "${bt_state}" | grep -qi "active"; then
                lifecycle_ready=true
            fi
        fi

        if [ "${rviz_ready}" = true ] && [ "${action_ready}" = true ] && [ "${lifecycle_ready}" = true ]; then
            echo ""
            log_success "파이프라인 준비 완료! (${elapsed}초 경과)"
            log_info "  RViz=✓  Action Server=✓  bt_navigator=active"
            log_info "추가 안정화 대기 5초..."
            sleep 5
            return 0
        fi

        elapsed=$((elapsed + check_interval))
        local s1=$([ "${rviz_ready}" = true ] && echo "✓" || echo "✗")
        local s2=$([ "${action_ready}" = true ] && echo "✓" || echo "✗")
        local s3=$([ "${lifecycle_ready}" = true ] && echo "✓" || echo "✗")
        echo -ne "\r  대기 중... ${elapsed}/${PIPELINE_READY_TIMEOUT}s [rviz=${s1} action=${s2} bt_nav=${s3}]    "
        sleep "${check_interval}"
    done

    echo ""
    log_error "파이프라인 준비 타임아웃 (${PIPELINE_READY_TIMEOUT}초)"
    log_error "  RViz: ${rviz_ready}, action_server: ${action_ready}, bt_navigator: ${lifecycle_ready}"
    return 1
}

# Send waypoints and wait for completion
run_navigation() {
    local scenario_num=$1
    local scenario_dir=$2

    log_step "웨이포인트 전송 시작 (${TOTAL_WAYPOINTS}개)..."

    # Create a named pipe for monitoring nav_status
    local status_log="${scenario_dir}/nav_status.log"
    > "${status_log}"

    # Start monitoring nav_status topic in background
    ros2 topic echo /nav_status std_msgs/msg/String --field data >> "${status_log}" 2>/dev/null &
    local monitor_pid=$!

    # Start goal_sender (conda 환경 활성화 필요)
    conda run -n foundation_stereo_py312 --no-capture-output \
        ros2 run nvblox_integration goal_sender --ros-args \
        -p waypoints_file:="${WAYPOINTS_FILE}" \
        -p wait_between_goals:=3.0 \
        -p use_sim_time:=true &
    local goal_sender_pid=$!

    log_info "Goal sender PID: ${goal_sender_pid}"
    log_info "완료 조건: 전체 웨이포인트 주행 완료 OR 시나리오 주행 시간 $((NAV_COMPLETE_TIMEOUT / 60))분 초과"

    # Wait for all waypoints to complete
    local elapsed=0
    local check_interval=5
    local completed_count=0

    while [ ${elapsed} -lt ${NAV_COMPLETE_TIMEOUT} ]; do
        sleep "${check_interval}"
        elapsed=$((elapsed + check_interval))

        # Count completed waypoints (SUCCEEDED or ABORTED)
        completed_count=$(grep -cE "^(SUCCEEDED|ABORTED):" "${status_log}" 2>/dev/null || echo "0")
        local succeeded_count=$(grep -c "^SUCCEEDED:" "${status_log}" 2>/dev/null || echo "0")
        local aborted_count=$(grep -c "^ABORTED:" "${status_log}" 2>/dev/null || echo "0")

        local mins=$((elapsed / 60))
        local secs=$((elapsed % 60))
        echo -ne "\r  진행: ${completed_count}/${TOTAL_WAYPOINTS} (성공: ${succeeded_count}, 실패: ${aborted_count}) | ${mins}m${secs}s / $((NAV_COMPLETE_TIMEOUT / 60))m  "

        if [ "${completed_count}" -ge "${TOTAL_WAYPOINTS}" ]; then
            echo ""
            log_success "전체 네비게이션 완료! (성공: ${succeeded_count}, 실패: ${aborted_count})"
            break
        fi

        # Check if goal_sender process is still alive
        if ! kill -0 "${goal_sender_pid}" 2>/dev/null; then
            echo ""
            log_warn "Goal sender 프로세스 종료됨. 완료: ${completed_count}/${TOTAL_WAYPOINTS}"
            break
        fi
    done

    if [ ${elapsed} -ge ${NAV_COMPLETE_TIMEOUT} ]; then
        echo ""
        log_warn "시나리오 주행 시간 $((NAV_COMPLETE_TIMEOUT / 60))분 초과! 완료된 웨이포인트: ${completed_count}/${TOTAL_WAYPOINTS}"
    fi

    # Cleanup
    kill "${monitor_pid}" 2>/dev/null || true
    kill "${goal_sender_pid}" 2>/dev/null || true
    wait "${monitor_pid}" 2>/dev/null || true
    wait "${goal_sender_pid}" 2>/dev/null || true

    return 0
}

# Save results for a scenario
save_results() {
    local scenario_num=$1
    local scenario_dir=$2

    log_step "결과 저장: ${scenario_dir}/"

    # Copy metrics file
    if [ -f "${METRICS_FILE}" ]; then
        cp "${METRICS_FILE}" "${scenario_dir}/navigation_metrics.csv"
        log_success "메트릭 저장 완료: navigation_metrics.csv"

        # Print summary
        local total_rows=$(tail -n +2 "${METRICS_FILE}" | wc -l)
        local succeeded=$(grep -c "SUCCEEDED" "${METRICS_FILE}" || echo "0")
        local aborted=$(grep -c "ABORTED" "${METRICS_FILE}" || echo "0")
        echo -e "  결과 요약: 총 ${total_rows}개 웨이포인트 | 성공: ${succeeded} | 실패: ${aborted}"
    else
        log_warn "메트릭 파일이 없습니다."
    fi

    # Save waypoints used
    if [ -f "${WAYPOINTS_FILE}" ]; then
        cp "${WAYPOINTS_FILE}" "${scenario_dir}/waypoints.json"
    fi

    # Save pipeline log if exists
    # (pipeline output goes to terminal, but we save what we can)
    echo "scenario: s${scenario_num}" > "${scenario_dir}/scenario_info.txt"
    echo "timestamp: $(timestamp)" >> "${scenario_dir}/scenario_info.txt"
    echo "pipeline_cmd: ${PIPELINE_CMD}" >> "${scenario_dir}/scenario_info.txt"
    echo "waypoints_file: ${WAYPOINTS_FILE}" >> "${scenario_dir}/scenario_info.txt"
    echo "total_waypoints: ${TOTAL_WAYPOINTS}" >> "${scenario_dir}/scenario_info.txt"
}

# Kill all pipeline processes
kill_all() {
    log_step "파이프라인 종료 중..."

    # Kill goal_sender first
    pkill -f "goal_sender" 2>/dev/null || true
    sleep 1

    # Use existing kill script
    if [ -f "${ROS2_WS}/kill_pipeline.sh" ]; then
        bash "${ROS2_WS}/kill_pipeline.sh" 2>/dev/null || true
    else
        # Fallback: manual kill
        pkill -f "run_pipeline.sh" 2>/dev/null || true
        pkill -f "navigation_metrics_logger" 2>/dev/null || true
        pkill -f "nvblox" 2>/dev/null || true
        pkill -f "orb_slam" 2>/dev/null || true
        pkill -f "foundation_stereo" 2>/dev/null || true
        pkill -f "nav2" 2>/dev/null || true
        pkill -f "rviz" 2>/dev/null || true
        pkill -f "controller_server" 2>/dev/null || true
        pkill -f "planner_server" 2>/dev/null || true
        pkill -f "bt_navigator" 2>/dev/null || true
        pkill -f "static_transform_publisher" 2>/dev/null || true
    fi

    # Kill any lingering ros2 topic echo monitors
    pkill -f "ros2 topic echo /nav_status" 2>/dev/null || true

    log_info "종료 후 안정화 대기 (${WAIT_AFTER_KILL}초)..."
    sleep "${WAIT_AFTER_KILL}"
    log_success "정리 완료."
}

# =============================================================================
# Ctrl+C handler
# =============================================================================
global_cleanup() {
    trap - INT TERM
    echo ""
    log_warn "인터럽트 감지! 전체 정리 중..."
    kill_all
    kill_simulator
    echo ""
    log_info "실험 중단됨. 지금까지 저장된 결과: ${RESULTS_BASE_DIR}/"
    exit 130
}
trap global_cleanup INT TERM

# =============================================================================
# Main Loop
# =============================================================================

# Create timestamped run directory
RUN_TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
RUN_DIR="${RESULTS_BASE_DIR}/run_${RUN_TIMESTAMP}"
mkdir -p "${RUN_DIR}"

log_header "실험 시작: $(timestamp)"
log_info "결과 디렉토리: ${RUN_DIR}"

# jaewon의 그래픽 세션 감지 (자동 시뮬레이터 모드에서만)
if [ "${MANUAL_SIM}" = false ] && [ "${SKIP_SIM_START}" = false ]; then
    detect_sim_display
fi

# Summary tracking
declare -A SCENARIO_RESULTS
TOTAL_SUCCEEDED=0
TOTAL_FAILED=0

for idx in "${!SCENARIOS[@]}"; do
    scenario_num=${SCENARIOS[$idx]}
    scenario_idx=$((idx + 1))

    log_header "시나리오 s${scenario_num} (${scenario_idx}/${TOTAL_SCENARIOS})"

    # Create scenario result directory
    SCENARIO_DIR="${RUN_DIR}/s${scenario_num}"
    mkdir -p "${SCENARIO_DIR}"

    echo "시작 시간: $(timestamp)" > "${SCENARIO_DIR}/timing.txt"

    # ── Step 1: 기존 프로세스 정리 ──
    log_step "[1/6] 기존 프로세스 정리"
    kill_all 2>/dev/null || true

    # ── Step 2: 시뮬레이터 시작 ──
    log_step "[2/6] 시뮬레이터 시작 (s${scenario_num})"
    if ! start_simulator "${scenario_num}"; then
        log_error "시뮬레이터 시작 실패. 이 시나리오를 스킵합니다."
        SCENARIO_RESULTS["s${scenario_num}"]="SIM_FAIL"
        TOTAL_FAILED=$((TOTAL_FAILED + 1))
        echo "결과: SIM_START_FAILED" >> "${SCENARIO_DIR}/timing.txt"
        continue
    fi

    # ── Step 3: 시뮬레이터 준비 대기 ──
    log_step "[3/6] 시뮬레이터 준비 확인"
    if ! wait_for_sim_ready; then
        log_error "시뮬레이터가 준비되지 않음. 이 시나리오를 스킵합니다."
        SCENARIO_RESULTS["s${scenario_num}"]="SIM_TIMEOUT"
        TOTAL_FAILED=$((TOTAL_FAILED + 1))
        echo "결과: SIM_READY_TIMEOUT" >> "${SCENARIO_DIR}/timing.txt"
        kill_simulator
        continue
    fi

    # ── Step 4: 파이프라인 시작 ──
    log_step "[4/6] 파이프라인 시작"
    start_pipeline

    if ! wait_for_nav2_ready; then
        log_error "Nav2 준비 타임아웃. 이 시나리오를 스킵합니다."
        SCENARIO_RESULTS["s${scenario_num}"]="NAV2_TIMEOUT"
        TOTAL_FAILED=$((TOTAL_FAILED + 1))
        echo "결과: NAV2_READY_TIMEOUT" >> "${SCENARIO_DIR}/timing.txt"
        kill_all
        kill_simulator
        continue
    fi

    # ── Step 5: 네비게이션 실행 ──
    log_step "[5/6] 네비게이션 실행"
    echo "네비게이션 시작: $(timestamp)" >> "${SCENARIO_DIR}/timing.txt"
    run_navigation "${scenario_num}" "${SCENARIO_DIR}"
    echo "네비게이션 종료: $(timestamp)" >> "${SCENARIO_DIR}/timing.txt"

    # ── Step 6: 결과 저장 & 정리 ──
    log_step "[6/6] 결과 저장 및 정리"
    save_results "${scenario_num}" "${SCENARIO_DIR}"

    echo "종료 시간: $(timestamp)" >> "${SCENARIO_DIR}/timing.txt"

    SCENARIO_RESULTS["s${scenario_num}"]="DONE"
    TOTAL_SUCCEEDED=$((TOTAL_SUCCEEDED + 1))

    # Kill pipeline & simulator before next scenario
    kill_all
    if [ "${SKIP_SIM_START}" = false ]; then
        kill_simulator
    fi

    # Wait between scenarios (except the last one)
    if [ "${scenario_idx}" -lt "${TOTAL_SCENARIOS}" ]; then
        log_info "다음 시나리오까지 ${WAIT_BETWEEN_SCENARIOS}초 대기..."
        sleep "${WAIT_BETWEEN_SCENARIOS}"
    fi
done

# =============================================================================
# Final Summary
# =============================================================================
log_header "실험 완료 - 최종 요약"

echo ""
echo -e "${BOLD}결과 요약:${NC}"
echo -e "  총 시나리오: ${TOTAL_SCENARIOS}"
echo -e "  성공: ${GREEN}${TOTAL_SUCCEEDED}${NC}"
echo -e "  실패: ${RED}${TOTAL_FAILED}${NC}"
echo ""
echo -e "${BOLD}시나리오별 결과:${NC}"

for scenario_num in "${SCENARIOS[@]}"; do
    result="${SCENARIO_RESULTS["s${scenario_num}"]:-UNKNOWN}"
    case "${result}" in
        DONE)
            echo -e "  s${scenario_num}: ${GREEN}✓ 완료${NC}" ;;
        SIM_FAIL)
            echo -e "  s${scenario_num}: ${RED}✗ 시뮬레이터 시작 실패${NC}" ;;
        SIM_TIMEOUT)
            echo -e "  s${scenario_num}: ${RED}✗ 시뮬레이터 타임아웃${NC}" ;;
        NAV2_TIMEOUT)
            echo -e "  s${scenario_num}: ${RED}✗ Nav2 타임아웃${NC}" ;;
        *)
            echo -e "  s${scenario_num}: ${YELLOW}? ${result}${NC}" ;;
    esac
done

echo ""
echo -e "${BOLD}결과 저장 위치:${NC} ${RUN_DIR}/"
echo ""

# Generate combined metrics CSV
COMBINED_METRICS="${RUN_DIR}/all_scenarios_metrics.csv"
FIRST=true
for scenario_num in "${SCENARIOS[@]}"; do
    local_csv="${RUN_DIR}/s${scenario_num}/navigation_metrics.csv"
    if [ -f "${local_csv}" ]; then
        if [ "${FIRST}" = true ]; then
            # Header + add scenario column
            head -1 "${local_csv}" | sed 's/^/scenario,/' > "${COMBINED_METRICS}"
            FIRST=false
        fi
        # Data rows with scenario prefix
        tail -n +2 "${local_csv}" | sed "s/^/s${scenario_num},/" >> "${COMBINED_METRICS}"
    fi
done

if [ -f "${COMBINED_METRICS}" ]; then
    log_success "통합 메트릭 저장: ${COMBINED_METRICS}"
fi

log_header "실험 종료: $(timestamp)"
