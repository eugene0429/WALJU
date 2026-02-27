#!/bin/bash
# =============================================================================
# 동영상 특정 구간 자르기 + 압축 + 배속 스크립트
# 사용법:
#   ./trim_and_compress_video.sh <입력파일> <시작시간> <종료시간> [출력파일] [품질] [배속]
#
# 예시:
#   ./trim_and_compress_video.sh input.mp4 00:01:30 00:03:45
#   ./trim_and_compress_video.sh input.mp4 00:01:30 00:03:45 output.mp4
#   ./trim_and_compress_video.sh input.mp4 00:01:30 00:03:45 output.mp4 medium
#   ./trim_and_compress_video.sh input.mp4 00:01:30 00:03:45 output.mp4 medium 2x
#   ./trim_and_compress_video.sh input.mp4 90 225           # 초 단위도 가능
#
# 품질 옵션: high, medium(기본), low, ultralow
# 배속 옵션: 0.5x, 1x(기본), 1.5x, 2x, 3x, 4x, 8x 등
# =============================================================================

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# ── 사용법 출력 ──────────────────────────────────────────────────────────────
usage() {
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN} 동영상 구간 자르기 + 압축 + 배속 스크립트${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "사용법: $0 ${GREEN}<입력파일> <시작시간> <종료시간>${NC} [출력파일] [품질] [배속]"
    echo ""
    echo -e "${YELLOW}시간 형식:${NC}"
    echo "  HH:MM:SS     예: 00:01:30"
    echo "  MM:SS        예: 01:30"
    echo "  초(seconds)  예: 90"
    echo ""
    echo -e "${YELLOW}품질 옵션:${NC}"
    echo "  high      - 고품질 (CRF 20, 원본 해상도)"
    echo "  medium    - 중간품질 (CRF 26, 원본 해상도) [기본값]"
    echo "  low       - 저품질 (CRF 30, 720p 스케일다운)"
    echo "  ultralow  - 최저품질 (CRF 35, 480p 스케일다운)"
    echo ""
    echo -e "${YELLOW}배속 옵션:${NC}"
    echo "  0.25x     - 0.25배속 (슬로우모션)"
    echo "  0.5x      - 0.5배속 (슬로우모션)"
    echo "  1x        - 원래 속도 [기본값]"
    echo "  1.5x      - 1.5배속"
    echo "  2x        - 2배속"
    echo "  3x        - 3배속"
    echo "  4x        - 4배속"
    echo "  8x        - 8배속 (타임랩스)"
    echo "  * 숫자 뒤에 x를 붙이면 됩니다 (예: 2.5x)"
    echo ""
    echo -e "${YELLOW}예시:${NC}"
    echo "  $0 recording.mp4 00:01:30 00:03:45"
    echo "  $0 recording.mp4 00:01:30 00:03:45 trimmed.mp4"
    echo "  $0 recording.mp4 00:01:30 00:03:45 trimmed.mp4 low"
    echo "  $0 recording.mp4 00:01:30 00:03:45 trimmed.mp4 medium 2x"
    echo "  $0 recording.mp4 00:01:30 00:03:45 trimmed.mp4 low 4x"
    echo "  $0 recording.mp4 30 120 output.mp4 medium 0.5x"
    echo ""
    exit 1
}

# ── ffmpeg 설치 확인 ─────────────────────────────────────────────────────────
check_ffmpeg() {
    if ! command -v ffmpeg &> /dev/null; then
        echo -e "${RED}[오류] ffmpeg가 설치되어 있지 않습니다.${NC}"
        echo "설치 방법:"
        echo "  Ubuntu/Debian: sudo apt install ffmpeg"
        echo "  macOS:         brew install ffmpeg"
        exit 1
    fi
}

# ── 파일 크기를 사람이 읽기 좋은 형태로 변환 ────────────────────────────────
human_size() {
    local bytes=$1
    if (( bytes >= 1073741824 )); then
        echo "$(awk "BEGIN {printf \"%.2f GB\", $bytes/1073741824}")"
    elif (( bytes >= 1048576 )); then
        echo "$(awk "BEGIN {printf \"%.2f MB\", $bytes/1048576}")"
    elif (( bytes >= 1024 )); then
        echo "$(awk "BEGIN {printf \"%.2f KB\", $bytes/1024}")"
    else
        echo "${bytes} B"
    fi
}

# ── 동영상 길이(초) 가져오기 ─────────────────────────────────────────────────
get_duration() {
    ffprobe -v error -show_entries format=duration \
        -of default=noprint_wrappers=1:nokey=1 "$1" 2>/dev/null
}

# ── 동영상 정보 출력 ─────────────────────────────────────────────────────────
print_video_info() {
    local file="$1"
    echo -e "${CYAN}── 입력 파일 정보 ──${NC}"
    echo -e "  파일: ${GREEN}$(basename "$file")${NC}"

    local size
    size=$(stat -c%s "$file" 2>/dev/null || stat -f%z "$file" 2>/dev/null)
    echo -e "  크기: $(human_size "$size")"

    local duration
    duration=$(get_duration "$file")
    if [[ -n "$duration" ]]; then
        local dur_int=${duration%.*}
        local h=$((dur_int / 3600))
        local m=$(( (dur_int % 3600) / 60 ))
        local s=$((dur_int % 60))
        printf "  길이: %02d:%02d:%02d\n" "$h" "$m" "$s"
    fi

    local resolution
    resolution=$(ffprobe -v error -select_streams v:0 \
        -show_entries stream=width,height \
        -of csv=s=x:p=0 "$file" 2>/dev/null)
    [[ -n "$resolution" ]] && echo -e "  해상도: ${resolution}"

    local codec
    codec=$(ffprobe -v error -select_streams v:0 \
        -show_entries stream=codec_name \
        -of default=noprint_wrappers=1:nokey=1 "$file" 2>/dev/null)
    [[ -n "$codec" ]] && echo -e "  코덱: ${codec}"
    echo ""
}

# ── 인자 검증 ────────────────────────────────────────────────────────────────
[[ $# -lt 3 ]] && usage

INPUT_FILE="$1"
START_TIME="$2"
END_TIME="$3"
QUALITY="${5:-medium}"
SPEED_INPUT="${6:-1x}"

# 배속 값 파싱 ("2x" → 2, "0.5x" → 0.5, "3" → 3)
SPEED=$(echo "$SPEED_INPUT" | sed 's/[xX]$//')

# 배속 값 검증
if ! awk "BEGIN {v=$SPEED+0; exit !(v > 0 && v <= 100)}" 2>/dev/null; then
    echo -e "${RED}[오류] 잘못된 배속 값: ${SPEED_INPUT}${NC}"
    echo "0보다 크고 100 이하의 숫자를 입력하세요 (예: 0.5x, 2x, 4x)"
    exit 1
fi

# 입력 파일 존재 확인
if [[ ! -f "$INPUT_FILE" ]]; then
    echo -e "${RED}[오류] 입력 파일을 찾을 수 없습니다: ${INPUT_FILE}${NC}"
    exit 1
fi

# 출력 파일명 자동 생성
if [[ -n "$4" ]]; then
    OUTPUT_FILE="$4"
else
    BASENAME=$(basename "$INPUT_FILE")
    NAME="${BASENAME%.*}"
    EXT="${BASENAME##*.}"
    SPEED_LABEL=$(echo "${SPEED}x" | sed 's/\./_/g')
    if [[ "$SPEED" == "1" ]]; then
        OUTPUT_FILE="${NAME}_trimmed_${QUALITY}.${EXT}"
    else
        OUTPUT_FILE="${NAME}_trimmed_${QUALITY}_${SPEED_LABEL}.${EXT}"
    fi
fi

check_ffmpeg

# ── 품질 설정 ────────────────────────────────────────────────────────────────
SCALE_PART=""
case "$QUALITY" in
    high)
        CRF=20
        PRESET="slow"
        AUDIO_BITRATE="192k"
        ;;
    medium)
        CRF=26
        PRESET="medium"
        AUDIO_BITRATE="128k"
        ;;
    low)
        CRF=30
        PRESET="fast"
        SCALE_PART="scale=-2:720"
        AUDIO_BITRATE="96k"
        ;;
    ultralow)
        CRF=35
        PRESET="veryfast"
        SCALE_PART="scale=-2:480"
        AUDIO_BITRATE="64k"
        ;;
    *)
        echo -e "${RED}[오류] 알 수 없는 품질 옵션: ${QUALITY}${NC}"
        echo "사용 가능: high, medium, low, ultralow"
        exit 1
        ;;
esac

# ── 배속 + 스케일 필터 조합 ──────────────────────────────────────────────────
# setpts: 영상 속도 (1/배속), atempo: 오디오 속도 (배속)
VIDEO_FILTERS=""
AUDIO_FILTERS=""
IS_SPEED_CHANGED=$(awk "BEGIN {print ($SPEED != 1) ? 1 : 0}")

if [[ "$IS_SPEED_CHANGED" == "1" ]]; then
    # 영상 속도 필터: setpts=PTS*(1/speed)
    PTS_FACTOR=$(awk "BEGIN {printf \"%.6f\", 1.0/$SPEED}")
    SPEED_VF="setpts=${PTS_FACTOR}*PTS"

    # 오디오 속도 필터: atempo는 0.5~2.0 범위만 지원하므로 체이닝 필요
    # 예: 4x → atempo=2.0,atempo=2.0 / 0.25x → atempo=0.5,atempo=0.5
    build_atempo_chain() {
        local spd=$1
        local chain=""
        local remaining
        remaining=$(awk "BEGIN {printf \"%.6f\", $spd}")

        # 배속 > 1 인 경우
        while awk "BEGIN {exit !($remaining > 2.0)}" 2>/dev/null; do
            [[ -n "$chain" ]] && chain="${chain},"
            chain="${chain}atempo=2.0"
            remaining=$(awk "BEGIN {printf \"%.6f\", $remaining/2.0}")
        done

        # 배속 < 1 인 경우
        while awk "BEGIN {exit !($remaining < 0.5)}" 2>/dev/null; do
            [[ -n "$chain" ]] && chain="${chain},"
            chain="${chain}atempo=0.5"
            remaining=$(awk "BEGIN {printf \"%.6f\", $remaining/0.5}")
        done

        # 남은 값 처리
        if awk "BEGIN {exit !($remaining != 1.0)}" 2>/dev/null; then
            [[ -n "$chain" ]] && chain="${chain},"
            chain="${chain}atempo=${remaining}"
        fi

        echo "$chain"
    }

    ATEMPO_CHAIN=$(build_atempo_chain "$SPEED")

    if [[ -n "$SCALE_PART" ]]; then
        VIDEO_FILTERS="-vf ${SCALE_PART},${SPEED_VF}"
    else
        VIDEO_FILTERS="-vf ${SPEED_VF}"
    fi

    if [[ -n "$ATEMPO_CHAIN" ]]; then
        AUDIO_FILTERS="-af ${ATEMPO_CHAIN}"
    fi
else
    # 배속 없음 (1x)
    if [[ -n "$SCALE_PART" ]]; then
        VIDEO_FILTERS="-vf ${SCALE_PART}"
    fi
fi

# ── 작업 정보 출력 ───────────────────────────────────────────────────────────
echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN} 동영상 구간 자르기 + 압축${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

print_video_info "$INPUT_FILE"

echo -e "${CYAN}── 작업 설정 ──${NC}"
echo -e "  구간:   ${GREEN}${START_TIME}${NC} → ${GREEN}${END_TIME}${NC}"
echo -e "  품질:   ${YELLOW}${QUALITY}${NC} (CRF=${CRF}, preset=${PRESET})"
if [[ "$IS_SPEED_CHANGED" == "1" ]]; then
    echo -e "  배속:   ${YELLOW}${SPEED}x${NC}"
    [[ -n "$VIDEO_FILTERS" ]] && echo -e "  영상필터: ${YELLOW}${VIDEO_FILTERS}${NC}"
    [[ -n "$AUDIO_FILTERS" ]] && echo -e "  오디오필터: ${YELLOW}${AUDIO_FILTERS}${NC}"
else
    echo -e "  배속:   1x (원본 속도)"
fi
[[ -n "$SCALE_PART" ]] && echo -e "  스케일: ${YELLOW}${SCALE_PART}${NC}"
echo -e "  출력:   ${GREEN}${OUTPUT_FILE}${NC}"
echo ""

# ── 실행 ─────────────────────────────────────────────────────────────────────
echo -e "${YELLOW}[작업중] 동영상 자르기 + 압축 진행중...${NC}"
echo ""

ffmpeg -y -hide_banner \
    -ss "$START_TIME" -to "$END_TIME" \
    -i "$INPUT_FILE" \
    -c:v libx264 -crf "$CRF" -preset "$PRESET" \
    $VIDEO_FILTERS \
    -c:a aac -b:a "$AUDIO_BITRATE" \
    $AUDIO_FILTERS \
    -movflags +faststart \
    "$OUTPUT_FILE"

# ── 결과 출력 ────────────────────────────────────────────────────────────────
if [[ -f "$OUTPUT_FILE" ]]; then
    echo ""
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN} ✅ 완료!${NC}"
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""

    INPUT_SIZE=$(stat -c%s "$INPUT_FILE" 2>/dev/null || stat -f%z "$INPUT_FILE" 2>/dev/null)
    OUTPUT_SIZE=$(stat -c%s "$OUTPUT_FILE" 2>/dev/null || stat -f%z "$OUTPUT_FILE" 2>/dev/null)

    echo -e "  원본 크기:  $(human_size "$INPUT_SIZE")"
    echo -e "  출력 크기:  $(human_size "$OUTPUT_SIZE")"

    if [[ "$INPUT_SIZE" -gt 0 ]]; then
        RATIO=$(awk "BEGIN {printf \"%.1f\", (1 - $OUTPUT_SIZE/$INPUT_SIZE) * 100}")
        echo -e "  압축률:     ${CYAN}${RATIO}%${NC} 줄어듦"
    fi

    echo -e "  출력 파일:  ${GREEN}${OUTPUT_FILE}${NC}"
    echo ""
else
    echo -e "${RED}[오류] 출력 파일 생성에 실패했습니다.${NC}"
    exit 1
fi
