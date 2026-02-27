#!/usr/bin/env python3
"""
Nav2 파라미터 파일 생성 스크립트

Footprint 스케일링, 속도 설정, 경로 추종 강건성 파라미터 적용
"""

import argparse
import sys
import os
import re


# 기본 footprint (로버 크기: 1.0m x 0.66m)
BASE_FOOTPRINT = [
    [0.5, 0.33],
    [0.5, -0.33],
    [-0.5, -0.33],
    [-0.5, 0.33]
]


def scale_footprint(base_footprint: list, scale: float) -> str:
    """
    Footprint를 스케일링하여 문자열로 반환
    
    Args:
        base_footprint: [[x1, y1], [x2, y2], ...] 형태의 기본 footprint
        scale: 스케일 팩터 (1.0 = 원본, 1.5 = 50% 확대)
    
    Returns:
        Nav2 footprint 문자열 형식: "[[x1, y1], [x2, y2], ...]"
    """
    scaled = [[round(x * scale, 3), round(y * scale, 3)] for x, y in base_footprint]
    return str(scaled).replace("'", "")


def update_yaml_footprint(yaml_content: str, scaled_footprint: str) -> str:
    """
    YAML 내용에서 footprint 값을 업데이트
    
    Args:
        yaml_content: 원본 YAML 파일 내용
        scaled_footprint: 스케일링된 footprint 문자열
    
    Returns:
        업데이트된 YAML 내용
    """
    # footprint 라인 패턴 매칭 (들여쓰기 유지)
    pattern = r'(^\s*footprint:\s*)".*"'
    replacement = rf'\1"{scaled_footprint}"'
    
    updated = re.sub(pattern, replacement, yaml_content, flags=re.MULTILINE)
    return updated


def update_yaml_goal_tolerance(yaml_content: str, xy_tolerance: float, yaw_tolerance: float) -> str:
    """
    YAML 내용에서 goal tolerance 값을 업데이트
    
    Args:
        yaml_content: 원본 YAML 파일 내용
        xy_tolerance: xy 위치 허용 오차 (meters)
        yaw_tolerance: yaw 방향 허용 오차 (radians)
    
    Returns:
        업데이트된 YAML 내용
    """
    # xy_goal_tolerance 패턴 매칭
    pattern_xy = r'(^\s*xy_goal_tolerance:\s*)[\d.]+'
    replacement_xy = rf'\g<1>{xy_tolerance}'
    updated = re.sub(pattern_xy, replacement_xy, yaml_content, flags=re.MULTILINE)
    
    # yaw_goal_tolerance 패턴 매칭
    pattern_yaw = r'(^\s*yaw_goal_tolerance:\s*)[\d.]+'
    replacement_yaw = rf'\g<1>{yaw_tolerance}'
    updated = re.sub(pattern_yaw, replacement_yaw, updated, flags=re.MULTILINE)
    
    return updated


def update_yaml_velocity(yaml_content: str, max_linear: float, max_angular: float,
                        linear_accel: float, angular_accel: float) -> str:
    """
    YAML 내용에서 속도 관련 파라미터를 업데이트
    
    모든 Nav2 컴포넌트에서 일관된 속도 설정을 유지:
    - controller_server (RPP): desired_linear_vel, max_angular_vel, max_linear_accel/decel
    - controller_server (DWB): max_vel_x, max_vel_theta, acc_lim_x, acc_lim_theta
    - RotationShimController: rotate_to_heading_angular_vel, max_angular_accel
    - velocity_smoother: max_velocity, max_accel
    - behavior_server: max_rotational_vel
    
    Args:
        yaml_content: 원본 YAML 파일 내용
        max_linear: 최대 선형 속도 (m/s)
        max_angular: 최대 각속도 (rad/s)
        linear_accel: 선형 가속도 (m/s^2)
        angular_accel: 각 가속도 (rad/s^2)
    
    Returns:
        업데이트된 YAML 내용
    """
    updated = yaml_content
    
    # =========================================================================
    # RPP (Regulated Pure Pursuit) Controller 속도 파라미터
    # =========================================================================
    
    # desired_linear_vel: 목표 선속도
    pattern = r'(^\s*desired_linear_vel:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{max_linear}', updated, flags=re.MULTILINE)
    
    # max_angular_vel: 최대 각속도 (RPP)
    pattern = r'(^\s*max_angular_vel:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{max_angular}', updated, flags=re.MULTILINE)
    
    # max_linear_accel: 최대 선가속도 (RPP)
    pattern = r'(^\s*max_linear_accel:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{linear_accel}', updated, flags=re.MULTILINE)
    
    # max_linear_decel: 최대 선감속도 (RPP)
    pattern = r'(^\s*max_linear_decel:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{linear_accel}', updated, flags=re.MULTILINE)
    
    # max_angular_accel: 최대 각가속도 (RPP)
    pattern = r'(^\s*max_angular_accel:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{angular_accel}', updated, flags=re.MULTILINE)
    
    # =========================================================================
    # NOTE: RotationShimController 파라미터는 더 이상 사용하지 않음
    # 대신 initial_rotation_controller 노드가 초기 회전을 담당
    # =========================================================================
    
    # =========================================================================
    # DWB Controller 속도 파라미터 (fallback/legacy)
    # =========================================================================
    
    # max_vel_x
    pattern = r'(^\s*max_vel_x:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{max_linear}', updated, flags=re.MULTILINE)
    
    # max_vel_theta
    pattern = r'(^\s*max_vel_theta:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{max_angular}', updated, flags=re.MULTILINE)
    
    # max_speed_xy
    pattern = r'(^\s*max_speed_xy:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{max_linear}', updated, flags=re.MULTILINE)
    
    # acc_lim_x
    pattern = r'(^\s*acc_lim_x:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{linear_accel}', updated, flags=re.MULTILINE)
    
    # acc_lim_theta
    pattern = r'(^\s*acc_lim_theta:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{angular_accel}', updated, flags=re.MULTILINE)
    
    # decel_lim_x
    pattern = r'(^\s*decel_lim_x:\s*)-?[\d.]+'
    updated = re.sub(pattern, rf'\g<1>-{linear_accel}', updated, flags=re.MULTILINE)
    
    # decel_lim_theta
    pattern = r'(^\s*decel_lim_theta:\s*)-?[\d.]+'
    updated = re.sub(pattern, rf'\g<1>-{angular_accel}', updated, flags=re.MULTILINE)
    
    # =========================================================================
    # velocity_smoother: 최종 cmd_vel 출력 제한 (가장 중요!)
    # =========================================================================
    
    # max_velocity [linear_x, linear_y, angular_z]
    pattern = r'(^\s*max_velocity:\s*\[)[\d.]+,\s*[\d.]+,\s*[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{max_linear}, 0.0, {max_angular}', updated, flags=re.MULTILINE)
    
    # min_velocity [-linear_x, linear_y, -angular_z]
    pattern = r'(^\s*min_velocity:\s*\[)-?[\d.]+,\s*[\d.]+,\s*-?[\d.]+'
    updated = re.sub(pattern, rf'\g<1>-{max_linear}, 0.0, -{max_angular}', updated, flags=re.MULTILINE)
    
    # max_accel
    pattern = r'(^\s*max_accel:\s*\[)[\d.]+,\s*[\d.]+,\s*[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{linear_accel}, 0.0, {angular_accel}', updated, flags=re.MULTILINE)
    
    # max_decel
    pattern = r'(^\s*max_decel:\s*\[)-?[\d.]+,\s*[\d.]+,\s*-?[\d.]+'
    updated = re.sub(pattern, rf'\g<1>-{linear_accel}, 0.0, -{angular_accel}', updated, flags=re.MULTILINE)
    
    # =========================================================================
    # behavior_server: 복구 동작 속도
    # =========================================================================
    
    # max_rotational_vel
    pattern = r'(^\s*max_rotational_vel:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{max_angular}', updated, flags=re.MULTILINE)
    
    # behavior_server: rotational_acc_lim
    pattern = r'(^\s*rotational_acc_lim:\s*)[\d.]+'
    updated = re.sub(pattern, rf'\g<1>{angular_accel}', updated, flags=re.MULTILINE)
    
    return updated


def update_yaml_robust_path_following(yaml_content: str) -> str:
    """
    경로 추종 강건성을 높이기 위한 파라미터 조정
    
    - sim_time 증가: 더 긴 예측 궤적으로 부드러운 경로 추종
    - PathAlign/PathDist 스케일 증가: 경로를 더 잘 따르도록
    - transform_tolerance 증가: tf 지연에 대한 내성
    - progress_checker 완화: 일시적인 정체에 대한 내성
    
    Args:
        yaml_content: 원본 YAML 파일 내용
    
    Returns:
        업데이트된 YAML 내용
    """
    updated = yaml_content
    
    # DWB sim_time: 더 긴 예측 (1.7 -> 2.5)
    pattern = r'(^\s*sim_time:\s*)[\d.]+'
    updated = re.sub(pattern, r'\g<1>2.5', updated, flags=re.MULTILINE)
    
    # PathAlign.scale: 경로 정렬 가중치 증가 (32.0 -> 48.0)
    pattern = r'(^\s*PathAlign\.scale:\s*)[\d.]+'
    updated = re.sub(pattern, r'\g<1>48.0', updated, flags=re.MULTILINE)
    
    # PathDist.scale: 경로 거리 가중치 증가 (32.0 -> 48.0)
    pattern = r'(^\s*PathDist\.scale:\s*)[\d.]+'
    updated = re.sub(pattern, r'\g<1>48.0', updated, flags=re.MULTILINE)
    
    # GoalDist.scale: 목표 거리 가중치 감소 (24.0 -> 16.0) - 경로 추종 우선
    pattern = r'(^\s*GoalDist\.scale:\s*)[\d.]+'
    updated = re.sub(pattern, r'\g<1>16.0', updated, flags=re.MULTILINE)
    
    # progress_checker movement_time_allowance 증가 (15.0 -> 20.0)
    pattern = r'(^\s*movement_time_allowance:\s*)[\d.]+'
    updated = re.sub(pattern, r'\g<1>20.0', updated, flags=re.MULTILINE)
    
    # progress_checker required_movement_radius 감소 (0.3 -> 0.2)
    pattern = r'(^\s*required_movement_radius:\s*)[\d.]+'
    updated = re.sub(pattern, r'\g<1>0.2', updated, flags=re.MULTILINE)
    
    # controller_patience 증가 (10.0 -> 15.0)
    pattern = r'(^\s*controller_patience:\s*)[\d.]+'
    updated = re.sub(pattern, r'\g<1>15.0', updated, flags=re.MULTILINE)
    
    # failure_tolerance 증가 (1.0 -> 2.0)
    pattern = r'(^\s*failure_tolerance:\s*)[\d.]+'
    updated = re.sub(pattern, r'\g<1>2.0', updated, flags=re.MULTILINE)
    
    return updated


def main():
    parser = argparse.ArgumentParser(
        description='Nav2 파라미터 파일 생성 (footprint 스케일링, 속도 설정, 경로 추종 강건성)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
예제:
  # Footprint를 1.5배로 확대
  %(prog)s --input sim_nav2.yaml --output /tmp/nav2_params.yaml --footprint-scale 1.5
  
  # Goal tolerance도 함께 조정
  %(prog)s --input sim_nav2.yaml --output /tmp/nav2_params.yaml \\
           --footprint-scale 1.2 --xy-tolerance 0.5 --yaw-tolerance 0.5
  
  # 속도 설정 (Isaac Sim과 동일하게)
  %(prog)s --input sim_nav2.yaml --output /tmp/nav2_params.yaml \\
           --max-linear-vel 0.5 --max-angular-vel 1.5
  
  # 스케일링된 footprint만 출력
  %(prog)s --footprint-scale 1.5 --print-footprint
        '''
    )
    
    parser.add_argument('--input', '-i', type=str,
                        help='입력 Nav2 YAML 파일 경로')
    parser.add_argument('--output', '-o', type=str,
                        help='출력 Nav2 YAML 파일 경로')
    parser.add_argument('--footprint-scale', '-s', type=float, default=1.0,
                        help='Footprint 스케일 팩터 (기본값: 1.0)')
    parser.add_argument('--xy-tolerance', type=float,
                        help='XY goal tolerance (meters)')
    parser.add_argument('--yaw-tolerance', type=float,
                        help='Yaw goal tolerance (radians)')
    parser.add_argument('--max-linear-vel', type=float, default=0.5,
                        help='최대 선형 속도 m/s (기본값: 0.5)')
    parser.add_argument('--max-angular-vel', type=float, default=1.5,
                        help='최대 각속도 rad/s (기본값: 1.5)')
    parser.add_argument('--linear-accel', type=float, default=2.5,
                        help='선형 가속도 m/s^2 (기본값: 2.5)')
    parser.add_argument('--angular-accel', type=float, default=3.2,
                        help='각 가속도 rad/s^2 (기본값: 3.2)')
    parser.add_argument('--no-robust-params', action='store_true',
                        help='경로 추종 강건성 파라미터 비활성화')
    parser.add_argument('--print-footprint', action='store_true',
                        help='스케일링된 footprint만 출력하고 종료')
    parser.add_argument('--base-footprint', type=str,
                        help='커스텀 기본 footprint (예: "[[0.5,0.3],[0.5,-0.3],[-0.5,-0.3],[-0.5,0.3]]")')
    parser.add_argument('--bt-xml', type=str,
                        help='BT XML 파일 경로 (default_nav_to_pose_bt_xml 설정)')
    
    args = parser.parse_args()
    
    # 기본 footprint 설정
    if args.base_footprint:
        try:
            base_fp = eval(args.base_footprint)
        except:
            print(f"Error: Invalid footprint format: {args.base_footprint}", file=sys.stderr)
            sys.exit(1)
    else:
        base_fp = BASE_FOOTPRINT
    
    # 스케일링된 footprint 계산
    scaled_footprint = scale_footprint(base_fp, args.footprint_scale)
    
    # footprint만 출력하는 모드
    if args.print_footprint:
        print(scaled_footprint)
        return
    
    # 파일 처리 모드
    if not args.input:
        print("Error: --input 옵션이 필요합니다.", file=sys.stderr)
        sys.exit(1)
    
    if not args.output:
        print("Error: --output 옵션이 필요합니다.", file=sys.stderr)
        sys.exit(1)
    
    # 입력 파일 읽기
    if not os.path.exists(args.input):
        print(f"Error: 입력 파일을 찾을 수 없습니다: {args.input}", file=sys.stderr)
        sys.exit(1)
    
    with open(args.input, 'r') as f:
        yaml_content = f.read()
    
    # Footprint 업데이트
    yaml_content = update_yaml_footprint(yaml_content, scaled_footprint)
    
    # Goal tolerance 업데이트 (옵션)
    if args.xy_tolerance is not None or args.yaw_tolerance is not None:
        xy_tol = args.xy_tolerance if args.xy_tolerance is not None else 0.5
        yaw_tol = args.yaw_tolerance if args.yaw_tolerance is not None else 0.5
        yaml_content = update_yaml_goal_tolerance(yaml_content, xy_tol, yaw_tol)
    
    # 속도 파라미터 업데이트
    yaml_content = update_yaml_velocity(
        yaml_content,
        args.max_linear_vel,
        args.max_angular_vel,
        args.linear_accel,
        args.angular_accel
    )
    
    # 경로 추종 강건성 파라미터 업데이트
    if not args.no_robust_params:
        yaml_content = update_yaml_robust_path_following(yaml_content)
    
    # BT XML 경로 업데이트
    if args.bt_xml:
        bt_xml_pattern = r'(^\s*default_nav_to_pose_bt_xml:\s*).*$'
        bt_xml_replacement = rf'\g<1>"{args.bt_xml}"'
        yaml_content = re.sub(bt_xml_pattern, bt_xml_replacement, yaml_content, flags=re.MULTILINE)
    
    # 출력 디렉토리 생성
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 출력 파일 쓰기
    with open(args.output, 'w') as f:
        f.write(yaml_content)
    
    # 결과 출력
    print(f"Generated Nav2 params: {args.output}")
    print(f"  Footprint scale: {args.footprint_scale}x")
    print(f"  Scaled footprint: {scaled_footprint}")
    print(f"  Max linear vel: {args.max_linear_vel} m/s")
    print(f"  Max angular vel: {args.max_angular_vel} rad/s")
    print(f"  Linear accel: {args.linear_accel} m/s^2")
    print(f"  Angular accel: {args.angular_accel} rad/s^2")
    if args.xy_tolerance is not None:
        print(f"  XY tolerance: {args.xy_tolerance}m")
    if args.yaw_tolerance is not None:
        print(f"  Yaw tolerance: {args.yaw_tolerance}rad")
    if not args.no_robust_params:
        print(f"  Robust path following: enabled")


if __name__ == '__main__':
    main()
