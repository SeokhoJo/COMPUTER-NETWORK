import os
import csv
import subprocess
import traci

print("### NEW BASELINE CODE RUNNING ###")

# =========================
# 경로 설정
# =========================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

PROJECT_ROOT = os.path.expanduser("~/projects/computerNetwork")

SUMO_DIR = os.path.join(PROJECT_ROOT, "sumo")
NS3_DIR = os.path.join(PROJECT_ROOT, "ns3", "ns-allinone-3.39", "ns-3.39")

SUMO_CFG = os.path.join(SUMO_DIR, "straight.sumocfg")
ROUTE_FILE = os.path.join(SUMO_DIR, "baseline_platoon.rou.xml")

# 🔥 여기 변경
RESULT_ROOT = os.path.join(PROJECT_ROOT, "result")
RESULT_DIR = os.path.join(RESULT_ROOT, "baseline")

LOG_DIR = os.path.join(RESULT_DIR, "baseline_logs")
SUMMARY_FILE = os.path.join(RESULT_DIR, "baseline_summary.csv")

os.makedirs(LOG_DIR, exist_ok=True)

# =========================
# 실험 설정
# =========================
SPACING_CASES = [10, 15, 20, 25, 30, 35, 40]   # m
VEHICLE_COUNT = 5
VEH_IDS = [f"car{i}" for i in range(VEHICLE_COUNT)]

STEP_LENGTH = 0.1
MAX_STEPS = 3000

# 모든 차량이 정지한 뒤 1초 후 종료
POST_STOP_HOLD_SEC = 2.0
POST_STOP_HOLD_STEPS = int(POST_STOP_HOLD_SEC / STEP_LENGTH)

# 100 km/h = 27.78 m/s
PLATOON_CRUISE_SPEED = 27.78

# baseline용 물리 제동 모델
LEAD_BRAKE_DECEL = 8.5       # m/s^2
FOLLOWER_BRAKE_DECEL = 6.0   # m/s^2

# 반응 시간
FAST_REACTION_SEC = 0.1
SLOW_REACTION_SEC = 0.5

# 5대가 모두 보이고 충분히 더 달린 뒤 급제동
EVENT_EXTRA_TRAVEL_AFTER_ALL_VISIBLE = 10.0

# 충돌 시 추가로 얼마나 gap이 더 필요했는지 추정용 기준
TARGET_SAFE_CLEARANCE = 2.0

LEAD_VEH = "car0"

# =========================
# 색상
# =========================
COLOR_LEAD = (255, 255, 0, 255)          # 노랑
COLOR_FOLLOWER = (80, 160, 255, 255)     # 파랑
COLOR_BRAKING = (255, 165, 0, 255)       # 주황
COLOR_COLLISION = (255, 0, 0, 255)       # 빨강
COLOR_STOPPED = (180, 180, 180, 255)     # 회색

# =========================
# 직선 맵 route
# =========================
OUTER_ROUTE_EDGES = "e0"

# =========================
# helper
# =========================
def write_route_file(spacing_m: float):
    """
    spacing_m = 실제 bumper-to-bumper 목표 간격
    차량 길이 포함해서 위치 계산
    """
    vehicle_length = 5.0
    leader_pos = 200.0  # 충분히 큰 값

    with open(ROUTE_FILE, "w", encoding="utf-8") as f:
        f.write(f"""<routes>
    <vType id="platoonCar"
           accel="2.8"
           decel="6.0"
           emergencyDecel="9.0"
           sigma="0.0"
           tau="0.1"
           length="{vehicle_length}"
           minGap="0.1"
           maxSpeed="{PLATOON_CRUISE_SPEED:.2f}"
           color="0,0,255"/>
    <route id="outerLoop" edges="{OUTER_ROUTE_EDGES}"/>
""")

        for i in range(VEHICLE_COUNT):
            pos = leader_pos - i * (spacing_m + vehicle_length)
            color = "255,255,0" if i == 0 else "80,160,255"

            f.write(f"""    <vehicle id="car{i}" type="platoonCar" route="outerLoop"
             depart="0.00"
             departLane="0"
             departPos="{pos:.2f}"
             departSpeed="{PLATOON_CRUISE_SPEED:.2f}"
             color="{color}"/>
""")

        f.write("</routes>\n")

def set_vehicle_color_if_exists(veh_id, color):
    try:
        if veh_id in traci.vehicle.getIDList():
            traci.vehicle.setColor(veh_id, color)
    except Exception:
        pass


def veh_to_node(veh_id: str) -> str:
    idx = int(veh_id.replace("car", ""))
    return f"node{idx}"


def all_platoon_visible():
    ids = set(traci.vehicle.getIDList())
    return all(v in ids for v in VEH_IDS)


def all_platoon_stopped():
    ids = set(traci.vehicle.getIDList())
    if not all(v in ids for v in VEH_IDS):
        return False
    for v in VEH_IDS:
        if traci.vehicle.getSpeed(v) >= 0.1:
            return False
    return True


def get_route_gap(follower_id: str, leader_id: str):
    """
    follower에서 leader 위치까지의 route 상 gap
    """
    ids = set(traci.vehicle.getIDList())
    if follower_id not in ids or leader_id not in ids:
        return None

    try:
        leader_edge = traci.vehicle.getRoadID(leader_id)
        leader_lane_index = traci.vehicle.getLaneIndex(leader_id)
        leader_lane_pos = traci.vehicle.getLanePosition(leader_id)
        leader_length = traci.vehicle.getLength(leader_id)

        driving_dist = traci.vehicle.getDrivingDistance(
            follower_id,
            leader_edge,
            leader_lane_pos,
            leader_lane_index
        )

        if driving_dist is None:
            return None

        gap = driving_dist - leader_length
        return max(0.0, gap)
    except Exception:
        return None


def get_pair_gaps():
    return {
        "car0_car1": get_route_gap("car1", "car0"),
        "car1_car2": get_route_gap("car2", "car1"),
        "car2_car3": get_route_gap("car3", "car2"),
        "car3_car4": get_route_gap("car4", "car3"),
    }


def get_pair_speeds():
    ids = set(traci.vehicle.getIDList())
    result = {}
    for v in VEH_IDS:
        result[v] = traci.vehicle.getSpeed(v) if v in ids else None
    return result


def estimate_extra_gap_needed(collision: bool, min_gap: float):
    if not collision:
        return 0.0
    if min_gap is None:
        return None
    return max(0.0, TARGET_SAFE_CLEARANCE - min_gap)


def save_step_log(path, rows):
    fieldnames = [
        "step", "sim_time",
        "all_visible",
        "event_triggered",
        "collision_happened",
        "all_stopped",
        "car0_speed", "car1_speed", "car2_speed", "car3_speed", "car4_speed",
        "gap_01", "gap_12", "gap_23", "gap_34"
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def append_summary(summary_row, write_header=False):
    fieldnames = [
        "scenario_spacing_m",
        "event_step",
        "event_time_s",
        "event_gap_01_m",
        "event_gap_12_m",
        "event_gap_23_m",
        "event_gap_34_m",
        "collision",
        "collision_pairs",
        "success_no_collision",
        "min_gap_01_m",
        "min_gap_12_m",
        "min_gap_23_m",
        "min_gap_34_m",
        "final_gap_01_m",
        "final_gap_12_m",
        "final_gap_23_m",
        "final_gap_34_m",
        "min_final_gap_m",
        "extra_gap_needed_est_m",
        "receivers",
        "reaction_map",
        "end_reason"
    ]
    with open(SUMMARY_FILE, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        writer.writerow(summary_row)


def run_ns3_normal(spacing_m: float):
    cmd = [
        "./ns3",
        "run",
        f"scratch/v2v-wireless --numNodes=5 --distance={spacing_m} --simTime=3"
    ]

    result = subprocess.run(
        cmd,
        cwd=NS3_DIR,
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        print("=== NS-3 STDOUT ===")
        print(result.stdout)
        print("=== NS-3 STDERR ===")
        print(result.stderr)
        raise RuntimeError("ns-3 실행 실패")

    rx_path = os.path.join(NS3_DIR, "ns3_rx_log.csv")
    rows = []
    if os.path.exists(rx_path):
        with open(rx_path, "r", newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    return rows


def parse_receivers_from_ns3(rx_rows):
    receivers = set()
    for row in rx_rows:
        if row["message"] == "Emergency Brake Warning":
            receivers.add(row["node"])
    return receivers


def apply_deceleration_step(veh_id: str, decel: float):
    """
    현재 속도에서 decel * STEP_LENGTH 만큼 감소
    """
    if veh_id not in traci.vehicle.getIDList():
        return None

    current_speed = traci.vehicle.getSpeed(veh_id)
    new_speed = max(0.0, current_speed - decel * STEP_LENGTH)

    traci.vehicle.setSpeedMode(veh_id, 0)
    traci.vehicle.setSpeed(veh_id, new_speed)
    return new_speed


# =========================
# 단일 시나리오 실행
# =========================
def run_single_spacing(spacing_m: float):
    print(f"\n==============================")
    print(f"SCENARIO spacing = {spacing_m} m")
    print(f"==============================")

    write_route_file(spacing_m)

    sumo_cmd = [
        "sumo-gui",
        "-c", SUMO_CFG,
        "--delay", "200",
        "--collision.action", "warn",
        "--collision.check-junctions", "true",
        "--step-length", str(STEP_LENGTH)
    ]
    traci.start(sumo_cmd)

    step = 0
    event_triggered = False
    collision_happened = False
    collision_pairs = set()
    collided_vehicles = set()
    post_collision_brake_active = False

    all_visible_reference_distance = None

    reaction_steps = {}
    brake_started = {f"car{i}": False for i in range(1, VEHICLE_COUNT)}

    lead_braking_active = False
    manual_control_active = False
    event_speed_snapshot = {}

    min_gap = {
        "01": float("inf"),
        "12": float("inf"),
        "23": float("inf"),
        "34": float("inf"),
    }

    event_gaps = {"01": None, "12": None, "23": None, "34": None}

    end_reason = None
    stop_confirmed_step = None
    event_step = None
    event_time = None

    step_log_rows = []
    receivers = set()

    while step < MAX_STEPS:
        traci.simulationStep()
        ids = set(traci.vehicle.getIDList())

        # 이미 충돌한 차량은 SUMO warn 모드에서 계속 지나가지 않도록 정지 상태로 고정
        for v_crashed in list(collided_vehicles):
            if v_crashed in ids:
                traci.vehicle.setSpeedMode(v_crashed, 0)
                traci.vehicle.setSpeed(v_crashed, 0.0)
                set_vehicle_color_if_exists(v_crashed, COLOR_COLLISION)

        # 기본 색상
        for v in VEH_IDS:
            if v in ids and v not in collided_vehicles:
                if v == "car0":
                    if not lead_braking_active:
                        set_vehicle_color_if_exists(v, COLOR_LEAD)
                elif not brake_started.get(v, False):
                    set_vehicle_color_if_exists(v, COLOR_FOLLOWER)

        visible = all_platoon_visible()
        pair_gaps = get_pair_gaps()
        pair_speeds = get_pair_speeds()

        # 최소 gap 기록
        if pair_gaps["car0_car1"] is not None:
            min_gap["01"] = min(min_gap["01"], pair_gaps["car0_car1"])
        if pair_gaps["car1_car2"] is not None:
            min_gap["12"] = min(min_gap["12"], pair_gaps["car1_car2"])
        if pair_gaps["car2_car3"] is not None:
            min_gap["23"] = min(min_gap["23"], pair_gaps["car2_car3"])
        if pair_gaps["car3_car4"] is not None:
            min_gap["34"] = min(min_gap["34"], pair_gaps["car3_car4"])

        # 5대 모두 보인 뒤 기준 거리 저장
        if visible and all_visible_reference_distance is None:
            all_visible_reference_distance = traci.vehicle.getDistance(LEAD_VEH)
            print(f"[VISIBLE] all 5 vehicles visible at step {step}, leader distance ref = {all_visible_reference_distance:.2f}")

        # 이벤트 발생
        if visible and not event_triggered and all_visible_reference_distance is not None:
            lead_dist = traci.vehicle.getDistance(LEAD_VEH)
            if (lead_dist - all_visible_reference_distance) >= EVENT_EXTRA_TRAVEL_AFTER_ALL_VISIBLE:
                event_triggered = True
                event_step = step
                event_time = step * STEP_LENGTH

                event_gaps["01"] = pair_gaps["car0_car1"]
                event_gaps["12"] = pair_gaps["car1_car2"]
                event_gaps["23"] = pair_gaps["car2_car3"]
                event_gaps["34"] = pair_gaps["car3_car4"]

                print(f"[EVENT] emergency braking on car0 at step {step}")
                print(f"[EVENT] event gaps = {event_gaps}")

                # 이벤트 순간부터 모든 차량 수동 제어
                for v in VEH_IDS:
                    if v in ids:
                        event_speed_snapshot[v] = traci.vehicle.getSpeed(v)
                        traci.vehicle.setSpeedMode(v, 0)
                        traci.vehicle.setSpeed(v, event_speed_snapshot[v])

                manual_control_active = True
                lead_braking_active = True
                set_vehicle_color_if_exists("car0", COLOR_BRAKING)

                rx_rows = run_ns3_normal(spacing_m)
                receivers = parse_receivers_from_ns3(rx_rows)

                print(f"[NS3] receivers = {sorted(receivers)}")

                for i in range(1, VEHICLE_COUNT):
                    veh = f"car{i}"
                    node = veh_to_node(veh)

                    if node in receivers:
                        reaction_delay_sec = FAST_REACTION_SEC
                    else:
                        reaction_delay_sec = SLOW_REACTION_SEC

                    reaction_steps[veh] = step + int(reaction_delay_sec / STEP_LENGTH)
                    print(f"[DECISION] {veh} -> reaction step {reaction_steps[veh]} ({reaction_delay_sec:.1f}s)")

        # 이벤트 후 follower는 반응 전까지 snapshot 속도 유지
        if event_triggered and manual_control_active:
            for i in range(1, VEHICLE_COUNT):
                veh = f"car{i}"
                if veh in ids and veh not in collided_vehicles and not brake_started[veh]:
                    keep_speed = event_speed_snapshot.get(veh, PLATOON_CRUISE_SPEED)
                    traci.vehicle.setSpeedMode(veh, 0)
                    traci.vehicle.setSpeed(veh, keep_speed)

        # 리더 제동
        if event_triggered and lead_braking_active and "car0" in ids and "car0" not in collided_vehicles:
            new_speed = apply_deceleration_step("car0", LEAD_BRAKE_DECEL)
            if new_speed is not None and new_speed <= 0.1:
                set_vehicle_color_if_exists("car0", COLOR_STOPPED)

        # follower 반응 시작
        if event_triggered:
            for i in range(1, VEHICLE_COUNT):
                veh = f"car{i}"
                if veh in ids and veh not in collided_vehicles and not brake_started[veh]:
                    if step >= reaction_steps.get(veh, 10**9):
                        brake_started[veh] = True
                        set_vehicle_color_if_exists(veh, COLOR_BRAKING)
                        print(f"[ACTION] {veh} braking starts at step {step}")

        # follower 제동
        if event_triggered:
            for i in range(1, VEHICLE_COUNT):
                veh = f"car{i}"
                if veh in ids and veh not in collided_vehicles and brake_started[veh]:
                    new_speed = apply_deceleration_step(veh, FOLLOWER_BRAKE_DECEL)
                    if new_speed is not None and new_speed <= 0.1:
                        set_vehicle_color_if_exists(veh, COLOR_STOPPED)

        # 충돌 이후 아직 충돌하지 않은 차량은 실제로 정지할 때까지 제동
        if event_triggered and post_collision_brake_active:
            for v_post in VEH_IDS:
                if v_post in ids:
                    if v_post in collided_vehicles:
                        traci.vehicle.setSpeedMode(v_post, 0)
                        traci.vehicle.setSpeed(v_post, 0.0)
                        set_vehicle_color_if_exists(v_post, COLOR_COLLISION)
                        continue

                    decel = LEAD_BRAKE_DECEL if v_post == LEAD_VEH else FOLLOWER_BRAKE_DECEL
                    new_speed = apply_deceleration_step(v_post, decel)

                    if new_speed is not None and new_speed <= 0.1:
                        set_vehicle_color_if_exists(v_post, COLOR_STOPPED)

        # 충돌 감지
        colliding = tuple(traci.simulation.getCollidingVehiclesIDList())
        if len(colliding) > 0:
            collision_happened = True

            for veh in colliding:
                set_vehicle_color_if_exists(veh, COLOR_COLLISION)

            newly_collided = [veh for veh in colliding if veh not in collided_vehicles]

            collision_pairs.update(colliding)
            collided_vehicles.update(colliding)

            print(f"[COLLISION] step={step}, vehicles={colliding}")

            for veh_crash in newly_collided:
                if veh_crash in ids:
                    traci.vehicle.setSpeedMode(veh_crash, 0)
                    traci.vehicle.setSpeed(veh_crash, 0.0)
                    set_vehicle_color_if_exists(veh_crash, COLOR_COLLISION)
                    print(f"[COLLISION-HOLD] {veh_crash} stopped and held after collision")

            if not post_collision_brake_active:
                post_collision_brake_active = True
                print(f"[COLLISION-BRAKE] collision detected; braking non-collided vehicles until full stop")

        # 모든 차량 정지 확인
        if visible and all_platoon_stopped() and stop_confirmed_step is None:
            stop_confirmed_step = step
            end_reason = "collision_then_all_stopped" if collision_happened else "safe_full_stop"
            print(f"[STOP] all vehicles stopped at step {step}, end at step {step + POST_STOP_HOLD_STEPS}")

            for veh in VEH_IDS:
                if veh in ids and veh not in collision_pairs:
                    set_vehicle_color_if_exists(veh, COLOR_STOPPED)

        # step log
        step_log_rows.append({
            "step": step,
            "sim_time": step * STEP_LENGTH,
            "all_visible": visible,
            "event_triggered": event_triggered,
            "collision_happened": collision_happened,
            "all_stopped": visible and all_platoon_stopped(),
            "car0_speed": pair_speeds["car0"],
            "car1_speed": pair_speeds["car1"],
            "car2_speed": pair_speeds["car2"],
            "car3_speed": pair_speeds["car3"],
            "car4_speed": pair_speeds["car4"],
            "gap_01": pair_gaps["car0_car1"],
            "gap_12": pair_gaps["car1_car2"],
            "gap_23": pair_gaps["car2_car3"],
            "gap_34": pair_gaps["car3_car4"],
        })

        if stop_confirmed_step is not None and step >= stop_confirmed_step + POST_STOP_HOLD_STEPS:
            print(f"[END] scenario {spacing_m}m finished")
            break

        step += 1

    final_gaps = get_pair_gaps()
    traci.close()

    min_gap_clean = {
        k: (None if v == float("inf") else v)
        for k, v in min_gap.items()
    }

    final_gap_clean = {
        "01": final_gaps["car0_car1"],
        "12": final_gaps["car1_car2"],
        "23": final_gaps["car2_car3"],
        "34": final_gaps["car3_car4"],
    }

    valid_final = [g for g in final_gap_clean.values() if g is not None]
    min_final_gap = min(valid_final) if valid_final else None

    observed_min_gap = [v for v in min_gap_clean.values() if v is not None]
    overall_min_gap = min(observed_min_gap) if observed_min_gap else None
    extra_gap_needed_est = estimate_extra_gap_needed(collision_happened, overall_min_gap)

    reaction_map_str = []
    for i in range(1, VEHICLE_COUNT):
        veh = f"car{i}"
        if veh in reaction_steps:
            reaction_map_str.append(f"{veh}:{reaction_steps[veh]}")
    reaction_map_str = "|".join(reaction_map_str)

    summary_row = {
        "scenario_spacing_m": spacing_m,
        "event_step": event_step if event_triggered else None,
        "event_time_s": event_time if event_triggered else None,
        "event_gap_01_m": event_gaps["01"],
        "event_gap_12_m": event_gaps["12"],
        "event_gap_23_m": event_gaps["23"],
        "event_gap_34_m": event_gaps["34"],
        "collision": collision_happened,
        "collision_pairs": "|".join(sorted(collision_pairs)) if collision_pairs else "",
        "success_no_collision": (not collision_happened and end_reason == "safe_full_stop"),
        "min_gap_01_m": min_gap_clean["01"],
        "min_gap_12_m": min_gap_clean["12"],
        "min_gap_23_m": min_gap_clean["23"],
        "min_gap_34_m": min_gap_clean["34"],
        "final_gap_01_m": final_gap_clean["01"],
        "final_gap_12_m": final_gap_clean["12"],
        "final_gap_23_m": final_gap_clean["23"],
        "final_gap_34_m": final_gap_clean["34"],
        "min_final_gap_m": min_final_gap,
        "extra_gap_needed_est_m": extra_gap_needed_est,
        "receivers": "|".join(sorted(receivers)),
        "reaction_map": reaction_map_str,
        "end_reason": end_reason
    }

    step_log_path = os.path.join(LOG_DIR, f"spacing_{spacing_m}_step_log.csv")
    save_step_log(step_log_path, step_log_rows)

    return summary_row


# =========================
# 전체 실행
# =========================
def main():
    if os.path.exists(SUMMARY_FILE):
        os.remove(SUMMARY_FILE)

    first = True
    for spacing in SPACING_CASES:
        result = run_single_spacing(spacing)
        print(result)
        append_summary(result, write_header=first)
        first = False

    print("\nDONE: baseline_summary.csv and baseline_logs/*.csv generated")


if __name__ == "__main__":
    main()