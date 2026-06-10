import os
import csv
import subprocess
import traci

# HYBRID SELECTIVE FAILURE POLICY V4
# - normal/wired 거리 미수신: 0.5s slow reaction 유지
# - wired path failure: DetNet/Full만 복구, 그 외 자동 제동 없음
# - wireless failure: URLLC 계열 수신 개선, 미수신은 자동 제동 없음
# - combined failure: DetNet + URLLC가 함께 있을 때 안정성 확보

print("### HYBRID FULL CODE RUNNING: TSN + DETNET + URLLC ###")

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
RESULT_DIR = os.path.join(RESULT_ROOT, "hybrid", "normal", "full")

LOG_DIR = os.path.join(RESULT_DIR, "hy_full_logs")
SUMMARY_FILE = os.path.join(RESULT_DIR, "hy_full_summary.csv")

os.makedirs(LOG_DIR, exist_ok=True)

# =========================
# 실험 설정
# =========================
SPACING_CASES = [15, 17, 19, 21, 23, 25]   # m
VEHICLE_COUNT = 5
VEH_IDS = [f"car{i}" for i in range(VEHICLE_COUNT)]

STEP_LENGTH = 0.1
MAX_STEPS = 3000

# 모든 차량이 정지한 뒤 1초 후 종료
POST_STOP_HOLD_SEC = 1.0
POST_STOP_HOLD_STEPS = int(POST_STOP_HOLD_SEC / STEP_LENGTH)

# 패킷/경로 실패 차량은 자동 제동하지 않으므로, 시나리오 종료를 위한 관찰 제한
EVENT_TIMEOUT_SEC = 12.0
EVENT_TIMEOUT_STEPS = int(EVENT_TIMEOUT_SEC / STEP_LENGTH)

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
# OMNeT++ wired baseline delay
# =========================

OMNET_DIR = os.path.expanduser("~/omnetpp-workspace/hybrid")

OMNET_INI = os.path.join(OMNET_DIR, "omnetpp_tsn_detnet.ini")
OMNET_VEC = os.path.join(OMNET_DIR, "results", "tsn_detnet_baseline.vec")
OMNET_SCA = os.path.join(OMNET_DIR, "results", "tsn_detnet_baseline.sca")
WIRED_DELAY_CSV = os.path.join(OMNET_DIR, "results", "tsn_detnet_delay_emergency.csv")

# =========================
# OMNeT++ DetNet delivery result
# =========================
DETNET_RUNTIME_RESULT_DIR = os.path.join(
    OMNET_DIR,
    "results",
    "hybrid_detnet_delivery_runtime"
)

DETNET_SCALAR_CSV = os.path.join(
    OMNET_DIR,
    "results",
    "hybrid_detnet_delivery_scalars.csv"
)

DETNET_VECTOR_CSV = os.path.join(
    OMNET_DIR,
    "results",
    "hybrid_detnet_delivery_vectors.csv"
)

DETNET_DELIVERY_CSV = os.path.join(
    OMNET_DIR,
    "results",
    "detnet_delivery.csv"
)

OPP_RUN = os.path.expanduser("~/omnetpp-6.3.0/bin/opp_run")
OPP_SCAVETOOL = os.path.expanduser("~/omnetpp-6.3.0/bin/opp_scavetool")
INET_SRC = os.path.expanduser("~/inet4.6/src")
INET_LIB = os.path.join(INET_SRC, "INET")

DEFAULT_WIRED_DELAY_SEC = 0.000067549

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
    leader_pos = 120.0  # 충분히 큰 값

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
        f"scratch/v2v-wireless-urllc --numNodes=5 --distance={spacing_m} --simTime=3"
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

def run_omnet_detnet_delivery_sim():
    """
    Emergency event 발생 시 OMNeT++/INET DetNet 시뮬레이션을 실행한다.

    v5 Step 1 목적:
    - Python에서 OMNeT++ DetNet 시뮬레이션 호출 확인
    - .sca / .vec 결과 생성 위치 확인
    - scalar/vector CSV export 확인

    주의:
    - 현 단계에서는 차량별 path A/B 수신 여부를 아직 Python 판단에 직접 사용하지 않는다.
    - 다음 단계에서 OMNeT++가 detnet_delivery.csv를 생성하면 Python의 경로 복구 판단을 그 파일 기반으로 교체한다.
    """

    print("[OMNET-DETNET] running DetNet delivery simulation...")

    import glob
    import shutil

    results_dir = os.path.join(OMNET_DIR, "results")
    os.makedirs(results_dir, exist_ok=True)

    if os.path.exists(DETNET_RUNTIME_RESULT_DIR):
        shutil.rmtree(DETNET_RUNTIME_RESULT_DIR)
    os.makedirs(DETNET_RUNTIME_RESULT_DIR, exist_ok=True)

    # 이전 export 및 기존 고정 결과 파일 제거
    for path in [
        DETNET_SCALAR_CSV,
        DETNET_VECTOR_CSV,
        DETNET_DELIVERY_CSV,
        OMNET_SCA,
        OMNET_VEC,
        OMNET_VEC + "i",
        OMNET_VEC + ".vci",
    ]:
        if os.path.exists(path):
            os.remove(path)

    env = os.environ.copy()
    env["PATH"] = f"{os.path.expanduser('~/omnetpp-6.3.0/bin')}:{env.get('PATH', '')}"
    env["LD_LIBRARY_PATH"] = f"{os.path.expanduser('~/omnetpp-6.3.0/lib')}:{env.get('LD_LIBRARY_PATH', '')}"

    run_cmd = [
        OPP_RUN,
        "-u", "Cmdenv",
        "-f", OMNET_INI,
        "-n", f".:{INET_SRC}",
        "-l", INET_LIB,
        f"--result-dir={DETNET_RUNTIME_RESULT_DIR}",
    ]

    print("[OMNET-DETNET] command:")
    print(" ".join(run_cmd))

    result = subprocess.run(
        run_cmd,
        cwd=OMNET_DIR,
        capture_output=True,
        text=True,
        env=env
    )

    print("=== OMNET DETNET STDOUT TAIL ===")
    print("\n".join(result.stdout.splitlines()[-30:]))
    print("=== OMNET DETNET STDERR TAIL ===")
    print("\n".join(result.stderr.splitlines()[-30:]))

    if result.returncode != 0:
        raise RuntimeError("OMNeT++ DetNet delivery simulation failed")

    # 1) --result-dir 내부 결과 확인
    sca_files = glob.glob(os.path.join(DETNET_RUNTIME_RESULT_DIR, "*.sca"))
    vec_files = glob.glob(os.path.join(DETNET_RUNTIME_RESULT_DIR, "*.vec"))

    # 2) omnetpp.ini가 고정 파일명으로 쓴 경우 확인
    if os.path.exists(OMNET_SCA):
        sca_files.append(OMNET_SCA)
    if os.path.exists(OMNET_VEC):
        vec_files.append(OMNET_VEC)

    # 3) results 폴더 전체에서 최신 파일 fallback 검색
    if not sca_files:
        sca_files = glob.glob(os.path.join(results_dir, "*.sca"))
    if not vec_files:
        vec_files = glob.glob(os.path.join(results_dir, "*.vec"))

    sca_files = sorted(set(sca_files), key=os.path.getmtime, reverse=True)
    vec_files = sorted(set(vec_files), key=os.path.getmtime, reverse=True)

    print(f"[OMNET-DETNET] found .sca files: {sca_files}")
    print(f"[OMNET-DETNET] found .vec files: {vec_files}")

    if not sca_files:
        print("[OMNET-DETNET] result directory listing:")
        for root, dirs, names in os.walk(results_dir):
            depth = root.replace(results_dir, "").count(os.sep)
            if depth > 2:
                continue
            print(root)
            for name in names[:20]:
                print("  ", name)

        raise RuntimeError(
            "OMNeT++ DetNet simulation finished, but no .sca file was found "
            "in runtime result dir or hybrid/results."
        )

    # scalar export
    scalar_export_cmd = [
        OPP_SCAVETOOL,
        "export",
        "--type", "s",
        "-F", "CSV-R",
        "-o", DETNET_SCALAR_CSV,
    ] + sca_files

    result = subprocess.run(
        scalar_export_cmd,
        cwd=OMNET_DIR,
        capture_output=True,
        text=True,
        env=env
    )

    if result.returncode != 0:
        print("=== OMNET SCALAR EXPORT STDOUT ===")
        print(result.stdout)
        print("=== OMNET SCALAR EXPORT STDERR ===")
        print(result.stderr)
        raise RuntimeError("OMNeT++ DetNet scalar export failed")

    print(f"[OMNET-DETNET] scalar CSV exported: {DETNET_SCALAR_CSV}")

    # vector export
    if vec_files:
        vector_export_cmd = [
            OPP_SCAVETOOL,
            "export",
            "--type", "v",
            "-F", "CSV-R",
            "-o", DETNET_VECTOR_CSV,
        ] + vec_files

        result = subprocess.run(
            vector_export_cmd,
            cwd=OMNET_DIR,
            capture_output=True,
            text=True,
            env=env
        )

        if result.returncode != 0:
            print("=== OMNET VECTOR EXPORT STDOUT ===")
            print(result.stdout)
            print("=== OMNET VECTOR EXPORT STDERR ===")
            print(result.stderr)
            raise RuntimeError("OMNeT++ DetNet vector export failed")

        print(f"[OMNET-DETNET] vector CSV exported: {DETNET_VECTOR_CSV}")
    else:
        print("[OMNET-DETNET] no vector file found; scalar export only")

    return {
        "sca_files": sca_files,
        "vec_files": vec_files,
        "scalar_csv": DETNET_SCALAR_CSV,
        "vector_csv": DETNET_VECTOR_CSV if vec_files else None,
        "delivery_csv": DETNET_DELIVERY_CSV,
    }

def run_omnet_tsn_backbone():
    """
    이벤트 발생 시점에 OMNeT++ TSN wired backbone을 실행하고,
    endToEndDelay vector를 tsn_delay.csv로 export한다.
    """

    print("[OMNET] running TSN wired backbone simulation...")

    os.makedirs(os.path.join(OMNET_DIR, "results"), exist_ok=True)

    for path in [OMNET_VEC, OMNET_SCA, WIRED_DELAY_CSV]:
        if os.path.exists(path):
            os.remove(path)

    run_cmd = [
        OPP_RUN,
        "-u", "Cmdenv",
        "-f", OMNET_INI,
        "-n", f".:{INET_SRC}",
        "-l", INET_LIB
    ]

    result = subprocess.run(
        run_cmd,
        cwd=OMNET_DIR,
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        print("=== OMNET STDOUT ===")
        print(result.stdout)
        print("=== OMNET STDERR ===")
        print(result.stderr)
        raise RuntimeError("OMNeT++ TSN backbone simulation failed")

    export_cmd = [
        OPP_SCAVETOOL,
        "export",
        "--type", "v",
        "-f", 'module =~ "*rsu1.app[0]" AND name =~ "*endToEndDelay*"',
        "-F", "CSV-R",
        "-o", WIRED_DELAY_CSV,
        OMNET_VEC
    ]

    result = subprocess.run(
        export_cmd,
        cwd=OMNET_DIR,
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        print("=== SCAVETOOL STDOUT ===")
        print(result.stdout)
        print("=== SCAVETOOL STDERR ===")
        print(result.stderr)
        raise RuntimeError("OMNeT++ TSN delay export failed")

    print("[OMNET] TSN delay exported:", WIRED_DELAY_CSV)

def load_wired_delay_from_omnet(csv_path: str):
    """
    OMNeT++ opp_scavetool로 export한 wired_delay.csv에서
    endToEndDelay:vector의 vecvalue 값을 읽어 평균 wired delay를 반환한다.
    """
    if not os.path.exists(csv_path):
        print(f"[OMNET] wired delay csv not found: {csv_path}")
        print(f"[OMNET] using default wired delay = {DEFAULT_WIRED_DELAY_SEC:.8f}s")
        return DEFAULT_WIRED_DELAY_SEC

    delays = []

    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            if row.get("type") == "vector" and "endToEndDelay" in row.get("name", ""):
                vecvalue = row.get("vecvalue", "").strip()

                if not vecvalue:
                    continue

                # CSV-R 형식에서는 vecvalue가 "6.398e-05 4.074e-05"처럼 들어감
                for value in vecvalue.replace('"', "").split():
                    try:
                        delays.append(float(value))
                    except ValueError:
                        pass

    if not delays:
        print(f"[OMNET] no wired delay samples found in {csv_path}")
        print(f"[OMNET] using default wired delay = {DEFAULT_WIRED_DELAY_SEC:.8f}s")
        return DEFAULT_WIRED_DELAY_SEC

    avg_delay = sum(delays) / len(delays)

    print(
        f"[OMNET] loaded wired baseline delay: "
        f"avg={avg_delay:.8f}s, samples={len(delays)}"
    )

    return avg_delay

def parse_receivers_from_ns3(rx_rows):
    receivers = set()

    sender_node = veh_to_node(LEAD_VEH)

    for row in rx_rows:
        if row.get("message") == "Emergency Brake Warning":
            node = row.get("node")

            if node == sender_node:
                continue

            receivers.add(node)

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
        "--time-to-teleport", "-1",
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
    reaction_status = {}
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
                detnet_omnet_result = run_omnet_detnet_delivery_sim()

                run_omnet_tsn_backbone()
                wired_delay_sec = load_wired_delay_from_omnet(WIRED_DELAY_CSV)

                print(f"[NS3] receivers = {sorted(receivers)}")

                for i in range(1, VEHICLE_COUNT):
                    veh = f"car{i}"
                    node = veh_to_node(veh)

                    if node in receivers:
                        reaction_delay_sec = FAST_REACTION_SEC + wired_delay_sec
                        reaction_steps[veh] = step + max(0, int(reaction_delay_sec / STEP_LENGTH))
                        reaction_status[veh] = "received"
                        print(
                            f"[HYBRID] {veh} received warning, "
                            f"reaction step {reaction_steps[veh]} ({reaction_delay_sec:.8f}s)"
                        )
                    else:
                        # Normal 조건에서는 수신 범위/거리 문제로 직접 수신하지 못한 차량도
                        # failure로 보지 않고 기존 모델처럼 0.5초 뒤 자연 반응한다.
                        reaction_delay_sec = SLOW_REACTION_SEC
                        reaction_steps[veh] = step + max(0, int(reaction_delay_sec / STEP_LENGTH))
                        reaction_status[veh] = "slow_reaction_distance"
                        print(f"[DECISION] {veh} -> slow reaction step {reaction_steps[veh]} ({reaction_delay_sec:.1f}s)")
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

        # 패킷/경로 실패 차량이 자동 제동하지 않으면 모든 차량 정지 조건이 안 걸릴 수 있음
        if event_triggered and event_step is not None and step >= event_step + EVENT_TIMEOUT_STEPS:
            if end_reason is None:
                end_reason = "event_timeout_no_auto_brake_collision" if collision_happened else "event_timeout_no_auto_brake"
                print(f"[TIMEOUT] scenario {spacing_m}m ended by observation timeout: {end_reason}")
            break

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
        else:
            reaction_map_str.append(f"{veh}:{reaction_status.get(veh, 'not_decided')}")
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