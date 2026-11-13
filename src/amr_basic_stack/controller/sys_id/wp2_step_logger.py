#!/usr/bin/env python3
# WP2 — 속도루프 스텝응답 로거 (ROS2 Jazzy / rclpy)   [담당: 준민·민철]
# ---------------------------------------------------------------
# 실기 확인된 인터페이스에 맞춤 (2026-08 브링업 점검 반영):
#   명령   : /cmd_vel  (geometry_msgs/Twist)  <- 몸체 twist [vx,vy,w]
#   피드백1: /odom         (nav_msgs/Odometry)      -> 몸체속도 실측
#   피드백2: /joint_states (sensor_msgs/JointState) -> 바퀴속도 실측(4개)
#
# 하는 일: 몸체 축(vx,vy,w)을 하나씩 스텝 여자하며 명령/실측을 CSV로 기록.
#   -> 축레벨 MPC(WP4)의 시스템식별(WP3) 입력.
#
# ★ 선행조건(로그 찍기 전 반드시):
#   (1) base.yaml qpps_per_rad_per_sec 실측 보정 완료 (아니면 속도 스케일 전부 틀림)
#   (2) /joint_states 가 실측인지 확인: 바퀴 손으로 돌려 echo 로 값 변하는지
#
# 사용: ROS2 소싱 후  python3 wp2_step_logger.py [옵션]
#   --amps-lin 0.3,0.45,0.6   vx,vy 계단 진폭 [m/s]        (기본 0.1,0.2,0.3)
#   --amps-ang 1.2,1.8,2.4    w 계단 진폭 [rad/s]           (기본 0.3,0.6,1.2)
#   --ramp 0.2                계단 대신 램프 상승 [s]. 0 = 계단. 고진폭에서 PI 포화 회피
#   --repeats 3  --t-step 2.0  --t-rest 1.0  --axes 0,1,2
#   --wheel-max 8.0           드라이버 max_wheel_rad_per_sec (base.yaml). 진폭이 이걸 넘으면
#                             드라이버가 자르므로 시작 전에 거부한다. 고진폭 실험 전 base.yaml 을 먼저 올릴 것
#   예) 0.6 m/s 까지:  base.yaml max_wheel_rad_per_sec: 16  후
#       python3 wp2_step_logger.py --amps-lin 0.3,0.45,0.6 --amps-ang 1.2,1.8,2.4 --ramp 0.2 --wheel-max 16
# CSV열: t_s, active, seg, amp, cvx,cvy,cw, ovx,ovy,ow, w0,w1,w2,w3, vbat_L,vbat_R,
#        pwm0..pwm3, cur0..cur3, t_odom, t_js
#   seg: 스케줄 세그먼트 번호 (정지·구동 각각 하나). amp: 그 세그먼트의 목표 진폭 (정지면 0).
#        fit_fopdt.py 는 열 이름으로 읽으므로 추가 열은 무해하다.
# 종료 시 진폭별로 '도달한 바퀴속도 / 기대값' 표를 찍는다. 큰 진폭에서만 비율이 떨어지면
# 드라이버 클립 또는 모터 한계다 — 그 진폭은 피팅에서 빼야 한다.
#   active: 0=vx,1=vy,2=w / c*=명령twist / o*=odom실측 / w*=바퀴속도
#   vbat_*: RoboClaw 좌/우 보드 메인전압 [V]. 1Hz 라 50Hz 행에는 마지막 값이 반복된다.
#           실행 중 전압이 떨어지면 응답도 느려지므로, 두 로그가 안 맞을 때 1순위 용의자.
#   pwm*  : 모터 PWM 듀티 [%]. 100 에 붙으면 포화다. 포화 구간은 1차 모델 피팅에서 빼야
#           한다 (tau 가 실제보다 느리게 나온다). 저역 발행이라 마지막 값이 반복된다.
#   cur*  : 모터 전류 [A]. 순서는 w0~w3 과 같은 FL,FR,RL,RR.
#   t_*   : 각 측정의 헤더 스탬프 [s]. 기록 시각(t_s)이 아니라 '그 값이 실제로 찍힌 시각'.
#           t_s 만 쓰면 마지막 수신 메시지가 최대 한 주기(20ms) 묵어 theta 가 그만큼
#           부풀고, Ts=0.05 에서 d=round(theta/Ts) 의 경계가 25ms 라 d 가 통째로 뒤집힌다.
#
# ★ 바퀴를 띄우고 찍은 로그에서는 ovx/ovy/ow 가 가짜다. /odom 은 바퀴속도로
#   계산되는 값이라 로봇이 제자리여도 움직인 것처럼 나온다. 공중 로그에서
#   유효한 실측은 w0~w3 뿐이다.
# ---------------------------------------------------------------
import argparse, csv, sys, time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

# ====================== CONFIG ======================
RATE_HZ    = 50                 # /odom·/joint_states 가 50Hz 이므로 맞춤 (= WP3의 Ts=0.02)
                                # 드라이버 update_rate_hz 100 이면 --rate 100 (2026-10-22, τ 15~40 ms 해상도)
CSV_PREFIX = "wp2_log"          # 실제 파일: wp2_log_20260820_143052.csv (실행시각)
                                # 땅/공중 로그가 서로 덮어쓰지 않게 매 실행 새 파일
CMD_TOPIC  = "/cmd_vel"
ODOM_TOPIC = "/odom"
JS_TOPIC   = "/joint_states"
BATT_TOPIC = "/roboclaw/battery_voltages"   # 1Hz, [좌보드, 우보드] 전압
DIAG_TOPIC = "/roboclaw/motor_diagnostics"  # 저역, [pwm×4, 전류×4] FL,FR,RL,RR

# 진폭은 액추에이터 한계 대비로 잡는다. base.yaml 기준
#   V_LIMIT = r·w_max = 0.04×8.0 = 0.32 m/s,  W_LIMIT = V_LIMIT/k = 1.61 rad/s  (k=0.19835)
AMP_LIN    = [0.1, 0.2, 0.3]    # vx,vy [m/s]   한계의 31 / 62 / 94%
AMP_ANG    = [0.3, 0.6, 1.2]    # w    [rad/s]  한계의 19 / 37 / 74%
                                # 0.9(56%) 는 직진(94%)과 커버리지가 안 맞아 1.2 로 올렸다.
                                # 스텝 수·소요시간은 그대로다.
DIRECTIONS = [+1, -1]
REPEATS    = 3
T_REST     = 1.0                # 스텝 전 정지 [s]
T_STEP     = 2.0                # 스텝 유지 [s]
AXES       = [0, 1, 2]          # 0=vx,1=vy,2=w  (필요시 [0] 만 등으로 축소)
RAMP_S     = 0.0                # 계단 상승 시간 [s]. 0 = 순수 계단
WHEEL_MAX  = 8.0                # 드라이버 max_wheel_rad_per_sec (base.yaml). 진폭 검사용
R_WHEEL    = 0.04
K_GEOM     = (0.1897 + 0.207) / 2.0

# 이상 감지 자동 정지: 명령을 냈는데 바퀴가 안 도는 상태가 이만큼 지속되면 실험을 끊는다.
# rest 1s + step 2s 보다 커야 정상 동작을 오탐하지 않는다.
DEAD_ABORT_S = 4.0
MIN_VBAT     = 9.0              # 중단 시 이 아래면 모터 전원 미인가로 지목 (공칭 12~24V)
# ====================================================
_DEFAULTS = dict(AMP_LIN=AMP_LIN, AMP_ANG=AMP_ANG, RAMP_S=RAMP_S, REPEATS=REPEATS,
                 T_STEP=T_STEP, T_REST=T_REST, AXES=AXES, WHEEL_MAX=WHEEL_MAX)


def build_schedule():
    seg = []
    for ax in AXES:
        amps = AMP_ANG if ax == 2 else AMP_LIN
        for amp in amps:
            # 방향을 매 반복 교대시킨다. +,+,+,-,-,- 로 몰면 한쪽으로 1.8 m
            # 밀려나서 넓은 자리가 필요하다. +,-,+,-,+,- 면 스텝마다 되돌아온다.
            for _ in range(REPEATS):
                for sgn in DIRECTIONS:
                    seg.append((ax, 0.0,       T_REST))
                    seg.append((ax, sgn * amp, T_STEP))
    return seg


def wheel_expected(ax, amp):
    """진폭 → 가장 빠른 바퀴의 각속도 [rad/s]. 메카넘 역기구학 (roboclaw_mecanum_node 와 동일)."""
    return abs(amp) / R_WHEEL if ax < 2 else abs(amp) * K_GEOM / R_WHEEL


def check_amplitudes():
    """드라이버 한계를 넘는 진폭은 시작 전에 거부한다 — 잘린 계단은 식별에 못 쓴다."""
    bad = [(ax, a) for ax in AXES for a in (AMP_ANG if ax == 2 else AMP_LIN)
           if wheel_expected(ax, a) > WHEEL_MAX + 1e-9]
    if bad:
        for ax, a in bad:
            print(f"[ERROR] 축 {ax} 진폭 {a} → 바퀴 {wheel_expected(ax, a):.1f} rad/s > 드라이버 한계 {WHEEL_MAX}")
        print("        base.yaml max_wheel_rad_per_sec 를 먼저 올리고 --wheel-max 로 같은 값을 주세요.")
        sys.exit(2)


class StepLogger(Node):
    def __init__(self):
        super().__init__("wp2_step_logger")
        self.odom = [0.0, 0.0, 0.0]           # [vx,vy,w] 실측
        self.wheels = [0.0, 0.0, 0.0, 0.0]    # 바퀴 속도 실측
        self.batt = [float('nan'), float('nan')]  # [좌,우] 보드 전압. 1Hz 라 마지막 값 유지
        self.pwm = [float('nan')] * 4         # 모터 PWM [%]  저역이라 마지막 값 유지
        self.cur = [float('nan')] * 4         # 모터 전류 [A]
        self.t_odom = float('nan')            # 각 측정의 헤더 스탬프 [s]
        self.t_js = float('nan')
        self.first_cmd_t = None               # 0 아닌 명령을 처음 낸 시각
        self.last_motion_t = None             # 바퀴가 마지막으로 0 이 아니었던 시각
        self.rows = []
        self.reach = {}                       # (축, 진폭) → 정착 구간 최대 바퀴속도들
        self.schedule = build_schedule()
        self.seg_idx = 0
        self.seg_t0 = None
        self.t_start = None
        self.done = False
        # 파일명 시각은 '실행 시작' 기준으로 고정한다 (종료시각으로 하면 중단 시 헷갈린다)
        self.csv_path = f"{CSV_PREFIX}_{time.strftime('%Y%m%d_%H%M%S')}.csv"

        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.cb_odom, 50)
        self.create_subscription(JointState, JS_TOPIC, self.cb_js, 50)
        self.create_subscription(Float32MultiArray, BATT_TOPIC, self.cb_batt, 10)
        self.create_subscription(Float32MultiArray, DIAG_TOPIC, self.cb_diag, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.on_tick)

        total = sum(s[2] for s in self.schedule)
        self.get_logger().info(f"WP2 로깅 시작: {len(self.schedule)}세그먼트 ~{total:.0f}s"
                               f" -> {self.csv_path}")
        self.get_logger().info(f"진폭 lin={AMP_LIN} ang={AMP_ANG}  램프 {RAMP_S}s  "
                               f"바퀴 최대 기대 {max(wheel_expected(ax, a) for ax in AXES for a in (AMP_ANG if ax == 2 else AMP_LIN)):.1f}"
                               f" / 한계 {WHEEL_MAX} rad/s")
        self.get_logger().warn("선행조건 확인: qpps 보정 완료? /joint_states 실측 확인?")

    @staticmethod
    def stamp_sec(header):
        """헤더 스탬프를 초 단위로. 안 채워졌으면 nan."""
        t = header.stamp.sec + header.stamp.nanosec * 1e-9
        return t if t > 0.0 else float('nan')

    def clock_sec(self):
        """메시지 스탬프와 같은 기준의 시각. time.time() 을 쓰면 기준이 어긋난다."""
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_odom(self, m):
        t = m.twist.twist
        self.odom = [t.linear.x, t.linear.y, t.angular.z]
        self.t_odom = self.stamp_sec(m.header)

    def cb_js(self, m):
        v = list(m.velocity)[:4]
        if v:
            self.wheels = (v + [0, 0, 0, 0])[:4]
        self.t_js = self.stamp_sec(m.header)

    def cb_batt(self, m):
        v = list(m.data)[:2]
        if v:
            self.batt = (v + [float('nan'), float('nan')])[:2]

    def cb_diag(self, m):
        d = list(m.data)[:8]                   # [pwm×4, 전류×4]
        if len(d) == 8:
            self.pwm, self.cur = d[:4], d[4:]

    def on_tick(self):
        now = self.clock_sec()
        if self.t_start is None:
            self.t_start = now; self.seg_t0 = now

        if self.seg_idx >= len(self.schedule):
            self.finish(); return

        ax, val, dur = self.schedule[self.seg_idx]
        if now - self.seg_t0 >= dur:
            self.seg_idx += 1; self.seg_t0 = now
            if self.seg_idx >= len(self.schedule):
                self.finish(); return
            ax, val, dur = self.schedule[self.seg_idx]

        cmd = [0.0, 0.0, 0.0]
        el = now - self.seg_t0
        cmd[ax] = val * min(1.0, el / RAMP_S) if (RAMP_S > 0 and val != 0.0) else val
        self.publish(cmd)
        # 세그먼트 마지막 0.5 s 의 최대 바퀴속도를 모아 진폭별 도달률을 낸다 (클립·모터 한계 감지)
        if val != 0.0 and dur - el <= 0.5:
            self.reach.setdefault((ax, abs(val)), []).append(max(abs(w) for w in self.wheels))

        # 이상 감지: 명령을 냈는데 바퀴가 계속 정확히 0 이면 식별에 못 쓸 로그다.
        # 완주하고 나서 알아채는 대신(2026-08-18: 163초를 통째로 날림) 여기서 끊는다.
        if val != 0.0 and self.first_cmd_t is None:
            self.first_cmd_t = now
        if any(w != 0.0 for w in self.wheels):
            self.last_motion_t = now
        if self.first_cmd_t is not None:
            dead = now - max(self.first_cmd_t, self.last_motion_t or self.first_cmd_t)
            if dead > DEAD_ABORT_S:
                self.abort(dead); return

        rel = lambda t: round(t - self.t_start, 4) if t == t else float('nan')
        self.rows.append([round(now - self.t_start, 4), ax, self.seg_idx, abs(val)] + cmd
                         + list(self.odom) + list(self.wheels) + list(self.batt)
                         + list(self.pwm) + list(self.cur)
                         + [rel(self.t_odom), rel(self.t_js)])

    def publish(self, cmd):
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = cmd
        self.pub.publish(tw)

    def abort(self, dead_s):
        """명령을 냈는데 바퀴가 안 돈다. 식별에 못 쓸 로그이므로 CSV 를 남기지 않는다."""
        if self.done:
            return
        self.done = True
        try:
            self.publish([0.0, 0.0, 0.0])
        except Exception as e:
            print(f"[WARN] 정지명령 발행 실패: {e}")
        lo = min((v for v in self.batt if v == v), default=float('nan'))
        why = f"메인전압 {lo:.1f}V — 모터 전원 미인가 의심" if lo == lo and lo < MIN_VBAT \
              else f"메인전압 {lo:.1f}V" if lo == lo else "전압 미수신"
        print(f"[ERROR] 중단: 명령 후 {dead_s:.1f}초간 네 바퀴 속도가 전부 0. {why}")
        print("        모터 전원 / 엔코더 배선 / 시리얼 포트를 확인하세요. CSV 는 남기지 않습니다.")
        try: self.timer.cancel()
        except Exception: pass
        if rclpy.ok():
            rclpy.shutdown()

    def finish(self):
        if self.done:                          # on_tick 과 Ctrl+C 양쪽에서 불릴 수 있다
            return
        self.done = True
        try:
            self.publish([0.0, 0.0, 0.0])      # 안전 정지
        except Exception as e:
            # context 가 이미 죽었으면 여기서 터진다. 로그 저장까지 같이 죽이면 안 된다.
            print(f"[WARN] 정지명령 발행 실패: {e}")
        with open(self.csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "active", "seg", "amp", "cvx", "cvy", "cw",
                        "ovx", "ovy", "ow", "w0", "w1", "w2", "w3",
                        "vbat_L", "vbat_R",
                        "pwm0", "pwm1", "pwm2", "pwm3",
                        "cur0", "cur1", "cur2", "cur3",
                        "t_odom", "t_js"])
            w.writerows(self.rows)
        print(f"[INFO] 완료: {len(self.rows)}행 -> {self.csv_path}")
        if self.reach:
            print(f"[INFO] 진폭별 도달률 (정착 0.5 s, 최대 바퀴속도 / 기대값)")
            print(f"       {'축':>3}{'진폭':>7}{'기대 rad/s':>11}{'도달 rad/s':>11}{'비율':>7}")
            for (ax, amp), ws in sorted(self.reach.items()):
                exp, got = wheel_expected(ax, amp), max(ws)
                flag = "  ← 클립/한계 의심, 피팅에서 제외" if got < 0.85 * exp else ""
                print(f"       {ax:>3}{amp:>7.2f}{exp:>11.2f}{got:>11.2f}{got/exp:>7.2f}{flag}")
        try: self.timer.cancel()
        except Exception: pass
        if rclpy.ok():
            rclpy.shutdown()


def parse_args():
    global AMP_LIN, AMP_ANG, RAMP_S, REPEATS, T_STEP, T_REST, AXES, WHEEL_MAX, RATE_HZ
    D = _DEFAULTS                          # CONFIG 상수. 전역이 바뀌어도 기본값은 고정
    ap = argparse.ArgumentParser(description="WP2 구동계 계단응답 로거")
    f = lambda s: [float(x) for x in s.split(",") if x]
    ap.add_argument("--amps-lin", type=f, default=D["AMP_LIN"], help="vx,vy 진폭 [m/s], 쉼표 구분")
    ap.add_argument("--amps-ang", type=f, default=D["AMP_ANG"], help="w 진폭 [rad/s]")
    ap.add_argument("--ramp", type=float, default=D["RAMP_S"], help="상승 시간 [s]. 0 = 계단")
    ap.add_argument("--repeats", type=int, default=D["REPEATS"])
    ap.add_argument("--t-step", type=float, default=D["T_STEP"])
    ap.add_argument("--t-rest", type=float, default=D["T_REST"])
    ap.add_argument("--axes", type=lambda s: [int(x) for x in s.split(",") if x], default=D["AXES"])
    ap.add_argument("--wheel-max", type=float, default=D["WHEEL_MAX"], help="드라이버 max_wheel_rad_per_sec")
    ap.add_argument("--rate", type=float, default=RATE_HZ, help="기록·명령 주기 [Hz] (드라이버 update_rate_hz 와 같게)")
    a, _ = ap.parse_known_args()          # ROS 인자(--ros-args) 는 무시
    RATE_HZ = a.rate
    AMP_LIN, AMP_ANG, RAMP_S, REPEATS = a.amps_lin, a.amps_ang, a.ramp, a.repeats
    T_STEP, T_REST, AXES, WHEEL_MAX = a.t_step, a.t_rest, a.axes, a.wheel_max
    check_amplitudes()


def main():
    parse_args()
    # 시그널 핸들러를 rclpy 에 넘기면 Ctrl+C 순간 context 가 먼저 죽어서
    # 정지명령 발행도, 그 뒤 CSV 저장도 못 하고 끝난다. 직접 받는다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = StepLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()
    finally:
        node.finish()          # 정상종료·예외 어느 쪽이든 저장 보장 (2회차는 no-op)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()