"""ROS ↔ Blackboard köprüsü.

Subscriber'lar yalnız Blackboard.obs'e yazar; Publisher'lar tick döngüsünden
çağrılır. Hiçbir behavior bu modüle bağımlı değil — test'te bypass edilir.
"""
from __future__ import annotations

import json
import time
import uuid
import math

import rospy
from std_msgs.msg import String, Int32, Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from bb import Blackboard

try:
    from cart_sim.msg import Decision as DecisionMsg
    _HAS_DECISION = True
except Exception:
    DecisionMsg = None
    _HAS_DECISION = False


class RosBridge:
    def __init__(self, bb: Blackboard):
        self.bb = bb

        # --- Subscribers (yalnız okuma) ---
        rospy.Subscriber("/trafik_levha", String, self._on_levha, queue_size=10)
        rospy.Subscriber("/yaya_gecidi",   String, self._on_yaya,   queue_size=10)

        rospy.Subscriber("/engel",            Int32,   self._on_engel,        queue_size=10)
        rospy.Subscriber("/engel_distance",   Float32, self._on_engel_dist,   queue_size=10)
        rospy.Subscriber("/engel_angle",      Float32, self._on_engel_angle,  queue_size=10)
        rospy.Subscriber("/engel_sol_mesafe", Float32, self._on_engel_sol,    queue_size=10)
        rospy.Subscriber("/engel_sag_mesafe", Float32, self._on_engel_sag,    queue_size=10)

        rospy.Subscriber("/line",        Float32, self._on_line,   queue_size=10)
        rospy.Subscriber("/lane_offset", Float32, self._on_offset, queue_size=10)

        rospy.Subscriber("/base_pose_ground_truth", Odometry, self._on_odom, queue_size=10)
        rospy.Subscriber("/hedef", String, self._on_hedef, queue_size=10)

        # --- Publishers (eski karar.py ile bire-bir uyumlu) ---
        self.pub_karar = rospy.Publisher("/karar", String, queue_size=10)
        if _HAS_DECISION:
            self.pub_decision = rospy.Publisher("/karar_decision", DecisionMsg, queue_size=10)
        else:
            self.pub_decision = None
            rospy.logwarn("[karar_bt] cart_sim.msg.Decision import edilemedi — yalnız /karar yayınlanacak.")

        self.pub_snapshot = rospy.Publisher("/karar_bt/snapshot", String, queue_size=2)
        self._snapshot_period_s = 0.5
        self._snapshot_last = 0.0

    # ============================================================
    # Subscriber callback'leri — yalnız blackboard'a yazar
    # ============================================================
    def _on_levha(self, msg: String):
        raw = (msg.data or "").strip()
        self.bb.obs.raw_levha = raw if raw else "none"
        if not raw or raw.lower() == "none":
            self.bb.write(
                levha_isim="NONE",
                levha_distance=-1.0,
                levha_x=-1.0, levha_y=0.0,
            )
            return
        try:
            parts = raw.split(",")
            isim = parts[0].strip().upper()
            x = float(parts[1])
            y = float(parts[2]) if len(parts) > 2 else 0.0
            d = math.hypot(x, y)
            self.bb.write(
                levha_isim=isim,
                levha_x=x, levha_y=y,
                levha_distance=d,
                levha_last_seen=time.time(),
            )
        except Exception:
            rospy.logwarn_throttle(5.0, f"[karar_bt] levha parse hatası: {raw!r}")

    def _on_yaya(self, msg: String):
        raw = (msg.data or "").strip()
        self.bb.obs.raw_yaya = raw if raw else "none"
        if not raw or raw.lower() == "none":
            self.bb.write(yaya_present=False, yaya_distance=-1.0, yaya_x=-1.0, yaya_y=0.0)
            return
        try:
            parts = raw.split(",")
            x = float(parts[0]); y = float(parts[1])
            d = math.hypot(x, y)
            self.bb.write(
                yaya_present=True,
                yaya_x=x, yaya_y=y,
                yaya_distance=d,
                yaya_last_seen=time.time(),
            )
        except Exception:
            rospy.logwarn_throttle(5.0, f"[karar_bt] yaya parse hatası: {raw!r}")

    def _on_engel(self, msg: Int32):
        self.bb.write(engel_present=bool(msg.data), engel_last_seen=time.time())

    def _on_engel_dist(self, msg: Float32):
        v = msg.data if math.isfinite(msg.data) else float("inf")
        self.bb.write(engel_d_overall=v, engel_d_center=v, engel_last_seen=time.time())

    def _on_engel_angle(self, msg: Float32):
        self.bb.write(engel_angle_deg=float(msg.data), engel_last_seen=time.time())

    def _on_engel_sol(self, msg: Float32):
        v = msg.data if math.isfinite(msg.data) else float("inf")
        self.bb.write(engel_d_left=v, engel_last_seen=time.time())

    def _on_engel_sag(self, msg: Float32):
        v = msg.data if math.isfinite(msg.data) else float("inf")
        self.bb.write(engel_d_right=v, engel_last_seen=time.time())

    def _on_line(self, msg: Float32):
        self.bb.write(line_angle_deg=float(msg.data), lane_last_seen=time.time())

    def _on_offset(self, msg: Float32):
        self.bb.write(lane_offset_px=float(msg.data), lane_last_seen=time.time())

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed_ms = math.hypot(vx, vy)
        self.bb.write(
            x=p.position.x, y=p.position.y, yaw=yaw,
            speed_kmh=speed_ms * 3.6,
            odom_last_seen=time.time(),
        )

    def _on_hedef(self, msg: String):
        raw = (msg.data or "").strip()
        if not raw:
            return
        try:
            segments = raw.split("|") if "|" in raw else raw.split(";")
            x, y = [float(v) for v in segments[0].split(",")[:2]]
            nx, ny = (None, None)
            if len(segments) > 1:
                nx, ny = [float(v) for v in segments[1].split(",")[:2]]
            self.bb.write(
                hedef_x=x, hedef_y=y,
                next_hedef_x=nx, next_hedef_y=ny,
                hedef_last_seen=time.time(),
            )
        except Exception:
            rospy.logwarn_throttle(5.0, f"[karar_bt] hedef parse hatası: {raw!r}")

    # ============================================================
    # Publisher — tick döngüsünden çağrılır
    # ============================================================
    def publish_decision(self):
        d = self.bb.last_decision
        karar = d.get("karar", "normal")
        reason = d.get("reason", "")
        phase = d.get("phase", "driving")
        wait_remaining = float(d.get("wait_remaining_s", 0.0))

        # 1) Geri uyumlu String
        self.pub_karar.publish(karar)

        # 2) Yapısal Decision msg
        if self.pub_decision is not None:
            decision_id = uuid.uuid4().hex
            self.bb.state.last_decision_id = decision_id

            m = DecisionMsg()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "karar_bt"
            m.decision_id = decision_id
            m.karar = karar
            m.reason = reason
            m.input_yaya = self.bb.obs.raw_yaya
            m.input_levha = self.bb.obs.raw_levha
            m.input_engel = "1" if self.bb.obs.engel_present else "0"
            m.yaya_distance = float(self.bb.obs.yaya_distance if self.bb.obs.yaya_distance is not None else -1.0)
            m.levha_class = self.bb.obs.levha_isim or "NONE"
            m.phase = phase
            m.wait_remaining_s = wait_remaining
            self.pub_decision.publish(m)

    def publish_snapshot(self, tree_ascii: str = ""):
        if (time.time() - self._snapshot_last) < self._snapshot_period_s:
            return
        self._snapshot_last = time.time()
        payload = {
            "tick_t": time.time(),
            "bb": self.bb.snapshot(),
            "tree": tree_ascii,
        }
        try:
            self.pub_snapshot.publish(json.dumps(payload, default=str))
        except Exception:
            pass
