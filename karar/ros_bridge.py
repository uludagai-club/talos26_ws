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
from geometry_msgs.msg import PoseArray
from tf.transformations import euler_from_quaternion

from bb import Blackboard
from obstacle_fusion import (ObstacleFusionParams, fuse_obstacles,
                             ArcGateParams, arc_blocking_distance)
from obstacle_memory import ObstacleMemory, MemParams

try:
    from cart_sim.msg import Decision as DecisionMsg
    _HAS_DECISION = True
except Exception:
    DecisionMsg = None
    _HAS_DECISION = False

try:
    from cart_sim.msg import cart_control as CartControlMsg
    _HAS_CART_CONTROL = True
except Exception:
    CartControlMsg = None
    _HAS_CART_CONTROL = False


# Pose pozisyonundan eksen seçici (axis_forward/axis_left config'i için)
def _axis_get(position, axis: str) -> float:
    return float(getattr(position, axis))


class RosBridge:
    def __init__(self, bb: Blackboard, params: dict | None = None):
        self.bb = bb
        params = params or {}

        # --- Engel kaynağı yapılandırması ---
        oc = params.get("obstacle", {}) or {}
        self._obs_source = str(oc.get("source", "auto")).lower()
        self._new_obs_max_age_s = float(oc.get("new_source_max_age_s", 0.5))
        self._fusion_p = ObstacleFusionParams.from_cfg(oc)
        self._axis_fwd = str(oc.get("axis_forward", "x")).lower()
        self._axis_left = str(oc.get("axis_left", "y")).lower()
        self._inv_fwd = -1.0 if bool(oc.get("invert_forward", False)) else 1.0
        self._inv_left = -1.0 if bool(oc.get("invert_left", False)) else 1.0
        self._new_obs_last = 0.0   # /obstacles/poses son geliş zamanı (failover için)

        # --- Yay-kapısı (2026-07-15): acil bandı bisiklet-modeli farkındalı ---
        # Mevcut direksiyonun süpürme bandı DIŞINDA kalan nesne acildurus
        # tetiklemez (canlı bulgu 173335: 41° yan bordür 2 dk kilitledi).
        # Direksiyon kaynağı: /cart (cart_control.steer ∈ [-1,1], + sol) ×
        # steer_full_deg. Veri yok/bayat → d_arc=d_center (bugünkü davranış).
        ag = oc.get("arc_gate", {}) or {}
        self._arc_gp = ArcGateParams.from_cfg(ag)
        self._arc_steer_topic = str(ag.get("steer_topic", "/cart"))
        self._arc_enabled = self._arc_gp.enabled and _HAS_CART_CONTROL
        if self._arc_gp.enabled and not _HAS_CART_CONTROL:
            rospy.logwarn("[karar_bt] yay-kapısı: cart_sim.msg.cart_control "
                          "import edilemedi — acil bandı d_center'a düşüyor.")

        # --- Duba DÜNYA-konum hafızası (dropout köprüsü) ---
        # Detektör kareyi düşürünce konfirme dubayı gövde-frame'e geri-projekte edip
        # füzyon girdisine enjekte eder → "lidar veri vermese bile duba orada".
        # Pose (x,y,yaw) bu tazelikten eski ise hafıza atlanır (lokalizasyon yoksa
        # sahte konum üretme). bkz obstacle_memory.py (canlı teşhis 2026-06-26).
        mc = oc.get("memory", {}) or {}
        self._obs_mem = (ObstacleMemory(MemParams.from_cfg(mc))
                         if bool(mc.get("enabled", True)) else None)
        self._odom_max_age_s = float(
            (params.get("freshness", {}) or {}).get("odom_max_age_s", 0.5))

        # --- Subscribers (yalnız okuma) ---
        rospy.Subscriber("/trafik_levha", String, self._on_levha, queue_size=10)
        # Adanmis 2-sinifli yaya gecidi modeli (yaya_gecidi_node). Bare /yaya_gecidi
        # levha modelinin 26-sinif icindeki zayif yaya tespiti; adanmis model esas alinir.
        rospy.Subscriber("/yaya_gecidi/model", String, self._on_yaya, queue_size=10)

        # YENI engel arayüzü: talos_obstacle_detector → /obstacles/poses (PoseArray)
        if self._obs_source in ("auto", "poses"):
            # queue_size=1: hafıza köprüsü dünya konumunu GÜNCEL pozla hesaplıyor →
            # birikmiş bayat PoseArray güncel pozla işlenirse iz kayar (/incele performans).
            rospy.Subscriber("/obstacles/poses", PoseArray, self._on_obstacles_poses, queue_size=1)

        # ESKI skaler engel arayüzü (auto: yeni kaynak yoksa devreye girer)
        if self._obs_source in ("auto", "legacy"):
            rospy.Subscriber("/engel",            Int32,   self._on_engel,        queue_size=10)
            rospy.Subscriber("/engel_distance",   Float32, self._on_engel_dist,   queue_size=10)
            rospy.Subscriber("/engel_angle",      Float32, self._on_engel_angle,  queue_size=10)
            rospy.Subscriber("/engel_sol_mesafe", Float32, self._on_engel_sol,    queue_size=10)
            rospy.Subscriber("/engel_sag_mesafe", Float32, self._on_engel_sag,    queue_size=10)

        # Yay-kapısı direksiyon kaynağı: control'ün gönderdiği komut açısı
        # (can-bridge → /cart). Komut açısı control'ün kendi e-stop'uyla da
        # aynı referanstır (_prev_cmd_steer) → iki katman tutarlı karar verir.
        if self._arc_enabled:
            rospy.Subscriber(self._arc_steer_topic, CartControlMsg,
                             self._on_cart_steer, queue_size=1)

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

        # karar → hedef komut kanalı (gelistirme_plani §3.2). Olay-komutu olduğu
        # için latch=False + queue_size=1: restart'ta eski "sollama" tekrar
        # ateşlenmesin. String-prototip ("komut;taraf;x;y;etiket;yaricap"); sözleşme
        # oturunca cart_sim/HedefKomut.msg'e terfi edilebilir.
        self.pub_hedef_komut = rospy.Publisher("/hedef_komut", String, queue_size=1)
        self._last_hedef_komut = ""

        # Görselleştirme: hafızadaki dubaların dünya (odom) konumu → can_visualizer
        # harita panelinde çizer. Salt-görsel; karar davranışını etkilemez.
        # Biçim: "x,y,conf|x,y,conf|..." (conf=1 konfirme, 0 aday).
        self.pub_hafiza_koni = rospy.Publisher("/karar/hafiza_koni", String, queue_size=2)

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

    # --- YENI engel kaynağı: /obstacles/poses (talos_obstacle_detector) ---
    def _on_obstacles_poses(self, msg: PoseArray):
        """PoseArray engel konumlarını sektör skalerlerine indirger.

        Boş array de geçerlidir → "temiz" (engel yok) anlamına gelir; tazelik
        damgası yine de güncellenir ki BT 'temiz'e güvenebilsin.
        """
        now = time.time()
        points = []
        for ps in msg.poses:
            pos = ps.position
            try:
                fwd = _axis_get(pos, self._axis_fwd) * self._inv_fwd
                lat = _axis_get(pos, self._axis_left) * self._inv_left
            except AttributeError:
                continue
            points.append((fwd, lat, 0.0))  # PoseArray boyut taşımaz → half_width=0

        # Hafıza köprüsü: pose taze ise konfirme dubaları dropout'ta yeniden enjekte et.
        # Pose lock altında tutarlı okunur; update() try/except'le sarılı → odom NaN/
        # math hatası callback'i çökertip engel körlüğü yapmasın (/incele güvenlik Yüksek).
        n_mem = 0
        if self._obs_mem is not None:
            rx, ry, ryaw, odom_ts = self.bb.read_pose()
            pose_fresh = (odom_ts > 0.0 and (now - odom_ts) <= self._odom_max_age_s)
            if pose_fresh:
                try:
                    points, stats = self._obs_mem.update(points, rx, ry, ryaw, now)
                    n_mem = stats["injected"]
                except Exception as e:
                    rospy.logwarn_throttle(2.0, f"[karar_bt] obstacle_memory hata: {e}")

        fused = fuse_obstacles(points, self._fusion_p)

        # Yay-kapısı: taze direksiyon varsa acil bandı için bant-içi en yakın
        # engel; yoksa d_center (fail-safe = eski düz-koridor davranışı).
        d_arc = fused.d_center
        if self._arc_enabled:
            o = self.bb.obs
            steer_fresh = (o.steer_last_seen > 0.0 and
                           (now - o.steer_last_seen) <= self._arc_gp.steer_max_age_s)
            if steer_fresh:
                try:
                    d_arc = arc_blocking_distance(points, o.steer_deg,
                                                  self._fusion_p, self._arc_gp)
                except Exception as e:
                    rospy.logwarn_throttle(2.0, f"[karar_bt] yay-kapısı hata: {e}")
                    d_arc = fused.d_center

        self._new_obs_last = now
        self.bb.write(
            engel_present=fused.present,
            engel_d_arc=d_arc,
            engel_d_center=fused.d_center,
            engel_d_overall=fused.d_overall,
            engel_d_left=fused.d_left,
            engel_d_right=fused.d_right,
            engel_angle_deg=fused.angle_deg,
            engel_count=fused.count,
            engel_mem_count=n_mem,
            engel_source="poses+mem" if n_mem > 0 else "poses",
            engel_last_seen=now,
            engel_left_last_seen=now,
            engel_right_last_seen=now,
        )

        # Görselleştirme: hafızadaki dubaların dünya konumu (odom-frame x,y +
        # konfirme). Salt-görsel; can_visualizer harita panelinde çizer.
        if self._obs_mem is not None:
            try:
                trk = self._obs_mem.world_tracks()
                payload = "|".join(f"{x:.2f},{y:.2f},{1 if c else 0}"
                                   for x, y, c in trk)
                self.pub_hafiza_koni.publish(payload)
            except Exception:
                pass

    # --- Failover: yeni kaynak tazeyse eski skaler topic'ler yok sayılır ---
    def _legacy_suppressed(self) -> bool:
        if self._obs_source == "legacy":
            return False
        return (time.time() - self._new_obs_last) < self._new_obs_max_age_s

    def _on_engel(self, msg: Int32):
        if self._legacy_suppressed():
            return
        self.bb.write(engel_present=bool(msg.data), engel_source="legacy",
                      engel_last_seen=time.time())

    def _on_engel_dist(self, msg: Float32):
        if self._legacy_suppressed():
            return
        v = msg.data if math.isfinite(msg.data) else float("inf")
        # Legacy skaler arayüzde nokta listesi yok → yay-kapısı uygulanamaz;
        # d_arc = d_center (düz-koridor davranışı korunur).
        self.bb.write(engel_d_overall=v, engel_d_center=v, engel_d_arc=v,
                      engel_source="legacy", engel_last_seen=time.time())

    def _on_engel_angle(self, msg: Float32):
        if self._legacy_suppressed():
            return
        self.bb.write(engel_angle_deg=float(msg.data), engel_last_seen=time.time())

    def _on_engel_sol(self, msg: Float32):
        if self._legacy_suppressed():
            return
        v = msg.data if math.isfinite(msg.data) else float("inf")
        now = time.time()
        self.bb.write(engel_d_left=v, engel_left_last_seen=now, engel_last_seen=now)

    def _on_engel_sag(self, msg: Float32):
        if self._legacy_suppressed():
            return
        v = msg.data if math.isfinite(msg.data) else float("inf")
        now = time.time()
        self.bb.write(engel_d_right=v, engel_right_last_seen=now, engel_last_seen=now)

    def _on_cart_steer(self, msg):
        """cart_control.steer ∈ [-1,1] (+ sol) → bisiklet-modeli derece."""
        try:
            s = float(msg.steer)
        except (AttributeError, TypeError, ValueError):
            return
        if not math.isfinite(s):
            return
        s = max(-1.0, min(1.0, s))
        self.bb.write(steer_deg=s * self._arc_gp.steer_full_deg,
                      steer_last_seen=time.time())

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

    def publish_hedef_komut(self, cmd: str):
        """OvertakeManager'ın ürettiği komutu /hedef_komut'a yayınla.

        Boş/None komut yok sayılır. Aynı komut art arda gelirse de yayınlanır
        (queue=1, latch=False → hedef tarafı son komutu görür; sollama tazeleme
        kasıtlı tekrar). hedef abone tarafı Samed'in işi (bkz plan §3.2)."""
        if not cmd:
            return
        try:
            self.pub_hedef_komut.publish(cmd)
            self._last_hedef_komut = cmd
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[karar_bt] /hedef_komut yayını başarısız: {e}")

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
