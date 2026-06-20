# karar_bt — Behavior Tree tabanlı karar mekanizması

**Modül:** `karar_bt/`
**Sorumlu:** Enes
**Yer aldığı servis:** `karar-node` (docker-compose.yml)
**Önceki sürüm:** `fixes/karar.py` (artık compose'tan bağlı değil; geri dönüş için git'te kalır)

## Neden BT?

Klasik if/elif zinciri (eski `karar.py`) yeni davranış eklemek istediğinde tüm
kararı yeniden okumayı gerektiriyordu. Behavior Tree:
- Her davranış küçük, izole bir node.
- Öncelikler ağaç yapısından okunuyor (Safety > Pedestrian > StopSign > … > Cruise).
- Debounce / latch / sensor freshness yan etkisiz decorator'larla eklenir.
- Test edilebilir: ROS olmadan blackboard'u elle doldurup ağacı tick'leyebiliyoruz
  (bkz. `test/replay_scenarios.py`).

## Çıkış kontratı (eski karar.py ile bire-bir)

| Topic | Tip | Notlar |
|---|---|---|
| `/karar` | `std_msgs/String` | `"normal" \| "slow" \| "dur" \| "acildurus" \| "sag" \| "sol"` — `control.py` dinler |
| `/karar_decision` | `cart_sim/Decision` | `decision_id`, `reason`, `phase`, `wait_remaining_s`, `input_*`, `yaya_distance`, `levha_class` |
| `/karar_bt/snapshot` | `std_msgs/String` (JSON) | Debug — blackboard özet + ağaç durumu |

Ek olarak `talos_common.TalosLogger` üzerinden `component=karar` CSV log'u —
eski şema aynen (`decision_id, karar, reason, input_*, yaya_distance, levha_class, phase, wait_remaining_s`).

## Giriş (yalnız subscribe — başkalarının topic'leri)

| Topic | Kaynak | Format |
|---|---|---|
| `/trafik_levha` | levha ekibi (yolov8) | `"ISIM,x,y"` (ISIM ∈ DUR/SAG/SOL/30/OKUL/YAVAS/KIRMIZI) |
| `/yaya_gecidi` | yaya ekibi | `"x,y"` |
| **`/obstacles/poses`** | **`talos_obstacle_detector` (YENİ, DBSCAN+OBB)** | **`geometry_msgs/PoseArray` — engel konumları; BT içinde sektörlere indirgenir** |
| `/engel`, `/engel_distance`, `/engel_sol_mesafe`, `/engel_sag_mesafe`, `/engel_angle` | engel ekibi (ESKİ skaler node) | merkez/sol/sağ minimum mesafeler — yeni kaynak yoksa fallback |
| `/line`, `/lane_offset` | şerit ekibi | yalnız gözlem (lane takibini control.py kullanıyor) |
| `/base_pose_ground_truth` | sim | Odometri |
| `/hedef` | hedef yöneticisi | yalnız gözlem; mission progression Samed/Hilmi'de kalır |

### Engel kaynağı — çift arayüz + otomatik failover

İki engel arayüzü desteklenir, `config/params.yaml → obstacle.source` ile:

- **`auto`** (varsayılan): `/obstacles/poses` son `new_source_max_age_s` (0.5s)
  içinde geldiyse yeni detektör kullanılır ve eski skaler `/engel_*` topic'leri
  **yok sayılır** (çift sayım olmaz). Yeni detektör yoksa eski node'a düşer.
- **`poses`**: yalnız yeni `talos_obstacle_detector` (`/obstacles/poses`).
- **`legacy`**: yalnız eski `engel_node_fixed.py` skaler topic'leri.

PoseArray konumları araç gövde çerçevesinde `(forward, left)` olarak okunur
(REP-103: x ileri, y sol — sensör farklıysa `obstacle.axis_*`/`invert_*` ile
düzeltilir) ve `obstacle_fusion.py` ile merkez/sol/sağ minimum mesafelere
indirgenir. Böylece **ağaç mantığı her iki kaynakta da aynıdır.**

> Hangi engel node'unun açık olduğuna bakılmaksızın karar düğümü çalışır —
> akşam sim'i kuran arkadaşın engel tarafında ne çalıştırdığı karar düğümünü
> kilitlemez.

> Hiçbir başkasının modülüne **dokunulmadı**. BT yalnızca okur ve `/karar`
> kontratını üretir.

## Ağaç şeması (öncelik yukarıdan aşağı)

```
Root (Selector)
├── 0. ReleaseEmergencyIfClear   (mührü çöz; çözüldüyse FAILURE)
├── 1. EmergencyTrigger          (yaya<2m | engel<2m → LatchEmergency → "acildurus")
├── 2. Pedestrian                (yaya<4m → "dur", yaya<12m → "slow") + debounce
├── 3. StopSign FSM              (approach="slow" → hold="dur" 3s → released)
├── 4. TrafficLight              ("KIRMIZI"→dur, "YAVAS"→slow)
├── 5a.LaneChangeHold            (başlayan manevrayı maneuver_hold_s boyunca tut — control.py senkronu)
├── 5. Obstacle avoidance        (lane change varsa "sol"/"sag", yoksa "dur")
├── 6. DirectionSign             (SAG/SOL<5m → "sag"/"sol")
├── 7. SpeedLimit                ("30"/"OKUL"<10m → "slow")
└── 8. Cruise (default)          → "normal"
```

Bütün eşikler `config/params.yaml`'da; sahada `docker compose restart karar-node`
ile uygulanır (bind-mount).

## Mimari katmanları

```
karar_bt/
├── bb.py                # Blackboard + Observations + StatePersist (yan etkisiz)
├── obstacle_fusion.py   # PoseArray engel konumları → sektör skalerleri (rospy'siz, test edilebilir)
├── ros_bridge.py        # Subscriber callback'leri (yalnız blackboard'a yazar) + Publisher
├── behaviors/
│   ├── conditions.py    # Yan etkisiz koşul node'ları
│   ├── actions.py       # SetKarar, DurLevhasiFSM, LatchEmergency, …
│   └── decorators.py    # Debounce
├── trees/main_tree.py   # Ağacı kuran build_root(bb, params)
├── karar_bt_node.py     # Ana giriş: tick döngüsü @ 10Hz
├── config/params.yaml   # Tüm eşik & zaman & debounce parametreleri
├── test/
│   ├── replay_scenarios.py   # ROS'suz offline senaryo (12 case)
│   └── smoke_test.sh         # Canlı ROS smoke (rostopic pub ile)
└── Dockerfile
```

Tüm ROS bağımlılığı `ros_bridge.py` + `karar_bt_node.py` içindedir;
`behaviors/` ve `trees/` saf Python ⇒ rospy olmadan da test edilebilir.

## Çalıştırma

### Sim'i kuran arkadaş için hızlı talimat

Karar düğümü hazır bir servis (`karar-node`) — ekstra kurulum gerektirmez,
`talos-all` imajını kullanır (yeni bağımlılık yok; PoseArray `geometry_msgs`
zaten imajda). Tek yapman gereken sistemi ayağa kaldırmak:

```bash
cd ~/talos26_ws
./setup-vcan.sh                 # vcan0 + X11 (bir kez)
docker compose up               # tüm sistem (karar-node dahil)
# veya yalnız karar düğümü (roscore + diğer node'lar ayaktayken):
docker compose up karar-node
```

Engel tarafında **ne çalıştırdığın fark etmez:**
- **Yeni** `talos_obstacle_detector` (`/obstacles/poses`) açıksa BT onu kullanır.
- Yoksa eski `engel-node` (skaler `/engel_*`) fallback olur.
- İkisi birden açıksa yeni kaynak öncelikli, eski yok sayılır (çift sayım yok).

Çalıştığını 5 saniyede doğrula:
```bash
rostopic hz /karar                       # ~10 Hz akmalı
rostopic echo -n1 /karar_bt/snapshot     # engel.source = "poses" | "legacy"
```

### Docker (sahada)
```bash
cd ~/talos26_ws
docker compose build karar-node
docker compose up karar-node
```

### Yerel (hızlı debug)
```bash
cd ~/talos26_ws/karar_bt
pip3 install py_trees==2.2.3 pyyaml
source /opt/ros/noetic/setup.bash
python3 karar_bt_node.py
```

### Offline senaryo testi (ROS yok)
```bash
cd ~/talos26_ws/karar_bt
python3 -m test.replay_scenarios
```

26 senaryo (yaya yakın/orta/uzak, DUR levhası 3-faz, engel kaçınma, hız sınırı,
sensör stale, **yeni PoseArray detektörü S21-S23**, **şerit-değişimi manevra
kilidi S17/S24/S25**, **DUR release_grace çift-duruş S26**) tek seferde çalışır.
Çıkış kodu 0 = hepsi geçti.

Engel füzyon geometrisini ayrıca test et (ROS yok):
```bash
python3 -m test.test_obstacle_fusion
```

### Canlı smoke testi (roscore + karar-node ayakta)
```bash
cd ~/talos26_ws/karar_bt
./test/smoke_test.sh
```

## Geri dönüş

Eğer BT'de bug → eski `fixes/karar.py` aynen duruyor. Tek satır revert:
`docker-compose.yml` içinde `karar-node` bloğundaki `image`/`build`/`command`/`volumes`
satırlarını eski haline döndür → `docker compose restart karar-node`.

## Parametre tuning

`config/params.yaml` içinde:
- `freshness.*_max_age_s` — sensor stale eşikleri (0.4–0.8s civarı sahaya göre)
- `distances.*` — eski karar.py değerleriyle aynı başladı
- `timers.dur_levhasi_bekleme_s` — kurallar gereği 3.0s
- `debounce.*_min_consecutive` — flicker önleme (2 tick ≈ 200ms @ 10Hz)
- `emergency.release_clear_ticks` — mühür çözülme süresi (~0.8s @ 10Hz)
- `lane_change.cooldown_s` — ardışık şerit değişimi arası

## Debug

Ağacın anlık durumu:
```bash
rostopic echo -n 1 /karar_bt/snapshot
```
JSON döner: blackboard + son karar + ascii ağaç (yapılandırılmışsa).

ROS log:
```bash
rosnode info /karar_bt
rostopic hz /karar
```
