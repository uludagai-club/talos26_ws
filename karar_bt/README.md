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
| `/engel`, `/engel_distance`, `/engel_sol_mesafe`, `/engel_sag_mesafe`, `/engel_angle` | engel ekibi | merkez/sol/sağ minimum mesafeler |
| `/line`, `/lane_offset` | şerit ekibi | yalnız gözlem (lane takibini control.py kullanıyor) |
| `/base_pose_ground_truth` | sim | Odometri |
| `/hedef` | hedef yöneticisi | yalnız gözlem; mission progression Samed/Hilmi'de kalır |

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

12 senaryo (yaya yakın/orta/uzak, DUR levhası 3-faz, engel kaçınma, hız sınırı,
sensör stale) tek seferde çalışır. Çıkış kodu 0 = hepsi geçti.

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
