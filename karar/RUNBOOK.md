# karar_bt — Çalıştırma Runbook'u

`karar-node` (Behavior Tree karar düğümü) operatör başvurusu. Modül mimarisi ve
ağaç şeması için `README.md`; sistemin tamamı için kökteki
`KURULUM_VE_SORUN_GIDERME.md`.

---

## Karar düğümünün sistemdeki yeri

```
sensör node'ları                         control
(/trafik_levha, /yaya_gecidi,   →   karar-node   →   /karar (String)   →   talos-controller
 /obstacles/poses | /engel_*,        (BT @10Hz)      /karar_decision        (control.py)
 /base_pose_ground_truth)                            /karar_bt/snapshot
```

karar-node yalnız okur ve `/karar` kontratını üretir; başka hiçbir modüle yazmaz.
`talos-all:latest` imajını kullanır (py_trees imajda gömülü), `./karar_bt`
bind-mount edilir → **kod/param değişince rebuild değil, restart yeter**.
`cart_sim/Decision` mesajı için `~/talos-sim/devel` mount'u gerekir (sim derlenmiş
olmalı); yoksa yalnız `/karar` String yayınlanır.

---

## Çalıştırma

Karar düğümü tek başına anlamlı değil — roscore, sim ve sensör/control node'ları
ayakta olmalı. Tam sistemi başlatmak (önerilen):

```bash
# Terminal 1 — roscore
source ~/talos-sim/devel/setup.bash && roscore
# Terminal 2 — Gazebo sim
source ~/talos-sim/devel/setup.bash && roslaunch cart_sim cart_sim.launch
# Terminal 3 — tüm Docker yığını (karar-node dahil)
cd ~/talos-sim/scripts/talos26_ws && bash baslat.sh
```

Diğer node'lar ayaktayken **yalnız karar düğümü**:
```bash
cd ~/talos-sim/scripts/talos26_ws
docker compose up karar-node            # veya: docker compose up -d karar-node
```

Yerel hızlı debug (Docker'sız):
```bash
cd ~/talos-sim/scripts/talos26_ws/karar_bt
pip3 install py_trees==2.2.3 pyyaml
source /opt/ros/noetic/setup.bash
python3 karar_bt_node.py
```

---

## Doğrulama (5 saniyede)

```bash
rostopic hz /karar                      # ~10 Hz akmalı
rostopic echo -n1 /karar_bt/snapshot    # engel.source = "poses" | "legacy", anlık karar
docker compose logs -f karar-node       # "karar_change: X -> Y (reason)" satırları
```

Öncelik (yukarıdan aşağı): `acildurus` > yaya `dur`/`slow` > DUR levhası 3-faz >
trafik ışığı > **şerit-değişimi kilidi** > engel kaçışı > yön levhası > hız sınırı
> `normal`.

Şerit-değişimi kilidi: engel kaçışı veya yön levhası tetiklendiğinde `/karar`,
manevra penceresi (`config/params.yaml → lane_change.maneuver_hold_s`, control.py
`LANE_CHANGE_DURATION` ile eşit ~2s) boyunca aynı `sol`/`sag` komutunu **tutar**;
böylece control.py'nin başlattığı manevra `dur`/`normal` ile kesilmez.

---

## Parametre tuning / restart

Tüm eşik, zaman ve debounce değerleri `config/params.yaml`'da. Değişiklik sahaya:
```bash
docker compose restart karar-node       # bind-mount → rebuild yok
```

Sık tune edilenler:
- `distances.*` — yaya/engel/levha mesafe eşikleri
- `timers.dur_levhasi_bekleme_s` — DUR levhası bekleme (kurallar: 3.0s)
- `lane_change.maneuver_hold_s` — control.py `LANE_CHANGE_DURATION` ile eşit tut
- `debounce.*_min_consecutive` — flicker önleme (2 tick ≈ 200ms @10Hz)
- `freshness.*_max_age_s` — sensör stale eşikleri

---

## Offline test (ROS gerekmez)

```bash
cd ~/talos-sim/scripts/talos26_ws/karar_bt
python3 -m test.replay_scenarios        # 26 senaryo; exit 0 = hepsi geçti
python3 -m test.test_obstacle_fusion    # engel füzyon geometrisi
```

---

## Sorun giderme

| Belirti | Çözüm |
|---|---|
| `/karar` akmıyor | roscore + sim ayakta mı? `docker compose logs karar-node` |
| snapshot'ta `Decision import edilemedi` | Sim derlenmemiş → `cd ~/talos-sim && catkin_make` (yine de `/karar` String akar) |
| `engel.source` hep `legacy` | Yeni detektör `/obstacles/poses` yayınlamıyor; `rostopic hz /obstacles/poses` |
| Karar beklenmedik | `rostopic echo -n1 /karar_bt/snapshot` ile blackboard'u oku (mesafe/tazelik) |
| Param değişikliği etkisiz | `docker compose restart karar-node` |

---

## Geri dönüş

BT'de sorun olursa eski `fixes/karar.py` git'te duruyor. `docker-compose.yml`
içindeki `karar-node` bloğunu eski haline döndürüp `docker compose restart
karar-node` yeterli (bkz. `README.md` → Geri dönüş).
