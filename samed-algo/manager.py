# -*- coding: utf-8 -*-
import sys
import os
import time
import math
import threading
try:
    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import Pose2D
except ImportError:
    rospy = None
    class String:
        pass
    class Pose2D:
        pass

try:
    from hedef_logger import HedefLogger
except Exception as _e:  # noqa: BLE001
    HedefLogger = None
    sys.stderr.write(f"[hedef_yoneticisi] hedef_logger yok, loglama kapalı: {_e}\n")

from hedef_son.config import (
    GOREV_GEOJSON, ENABLE_GUI, ILERI_MESAFE_M, ILERI_MESAFE_BLOK_M, SAPMA_ESIK_METRE,
    SAPMA_TEMIZ_METRE, SAPMA_DEBOUNCE_SURE, GOREV_YAKINLIK_M, YON_FILTRE_ACIISI,
    MATCH_PENCERE, MATCH_KORIDOR_M, CEZA_ETKI, CEZA_DUZ_SERIT, CEZA_BAGLANTI,
    CEZA_SERIT_DEGISTIRME, CEZA_TERS_YON, IKI_YONLU_DURAK_AKTIF, R_DURAK_M,
    TURN_AWARE_AKTIF, ARAC_DINGIL_M, ARAC_MAX_DIREKSIYON, DONUS_CEZA_AGIRLIK,
    DONUS_CEZA_MAX, DONUS_CUSP_ESIK, DONUS_CUSP_CEZA, HEDEF_KOMUT_AKTIF,
    BLOK_TTL_S, BLOK_MARJIN_M, KONUM_DEGISIM_ESIK_M, BLOK_SERT_AKTIF,
    BLOK_YARICAP_M, HESAP_KILIDI_AKTIF, DURMA_BEKLEME_SN, DURMA_HIZ_ESIK_MS,
    DURMA_YARICAP_M,
    KILIT_COOLDOWN_SN, KILIT_BYPASS_COOLDOWN_SN, SERIT_MARJIN_M, SLALOM_ENJEKSIYON_AKTIF,
    SLALOM_YALNIZ_GEREKINCE, CEZA_TERS_CIKIS, CEZA_TERS_KALMA, BLOK_EK_CEZA,
    SLALOM_ENJEKSIYON_R, KENAR_GUVENLI_M, CEZA_SIYIRMA_M, B_PLAN_YAW_ROTA_AKTIF,
    B_PLAN_OFFSET_M, B_PLAN_OFFSET_MARJIN_M, B_PLAN_LEAD_M, SADECE_ENGELDE_YENIDEN_PLANLA,
    SNAP_YAW_AGIRLIK, KONUM_FILTRE_AKTIF, KONUM_FILTRE_ALPHA, KONUM_JUMP_LIMIT_MS,
    SECILEN_SENARYO, CEZA_KARSIN_GECIS, CEZA_KARSIN_SEYIR
)

from hedef_son.dstar_lite import (
    DLitePlanner, ceza_carpani, _yon_farki, _nokta_segment_mesafe,
    bisiklet_donus_cezasi, rota_donus_metrigi
)

from hedef_son.graph_builder import build_track_graph
from hedef_son.visualizer import GraphVisualizer

YESIL  = "\033[92m"
KIRMIZI = "\033[91m"
SARI   = "\033[93m"
SIFIRLA = "\033[0m"

class HedefYoneticisi:
    def __init__(self):
        rospy.init_node('hedef_yoneticisi')

        # ── Görselleştirme verisi ────────────────────────────────────
        self.new_data_available = False

        # ── Robot durumu ─────────────────────────────────────────────
        self.robot_x         = None   # None: henüz konum gelmedi
        self.robot_y         = None
        self.robot_yaw       = None
        self._ilk_konum_alindi = False  # FIX: ilk konuma kadar görev kontrolü yapma

        # ── Rota durumu ──────────────────────────────────────────────
        self.full_path_world     = []
        self.current_wp_index    = 0
        self.current_task_index  = 0
        self.is_path_calculated  = False
        self.geo_targets_world   = []
        self.geo_targets_built   = False  # FIX: tek seferlik build kontrolü
        self._graph_loaded       = False

        # ── Zamanlayıcılar ───────────────────────────────────────────
        # FIX: 0.0 yerine time.time() → node başlar başlamaz cooldown aktif
        self.son_hesaplama_zamani = time.time()
        self._son_varildi_zamani  = time.time()
        self._son_gorev_zamani    = time.time()
        # FAZ2: sapmanın eşiği kesintisiz aştığı ilk an (debounce); eşik altına
        # düşünce None'a sıfırlanır. reroute ancak bu süre >= SAPMA_DEBOUNCE_SURE olunca.
        self._sapma_baslangic     = None

        # ── Thread güvenliği ─────────────────────────────────────────
        # FIX: varildi_callback & konum_callback çakışmasını önler
        self._wp_lock = threading.Lock()

        # ── Karar komutu (/hedef_komut) blok durumu ──────────────────
        # Her blok: {x, y, r, taraf, t}. TTL içinde aktif (karar ~1s tazeler).
        # _komut_lock callback↔recalc çakışmasını önler.
        self._bloklu_engeller = []
        self._komut_lock      = threading.Lock()

        # ── HESAPLAMA KİLİDİ (yol ayrımı + 15s durma + sağ-şerit açma) durumu ──
        self._forward_poz     = None    # forward(sağ) şerit düğüm pozisyonları; load'da
        self._karsi_poz       = None    # karşı(sol/overtake) şerit düğüm pozisyon KÜMESİ; load'da
        self._karsi_seritler  = set()   # sollama/overtake lane kümesi (B/D/F/...); load'da doldurulur
        self._hesap_kilitli   = False   # True → recalc bastırılır (sollama kararı verildi; sağ şerit/15s açar)
        self._kilit_sol_serite_girdi = False  # kilitliyken araç sol şeride girdi mi (sağ'a dönüş = açma şartı)
        self._son_sag_serit   = True    # _sag_seritte histerezis durumu (başlangıç: sağ şerit)
        self._son_kilit_recalc = 0.0    # son kilit-açma recalc zamanı (cooldown / churn engeli)
        self._path_creation_time = 0.0   # Rota kilit süre takibi için son rota oluşturma zamanı
        self._filtered_x      = None    # Filtrelenmiş X koordinatı
        self._filtered_y      = None    # Filtrelenmiş Y koordinatı
        self._filtered_yaw    = None    # Filtrelenmiş Yaw açısı
        self._last_konum_t    = None    # Zaman farkı ve outlier hız hesabı için son konum zamanı
        self._durma_capa_xy   = None    # durma tespiti ÇAPASI (x,y) — gürültü-dayanıklı (DURMA_YARICAP_M)
        self._durma_baslangic = None    # araç "durdu" sayıldığı an (15s sayacı); hareket edince None
        self._son_kilit_log_t = 0.0     # 1Hz loglama periyodu için zaman damgası
        self._son_bypass_t    = 0.0     # son bypass recalc zaman damgası
        # recalc'taki graf mutasyonunu (blok ağırlık-şişirme + slalom enjeksiyon)
        # serialize eder: konum_callback ile hedef_komut_callback ayrı thread'lerde
        # eşzamanlı recalc çağırırsa apply/restore çakışıp ağırlık sızdırmasın.
        self._graf_lock       = threading.Lock()
        self._slalom_conns    = []  # karşı-şerit crossing adayları (load'da doldurulur)
        self._slalom_segs     = []  # karşı-şerit boylamasına segment adayları (KALMA)

        # ── Tanı logu (kalıcı; docker kapanınca host'ta kalır) ───────
        self.logger = None
        if HedefLogger is not None:
            try:
                self.logger = HedefLogger()
                # SIGTERM/shutdown'da tamponları flush et (son satırlar kaybolmasın)
                rospy.on_shutdown(lambda: self.logger.close() if self.logger else None)
            except Exception as e:  # noqa: BLE001
                sys.stderr.write(f"[hedef_yoneticisi] logger başlatılamadı: {e}\n")

        # ── Config/ceza snapshot: bu run hangi parametre/ceza ile çalıştı ──
        # (ceza-puan ilişkisini sonradan loglardan analiz edebilmek için)
        if self.logger is not None:
            self.logger.log_event(
                "config",
                ILERI_MESAFE_M=ILERI_MESAFE_M,
                ILERI_MESAFE_BLOK_M=ILERI_MESAFE_BLOK_M,
                SAPMA_ESIK_METRE=SAPMA_ESIK_METRE,
                SAPMA_TEMIZ_METRE=SAPMA_TEMIZ_METRE,
                SAPMA_DEBOUNCE_SURE=SAPMA_DEBOUNCE_SURE,
                GOREV_YAKINLIK_M=GOREV_YAKINLIK_M,
                MATCH_PENCERE=MATCH_PENCERE,
                MATCH_KORIDOR_M=MATCH_KORIDOR_M,
                CEZA_ETKI=CEZA_ETKI,
                ceza_duz_serit=CEZA_DUZ_SERIT,
                ceza_baglanti=CEZA_BAGLANTI,
                ceza_serit_degistirme=CEZA_SERIT_DEGISTIRME,
                ceza_ters_yon=CEZA_TERS_YON,
                carpan_baglanti=round(ceza_carpani(CEZA_BAGLANTI), 3),
                carpan_serit_deg=round(ceza_carpani(CEZA_SERIT_DEGISTIRME), 3),
                iki_yonlu_durak=IKI_YONLU_DURAK_AKTIF,
                r_durak_m=R_DURAK_M,
                turn_aware=TURN_AWARE_AKTIF,
                arac_dingil_m=ARAC_DINGIL_M,
                arac_max_direksiyon_deg=round(math.degrees(ARAC_MAX_DIREKSIYON), 1),
                donus_ceza_agirlik=DONUS_CEZA_AGIRLIK,
                donus_cusp_esik_deg=round(math.degrees(DONUS_CUSP_ESIK), 1),
                hedef_komut=HEDEF_KOMUT_AKTIF,
                blok_ttl_s=BLOK_TTL_S,
                konum_degisim_esik_m=KONUM_DEGISIM_ESIK_M,
                carpan_ters_yon=round(ceza_carpani(CEZA_TERS_YON), 3),
                slalom_enjeksiyon=SLALOM_ENJEKSIYON_AKTIF,
                ceza_ters_cikis=CEZA_TERS_CIKIS,
                carpan_ters_cikis=round(ceza_carpani(CEZA_TERS_CIKIS), 3),
                ceza_ters_kalma=CEZA_TERS_KALMA,
                blok_ek_ceza=BLOK_EK_CEZA,
                slalom_enjeksiyon_r=SLALOM_ENJEKSIYON_R,
                kenar_guvenli_m=KENAR_GUVENLI_M,
                ceza_siyirma_m=CEZA_SIYIRMA_M,
                blok_sert_aktif=BLOK_SERT_AKTIF,
                blok_yaricap_m=BLOK_YARICAP_M,
                hesap_kilidi_aktif=HESAP_KILIDI_AKTIF,
                durma_bekleme_sn=DURMA_BEKLEME_SN,
                durma_hiz_esik_ms=DURMA_HIZ_ESIK_MS,
                kilit_cooldown_sn=KILIT_COOLDOWN_SN,
                serit_marjin_m=SERIT_MARJIN_M,
                b_plan_yaw_rota=B_PLAN_YAW_ROTA_AKTIF,
                b_plan_offset_m=B_PLAN_OFFSET_M,
                b_plan_offset_marjin_m=B_PLAN_OFFSET_MARJIN_M,
                b_plan_lead_m=B_PLAN_LEAD_M,
            )

        # ── Planner ─────────────────────────────────────────────────
        # ── Planner ─────────────────────────────────────────────────
        self.planner = DLitePlanner()
        self._load_graph_from_import()

        # ── Görselleştirme ───────────────────────────────────────────
        self.visualizer = GraphVisualizer(self)

        # ── ROS bağlantıları ─────────────────────────────────────────
        self.pub_hedef = rospy.Publisher('/hedef', String, queue_size=10)
        self.pub_durum = rospy.Publisher('/hedef_durum', String, queue_size=5)

        # queue_size: /konum ~20Hz, recalc'lı callback bazen 50ms'yi aşabilir →
        # eski konum değersiz, daima en taze kullanılmalı → 1 (sonsuz iç kuyruk
        # birikmesini önler). Komut/durum mesajları seyrek → küçük tampon.
        rospy.Subscriber('/konum',         Pose2D,        self.konum_callback,    queue_size=1)
        rospy.Subscriber('/gorev_durumu',  String,        self.varildi_callback,  queue_size=5)
        if HEDEF_KOMUT_AKTIF:
            rospy.Subscriber('/hedef_komut', String,       self.hedef_komut_callback, queue_size=5)

        self.new_data_available = True
        print(f"{YESIL}>>> SİSTEM HAZIR. Bekleniyor: /konum{SIFIRLA}")

    # ==========================================
    #   GRAF YÜKLEME
    # ==========================================
    def _load_graph_from_import(self) -> None:
        rospy.loginfo("[hedef_yoneticisi] Graf yapısı oluşturuluyor...")
        try:
            G = build_track_graph()
            self.G = G
        except Exception as e:
            rospy.logerr(f"[hedef_yoneticisi] Graf oluşturulamadı: {e}")
            return

        self.planner.adj_list.clear()
        self.planner.pred_list.clear()
        self.planner.edge_weights.clear()
        self._forward_poz = None   # ÖNCE invalidate (aşağıdaki node_types.clear ile yarış penceresini kapat)
        self.planner.node_types.clear()
        self.planner.nodes.clear()
        self.pos_to_node = {}

        node_to_pos = {}
        for node_name, data in G.nodes(data=True):
            pos = data.get('pos')
            if pos is not None:
                node_to_pos[node_name] = (pos[0], pos[1])
                self.planner.node_types[(pos[0], pos[1])] = data.get('type', 'intermediate')
                self.pos_to_node[(pos[0], pos[1])] = node_name

        edge_info: dict[tuple[tuple, tuple], tuple[str, float]] = {}
        for u, v, edge_data in G.edges(data=True):
            p1 = node_to_pos.get(u)
            p2 = node_to_pos.get(v)
            if not p1 or not p2 or p1 == p2:
                continue

            # Base distance
            d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            etype = edge_data.get('type', 'lane')

            # İleri yön ağırlığı — ceza puanı (0-100) → çarpan
            if etype == 'lane':
                w_forward = d * ceza_carpani(CEZA_DUZ_SERIT)
            elif etype == 'connection':
                w_forward = d * ceza_carpani(CEZA_BAGLANTI)
            elif etype == 'slalom':   # şu an ÖLÜ dal: slalom kenarı grafa eklenmiyor
                w_forward = d * ceza_carpani(CEZA_SERIT_DEGISTIRME)   # (dinamik akış gelince anlamlı)
            else:
                w_forward = d

            # Add forward edge
            self.planner.add_edge(p1, p2, w_forward)
            edge_info[(p1, p2)] = (etype, d)

            # NOT: Ana tek-yön loop (A,B,C,...) kenarlarının tersi grafa EKLENMEZ
            # (tek yönlü yapı korunur). Durak çevresi iki yönlü yapımı
            # _iki_yonlu_durak_ekle()'de (geo_targets kurulduktan SONRA) yapılır.

        # edge_info'yu sakla → iki-yönlü durak adımı (goal'lar kurulunca) kullanır
        self._edge_info = edge_info

        # ── Karşı-şerit (slalom) crossing adayları (§16 S-A) ──────────────
        # build_track_graph bunları TABAN grafa eklemez (tek-yön korunur); biz
        # pozisyon-anahtarlı aday liste olarak saklarız. Blok geldiğinde engel
        # çevresindekiler _slalom_enjekte() ile GEÇİCİ eklenir, recalc sonunda
        # geri alınır. (p1, p2, d): yönlü crossing kenarı + ham uzunluk.
        slalom_conns = []
        for (u, v, _app, _ex) in G.graph.get('slalom_connections', []):
            p1 = node_to_pos.get(u)
            p2 = node_to_pos.get(v)
            if not p1 or not p2 or p1 == p2:
                continue
            if (p1, p2) in self.planner.edge_weights:   # taban grafta zaten varsa atla
                continue
            d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            slalom_conns.append((p1, p2, d))
        self._slalom_conns = slalom_conns

        # ── Karşı-şerit boylamasına segment adayları (§16, "ters şeritte KALMA") ──
        # Ardışık eşleşmiş şerit node'ları arası (yalnız düz paralel kesim).
        # Blok geldiğinde EKSİK yönü ucuz (CEZA_TERS_KALMA) enjekte edilir →
        # araç karşı şeritte İLERİ seyredip engeli geçene dek solda kalabilir.
        slalom_segs = []
        for (a, b) in G.graph.get('slalom_lane_segments', []):
            p1 = node_to_pos.get(a)
            p2 = node_to_pos.get(b)
            if not p1 or not p2 or p1 == p2:
                continue
            d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            slalom_segs.append((p1, p2, d))
        self._slalom_segs = slalom_segs
        rospy.loginfo(f"[graph] {len(slalom_conns)} crossing + {len(slalom_segs)} "
                      f"karşı-şerit segment adayı saklandı (blok-tetikli enjeksiyon).")

        # YOL AYRIMI (out-degree ≥ 2 = yolun dallandığı düğüm) pozisyonlarını YÜKLEME
        # Forward(sağ) liste + karşı(sol/overtake) şerit düğüm pozisyon KÜMESİ (subscriber'lar
        # register olmadan, tek-thread doldur). forward: _sag_seritte histerezisi için (min mesafe);
        # karsi (set): hem _sag_seritte hem "committed rota karşı şeride girdi mi" (kilit) için.
        self._karsi_seritler = set(G.graph.get('karsi_seritler', set()))
        self._forward_poz = [node_to_pos[n] for n, data in G.nodes(data=True)
                             if n in node_to_pos and data.get('lane') not in self._karsi_seritler]
        self._karsi_poz = {node_to_pos[n] for n, data in G.nodes(data=True)
                           if n in node_to_pos and data.get('lane') in self._karsi_seritler}

        self._graph_loaded = True
        rospy.loginfo(
            f"[graph] Graf yapısından {len(self.planner.nodes)} düğüm başarıyla yüklendi "
            f"({len(self._karsi_poz)} karşı-şerit düğümü)."
        )

        if not self.geo_targets_built:
            self._build_geo_targets()

    def _build_geo_targets(self) -> None:
        """Her görev koordinatını graph'taki en yakın düğüme snap'ler."""
        self.geo_targets_world.clear()
        for feature in GOREV_GEOJSON['features']:
            coords = feature['geometry']['coordinates']
            tx, ty = coords[0], coords[1]
            nearest = min(
                self.planner.nodes,
                key=lambda n: (n[0] - tx)**2 + (n[1] - ty)**2
            )
            snap = math.hypot(nearest[0] - tx, nearest[1] - ty)
            print(f"{YESIL}>>> [{feature['properties']['name']}] "
                  f"snap:{snap:.2f}m → {nearest}{SIFIRLA}")
            self.geo_targets_world.append(nearest)

        self.geo_targets_built = True

        # ── İki-yönlü durak erişimi (goal'lar artık biliniyor) ──────────────
        if IKI_YONLU_DURAK_AKTIF:
            self._iki_yonlu_durak_ekle()

        # İlk konum zaten geldiyse hemen rota hesapla
        if self._ilk_konum_alindi and self.robot_x is not None:
            self.recalculate_path_from_robot()

    def _iki_yonlu_durak_ekle(self) -> None:
        """Her durak goal'ünün R_DURAK_M yarıçapındaki TÜM tek-yönlü kenarların
        (cep içi lane + giriş/çıkış connection'ları) tersini ekler → durağa her
        iki uçtan girilebilir. Ana tek-yön loop bu yarıçaptan uzakta olduğu için
        dokunulmaz. Cep çıkmaz yapı → iki yönlü olması ana trafiğe yeni geçiş
        rotası açmaz. Kalıcı (graf yüklemede bir kez); duraklar sabit."""
        edge_info = getattr(self, "_edge_info", None)
        if not edge_info or not self.geo_targets_world:
            return
        goals = list(self.geo_targets_world)
        R = R_DURAK_M
        n_eklendi = 0
        for (p1, p2), (et, d) in list(edge_info.items()):
            if (p2, p1) in edge_info:          # zaten çift yönlü → atla
                continue
            yakin = any(math.hypot(p1[0] - gx, p1[1] - gy) <= R and
                        math.hypot(p2[0] - gx, p2[1] - gy) <= R
                        for (gx, gy) in goals)
            if yakin:
                w = d * (ceza_carpani(CEZA_BAGLANTI) if et == 'connection'
                         else ceza_carpani(CEZA_DUZ_SERIT))
                self.planner.add_edge(p2, p1, w)
                n_eklendi += 1
        rospy.loginfo(f"[graph] İki-yönlü durak erişimi: {n_eklendi} ters kenar "
                      f"eklendi (R={R}m, {len(goals)} durak).")

    # ==========================================
    #   CALLBACKLER
    # ==========================================
    def konum_callback(self, msg: Pose2D) -> None:
        raw_x   = msg.x
        raw_y   = msg.y
        raw_yaw = msg.theta

        if raw_x is None or raw_y is None:
            return

        now = time.time()

        if KONUM_FILTRE_AKTIF:
            # 1. Outlier Rejection (GPS Jump Protection)
            if (self._filtered_x is not None 
                    and self._filtered_y is not None 
                    and self._last_konum_t is not None):
                dt = now - self._last_konum_t
                if dt > 0.001:
                    dist = math.hypot(raw_x - self._filtered_x, raw_y - self._filtered_y)
                    speed = dist / dt
                    if speed > KONUM_JUMP_LIMIT_MS:
                        rospy.logwarn_throttle(
                            2.0, f"[filtre] GPS Sıçraması algılandı! "
                                 f"Mesafe: {dist:.2f}m, dt: {dt:.2f}s, Hız: {speed:.1f} m/s (Limit: {KONUM_JUMP_LIMIT_MS}). Girdi reddedildi.")
                        return

            # 2. Low-Pass (EMA) Konum Filtresi
            if self._filtered_x is None or self._filtered_y is None:
                self._filtered_x = raw_x
                self._filtered_y = raw_y
            else:
                alpha = KONUM_FILTRE_ALPHA
                self._filtered_x = alpha * raw_x + (1.0 - alpha) * self._filtered_x
                self._filtered_y = alpha * raw_y + (1.0 - alpha) * self._filtered_y

            # 3. Açısal Yaw Filtresi (Vector-based)
            if raw_yaw is not None:
                if self._filtered_yaw is None:
                    self._filtered_yaw = raw_yaw
                else:
                    alpha = KONUM_FILTRE_ALPHA
                    dx = alpha * math.cos(raw_yaw) + (1.0 - alpha) * math.cos(self._filtered_yaw)
                    dy = alpha * math.sin(raw_yaw) + (1.0 - alpha) * math.sin(self._filtered_yaw)
                    self._filtered_yaw = math.atan2(dy, dx)

            self._last_konum_t = now
            self.robot_x   = self._filtered_x
            self.robot_y   = self._filtered_y
            self.robot_yaw = self._filtered_yaw
        else:
            self._last_konum_t = now
            self.robot_x   = raw_x
            self.robot_y   = raw_y
            self.robot_yaw = raw_yaw

        # ── FIX: İlk konum alındığında cooldown'ları sıfırla ────────
        if not self._ilk_konum_alindi:
            self._ilk_konum_alindi = True
            now = time.time()
            self._son_gorev_zamani    = now
            self._son_varildi_zamani  = now
            self.son_hesaplama_zamani = now
            rospy.loginfo(f"[konum] İlk konum alındı: "
                          f"({self.robot_x:.1f}, {self.robot_y:.1f})")

        # ── Rota yoksa hesapla ───────────────────────────────────────
        if (not self.is_path_calculated
                and self.planner.nodes
                and self.geo_targets_world):
            self.recalculate_path_from_robot(reason="ilk_rota")

        if not self.is_path_calculated or not self.full_path_world:
            self.new_data_available = True
            return

        now = time.time()

        # ── Karar bloğu TTL düşüşü ───────────────────────────────────
        # NOT: Eskiden blok TTL'i/aktif blok sayısı düşünce bir kez reroute
        # ediliyordu (fail-safe). KALDIRILDI (canlı off-road bug,
        # logs/20260624T193310Z): araç sol şeride GEÇERKEN blok düşünce recalc
        # aracı B düğümüne snap'leyip (blok yok → enjeksiyon yok → B tek-yönlü)
        # U-dönüşü/yukarı saçma rota çizip YOLDAN ÇIKARIYORDU. Blok kalkınca
        # yeniden rota çizmiyoruz; araç committed overtake path'ini (B→A→hedef
        # döner) sürer. (Sadece sayacı izlemeye devam — başka tüketici yok.)
        if HEDEF_KOMUT_AKTIF:
            # HESAPLAMA KİLİDİ: burun yol ayrımını geçince kilitle; araç 15s durunca aç
            # (+ bir temiz recalc). try/except: konum_callback'i (üst-düzey guard'sız) çökertmesin.
            try:
                self._kilit_guncelle()
            except Exception as e:  # noqa: BLE001
                rospy.logwarn_throttle(5.0, f"[kilit] güncelleme hatası: {e!r}")

        # ── Otomatik WP geçişi: hafif map-matching ──────────────────
        # FAZ3: tek-tek +1 yerine, aracın rotadaki yerini İLERİ pencerede
        # [idx, idx+MATCH_PENCERE) en yakın noktaya snap'le (geri zıplama yok,
        # pencere ileri başlar). Sadece koridor içindeyse (best_d < MATCH_KORIDOR_M)
        # ilerlet; dışındaysa off-route → snap yapma, sapma/reroute mantığı halleder.
        wp_gecis_log = None   # lock içinde doldurulur, log lock DIŞINDA atılır
        with self._wp_lock:
            n_path = len(self.full_path_world)
            ust    = min(self.current_wp_index + MATCH_PENCERE, n_path)
            best_i = self.current_wp_index
            best_d = math.hypot(self.robot_x - self.full_path_world[best_i][0],
                                self.robot_y - self.full_path_world[best_i][1])
            for i in range(self.current_wp_index + 1, ust):
                wx_wp, wy_wp = self.full_path_world[i]
                d = math.hypot(self.robot_x - wx_wp, self.robot_y - wy_wp)
                if d < best_d:
                    best_d = d
                    best_i = i

            if (best_i > self.current_wp_index
                    and best_i <= n_path - 1
                    and best_d < MATCH_KORIDOR_M):
                atlanan = best_i - self.current_wp_index
                self.current_wp_index = best_i
                self._son_varildi_zamani = now
                rospy.loginfo(f"[OTO] WP {self.current_wp_index} "
                              f"(map-match +{atlanan}, d:{best_d:.1f}m)")
                wp_gecis_log = dict(wp_idx=self.current_wp_index,
                                    n_path=n_path,
                                    dist=round(best_d, 2),
                                    atlanan=atlanan,
                                    task_idx=self.current_task_index)

        # Disk I/O'yu _wp_lock dışında yap (lock contention'ı önler)
        if wp_gecis_log is not None and self.logger is not None:
            self.logger.log_event("wp_gecis", **wp_gecis_log)

        # ── Ana hedef (durak) kontrolü ───────────────────────────────
        if self.current_task_index < len(self.geo_targets_world):
            wx_g, wy_g = self.geo_targets_world[self.current_task_index]
            dist_to_goal = math.hypot(self.robot_x - wx_g, self.robot_y - wy_g)

            if dist_to_goal < GOREV_YAKINLIK_M and now - self._son_gorev_zamani > 5.0:
                self._son_gorev_zamani = now
                self.current_task_index += 1

                if self.current_task_index >= len(self.geo_targets_world):
                    print(f"{YESIL}>>> TÜM GÖREVLER TAMAMLANDI!{SIFIRLA}")
                    # NOT: bu sadece LOG; dur/hold sinyali ayrı iş (control tarafı, bekleyen #1)
                    if self.logger is not None:
                        self.logger.log_event(
                            "tum_gorev_tamam",
                            son_durak=GOREV_GEOJSON['features'][-1]['properties']['name'],
                            robot=[round(self.robot_x, 2), round(self.robot_y, 2)],
                            yaw_deg=(round(math.degrees(self.robot_yaw), 2)
                                     if self.robot_yaw is not None else None))
                    self.is_path_calculated = False
                    self.full_path_world    = []
                    self.new_data_available = True
                    return

                next_name = GOREV_GEOJSON['features'][self.current_task_index]['properties']['name']
                print(f"{YESIL}>>> DURAK TAMAMLANDI! Yeni hedef: {next_name}{SIFIRLA}")
                if self.logger is not None:
                    self.logger.log_event("gorev_tamam", task_idx=self.current_task_index,
                                          next_name=next_name,
                                          robot=[round(self.robot_x, 2), round(self.robot_y, 2)])
                self.recalculate_path_from_robot(reason="durak_tamamlandi")

        # ── Sapma kontrolü ───────────────────────────────────────────
        # FIX: Tüm rota yerine sadece yakındaki WP'lere bak (CPU tasarrufu)
        with self._wp_lock:
            lookahead = self.full_path_world[self.current_wp_index:
                                             self.current_wp_index + 20]
        if lookahead:
            # ── FAZ5: sapmayı aracın BURUN noktasından ölç ──────────────
            # Burun = robot + ILERI_MESAFE_M * yaw yönü (start seçiminde kullanılan
            # nokta). Böylece on-route'ta mesafe ~0 (döngü kendiliğinden kapanır) ve
            # ölçüm yön-bilinçli: araç rotaya dönükse burun rotaya yakın → kopmaz.
            if self.robot_yaw is not None:
                burun_x = self.robot_x + ILERI_MESAFE_M * math.cos(self.robot_yaw)
                burun_y = self.robot_y + ILERI_MESAFE_M * math.sin(self.robot_yaw)
            else:
                burun_x, burun_y = self.robot_x, self.robot_y
            min_dist = min(
                math.hypot(burun_x - wx, burun_y - wy)
                for wx, wy in lookahead
            )
            # ── FAZ2: debounce + histerezis (Schmitt-trigger) ───────────
            # Sapma SAPMA_DEBOUNCE_SURE boyunca sürmeli. Eşik etrafında salınan
            # (flapping) araçta sayaç sıfırlanmasın diye: sayaç SAPMA_ESIK üstünde
            # KURULUR, ancak SAPMA_TEMIZ ALTINA inince SIFIRLANIR. Ara bantta
            # (TEMIZ..ESIK) sayaca dokunulmaz → kenarda süren araç da tetikler.
            if min_dist > SAPMA_ESIK_METRE:
                if self._sapma_baslangic is None:
                    self._sapma_baslangic = now
            elif min_dist < SAPMA_TEMIZ_METRE:
                self._sapma_baslangic = None
            sapma_sureli = (self._sapma_baslangic is not None
                            and (now - self._sapma_baslangic) >= SAPMA_DEBOUNCE_SURE)

            if (sapma_sureli
                    and now - self.son_hesaplama_zamani > 5.0):
                # Sapma → reroute. Mid-manevra (araç sol şeride çıkmışken) cusp/U-dönüşü
                # re-plan'ını artık HESAPLAMA KİLİDİ engelliyor (burun yol ayrımını geçince
                # kilitli; recalc kilitliyse erken döner) → ayrı off-road guard gereksiz.
                sapma_suresi = now - self._sapma_baslangic
                print(f"{SARI}>>> [DİKKAT] Burun rotadan {min_dist:.1f}m uzak "
                      f"({sapma_suresi:.1f}s süregeldi)! Güncelleniyor...{SIFIRLA}")
                if self.logger is not None:
                    self.logger.log_event("sapma", min_dist=round(min_dist, 2),
                                          esik=SAPMA_ESIK_METRE,
                                          burun=[round(burun_x, 2), round(burun_y, 2)],
                                          robot=[round(self.robot_x, 2), round(self.robot_y, 2)],
                                          yaw_deg=round(math.degrees(self.robot_yaw), 2)
                                          if self.robot_yaw is not None else None,
                                          wp_idx=self.current_wp_index,
                                          task_idx=self.current_task_index)
                self.son_hesaplama_zamani = now
                self._sapma_baslangic = None   # FAZ2: reroute sonrası debounce sıfırla
                self.recalculate_path_from_robot(reason="sapma")

        # ── Konum izi (kısılmış; pose.csv) ──────────────────────────
        # pose_due(): throttle hint — mesafe hesapları sadece yazılacaksa
        # yapılır (kısılan tick'lerde boşa hesap yok). d_wp/d_goal o anki
        # current_wp_index/task ile tutarlı kalsın diye burada hesaplanır.
        if self.logger is not None and self.logger.pose_due():
            d_wp = d_goal = None
            try:
                nx_idx = min(self.current_wp_index + 1, len(self.full_path_world) - 1)
                wxn, wyn = self.full_path_world[nx_idx]
                d_wp = math.hypot(self.robot_x - wxn, self.robot_y - wyn)
                if self.current_task_index < len(self.geo_targets_world):
                    gxn, gyn = self.geo_targets_world[self.current_task_index]
                    d_goal = math.hypot(self.robot_x - gxn, self.robot_y - gyn)
            except Exception:  # noqa: BLE001
                pass
            self.logger.log_pose(
                self.robot_x, self.robot_y, self.robot_yaw,
                self.current_task_index, self.current_wp_index,
                len(self.full_path_world), d_wp, d_goal,
            )

        self.new_data_available = True

    def varildi_callback(self, msg: String) -> None:
        """
        /gorev_durumu 'varildi' gelince WP'yi ilerlet.
        FIX: mesaj içeriği kontrol ediliyor + mutex ile konum_callback çakışması önleniyor.
        """
        if msg.data.strip().lower() != 'varildi':
            return

        now = time.time()
        if now - self._son_varildi_zamani < 0.5:
            return

        if not self.is_path_calculated or not self.full_path_world:
            return

        with self._wp_lock:
            wp1_idx = min(self.current_wp_index + 1, len(self.full_path_world) - 1)
            if wp1_idx < len(self.full_path_world) - 1:
                self.current_wp_index = wp1_idx
                self._son_varildi_zamani = now
                rospy.loginfo(f"[varildi] WP → {self.current_wp_index}")

    def hedef_komut_callback(self, msg: String) -> None:
        """karar → hedef komutu (/hedef_komut). String: "komut;taraf;x;y;etiket;yaricap".
        sollama/kenar_blok → engeli blokla (ağırlık şişirme; recalc'ta uygulanır);
        kenar_serbest → bloğu kaldır; replan → yalnız yeniden hesapla.
        Aynı engelin sollama tazelemeleri (≈1s) sadece TTL'i günceller (reroute yok);
        rota yalnız blok KÜMESİ değişince yeniden çizilir."""
        try:
            parcalar = msg.data.strip().split(';')
            if not parcalar or not parcalar[0]:
                return
            komut = parcalar[0].strip().lower()
            taraf = parcalar[1].strip().lower() if len(parcalar) > 1 else ""

            def _f(i):
                try:
                    return float(parcalar[i])
                except (IndexError, ValueError):
                    return None

            now = time.time()
            kume_degisti = False
            # reroute YALNIZ blok EKLENİNCE (yeni engel) veya replan'da yapılır.
            # Blok KALDIRMA (kenar_serbest) reroute TETİKLEMEZ — KRİTİK (canlı bug,
            # logs/20260624T193310Z): araç sol şeride GEÇERKEN karar kenar_serbest
            # yollayınca recalc, aracı (blok artık yok → enjeksiyon yok) B düğümüne
            # snap'liyordu; B tek-yönlü olduğundan planlayıcı U-dönüşü/yukarı saçma
            # rota çizip aracı YOLDAN ÇIKARIYORDU. Çözüm: blok kalkınca yeniden rota
            # çizme; araç zaten committed overtake path'ini (B→A→hedef döner) sürsün.
            reroute_iste = False
            yeni_engel = False   # GERÇEK yeni koni (append) — kilit_bypass için (deadlock fix)

            if komut in ('sollama', 'kenar_blok'):
                ox, oy = _f(2), _f(3)
                r = _f(5)
                if ox is None or oy is None:
                    return
                if r is None or r <= 0.0:
                    r = 1.0
                with self._komut_lock:
                    mevcut = self._yakin_blok_bul(ox, oy)
                    if mevcut is not None:
                        # KONUM güncelle (her zaman en tazeye) — eski/gürültülü konumda
                        # donup kalma. Konum yeterince TAŞINDIYSA reroute de tetikle
                        # (kullanıcı: "duba konumu tetiklesin güncellemeyi").
                        tasindi = (math.hypot(mevcut['x'] - ox, mevcut['y'] - oy)
                                   > KONUM_DEGISIM_ESIK_M)
                        mevcut['x'], mevcut['y'], mevcut['r'], mevcut['t'] = ox, oy, r, now
                        if tasindi:
                            kume_degisti = True
                            reroute_iste = True    # konum değişti → hemen yeniden planla
                    else:
                        self._bloklu_engeller.append(
                            {'x': ox, 'y': oy, 'r': r, 'taraf': taraf, 't': now})
                        kume_degisti = True
                        reroute_iste = True        # YENİ engel → bloklu reroute (recalc kilidi izin verirse)
                        yeni_engel = True          # → kilit_bypass: karşı şeritte kilitliyse bile recalc çalışsın
            elif komut == 'kenar_serbest':
                ox, oy = _f(2), _f(3)
                with self._komut_lock:
                    mevcut = (self._yakin_blok_bul(ox, oy)
                              if ox is not None and oy is not None else None)
                    if mevcut is not None:
                        self._bloklu_engeller.remove(mevcut)
                        kume_degisti = True
                    elif self._bloklu_engeller:
                        # Eşleşmeyen serbest TÜM blokları SİLMEZ (çoklu-koni: başka
                        # koninin bloğunu düşürmek rotayı koninin üstünden geçirir).
                        # Bayat blok zaten BLOK_TTL_S ile kendiliğinden düşer.
                        rospy.logwarn_throttle(
                            5.0, f"[hedef_komut] kenar_serbest ({ox},{oy}) hiçbir "
                                 f"blokla eşleşmedi — yok sayıldı (TTL düşürür)")
                # NOT: reroute_iste = False kalır → committed path'i koru (off-road fix)
            elif komut == 'replan':
                kume_degisti = True
                reroute_iste = True
            else:
                rospy.logwarn_throttle(5.0, f"[hedef_komut] bilinmeyen komut: {komut}")
                return

            if self.logger is not None:
                with self._komut_lock:
                    n_blok = len(self._bloklu_engeller)
                self.logger.log_event("hedef_komut", komut=komut, taraf=taraf,
                                      x=_f(2), y=_f(3), yaricap=_f(5),
                                      n_blok=n_blok, kume_degisti=kume_degisti,
                                      reroute=reroute_iste)

            # Rota yalnız YENİ blok/replan'da çizilir (blok kaldırma reroute etmez)
            if reroute_iste and self.is_path_calculated:
                rospy.loginfo(f"[hedef_komut] {komut} → reroute (yeni blok/replan)")
                self.recalculate_path_from_robot(reason=f"komut_{komut}", kilit_bypass=yeni_engel)
        except Exception as e:  # noqa: BLE001 — callback node'u çökertmesin
            rospy.logwarn_throttle(5.0, f"[hedef_komut] işlenemedi: {e!r} (msg={msg.data!r})")

    def _yakin_blok_bul(self, ox, oy, esik=2.0):
        """(ox,oy)'ye `esik` m'den yakın mevcut bloğu döndürür (yoksa None).
        ÇAĞIRAN _komut_lock'u tutmalı."""
        for b in self._bloklu_engeller:
            if math.hypot(b['x'] - ox, b['y'] - oy) <= esik:
                return b
        return None

    def _aktif_bloklar(self):
        """TTL içindeki blokları döndürür; süresi geçenleri listeden ayıklar.
        (karar engeli ~1s'de tazeler; dropout'ta karar obstacle_memory köprüsü tutar.)"""
        now = time.time()
        with self._komut_lock:
            taze = [b for b in self._bloklu_engeller if now - b['t'] <= BLOK_TTL_S]
            if len(taze) != len(self._bloklu_engeller):
                self._bloklu_engeller = taze
            return [(b['x'], b['y'], b['r']) for b in taze]

    def _sag_seritte(self):
        """Araç şu an SAĞ (forward) şeritte mi — karşı/sollama şeridinde DEĞİL mi?
        HİSTEREZİSLİ: forward şerit düğümlerine en yakın mesafe (d_f) karşı şerit
        düğümlerine en yakın mesafeden (d_k) SERIT_MARJIN_M kadar AZsa → sağ (True);
        d_k, d_f'den o kadar azsa → sol (False); ARADA (boundary, yanal salınım) →
        önceki durumu KORU. Yoksa araç şeritler arasında salınınca flicker olup kilidi
        39×/s aç-kapa edip recalc churn'ü yapıyordu (canlı 203548Z). Bilgi yoksa True."""
        if not self._forward_poz or self.robot_x is None or self.robot_y is None:
            return True
        rx, ry = self.robot_x, self.robot_y
        d_f = min((p[0] - rx) ** 2 + (p[1] - ry) ** 2 for p in self._forward_poz) ** 0.5
        d_k = (min((p[0] - rx) ** 2 + (p[1] - ry) ** 2 for p in self._karsi_poz) ** 0.5
               if self._karsi_poz else float('inf'))
        if d_f < d_k - SERIT_MARJIN_M:
            self._son_sag_serit = True
        elif d_k < d_f - SERIT_MARJIN_M:
            self._son_sag_serit = False
        # arada: histerezis → _son_sag_serit değişmez
        return self._son_sag_serit

    def _overtake_rotasi_mi(self, path):
        """Verilen committed path KARŞI (sol/overtake) şeride giriyor mu — yani bu bir
        SOLLAMA rotası mı? (Herhangi bir WP'si karşı-şerit düğüm kümesinde.) recalc'ta
        sollama kararı verildiği AN kilitlemek için kullanılır."""
        if not path or not self._karsi_poz:
            return False
        ks = self._karsi_poz
        return any(wp in ks for wp in path)

    def _kilit_guncelle(self):
        """HESAPLAMA KİLİDİ durumunu günceller (konum_callback'ten her poz'da). Kilit
        recalc'ta SOLLAMA KARARIYLA kurulur (_overtake_rotasi_mi → _hesap_kilitli=True).
        Burada AÇMA yönetilir (kullanıcı 2026-06-27: "sağ şeride döndüğü an bitsin"):
          • FAIL-SAFE (gürültü-dayanıklı ÇAPA, en başta — yutulamaz): kilitliyken araç
            DURMA_YARICAP_M çapasında DURMA_BEKLEME_SN kalırsa (durdu) → AÇ. Canlı
            100603Z deadlock: eski anlık-hız durma tespiti konum gürültüsünde sayacı
            HER örnekte sıfırlayıp 15s fail-safe'i HİÇ ateşletmedi → 222s kilit.
          • ASIL: araç SOL şeride girip SAĞ (forward) şeride DÖNÜNCE → AÇ + temiz recalc
            (sonraki dubayı sağ şeritten planlar). COOLDOWN: churn engeli (203548Z).
            _sag_seritte hesap-yoğun → SADECE o try/except ile korunur (fail-safe DIŞI).
        AÇMA reroute TETİKLER (recalc), kilitlemenin kendisi recalc'ta yapılır."""
        if not HESAP_KILIDI_AKTIF:
            return
        rx, ry = self.robot_x, self.robot_y
        if rx is None or ry is None:
            return
        now = time.time()
        if not self._hesap_kilitli:
            self._durma_capa_xy = None      # kilit yokken çapayı temizle (taze başla)
            return
        # ── FAIL-SAFE (gürültü-dayanıklı ÇAPA, try/except DIŞI — sade math, yutulamaz) ──
        # Araç çapadan DURMA_YARICAP_M'den uzaklaşınca çapa+sayaç yenilenir; o yarıçapta
        # DURMA_BEKLEME_SN kalırsa "durdu" → kilidi aç (kalıcı deadlock'tan çık). Tek
        # gürültülü örnek (jitter < yarıçap) sayacı SIFIRLAMAZ → fail-safe güvenilir ateşler.
        if (self._durma_capa_xy is None
                or math.hypot(rx - self._durma_capa_xy[0],
                              ry - self._durma_capa_xy[1]) > DURMA_YARICAP_M):
            self._durma_capa_xy = (rx, ry)
            self._durma_baslangic = now
        elif now - self._durma_baslangic >= DURMA_BEKLEME_SN:
            self._kilit_ac(now, "kilit_acildi",
                           f"araç {DURMA_BEKLEME_SN:.0f}s durdu (fail-safe)")
            return
        # ── AÇMA asıl: sol şeride girip SAĞ şeride dönünce. _sag_seritte hesap-yoğun →
        #    SADECE o try/except'le korunur. sag=None (True DEĞİL): hata olursa lane
        #    mantığı atlanır + observability "hata" yazar (yanlış sag_serit=True YAZMAZ). ──
        sag = None
        try:
            sag = self._sag_seritte()
        except Exception as e:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, f"[kilit] sağ-şerit hesap hatası: {e!r}")
        if sag is False:
            self._kilit_sol_serite_girdi = True       # sollama ortası: sol şeride girdi
        elif (sag is True and self._kilit_sol_serite_girdi
              and now - self._son_kilit_recalc >= KILIT_COOLDOWN_SN):
            self._kilit_ac(now, "sag_serit", "araç sağ şeride döndü")
            return
        # ── Observability: kilitliyken durum + fail-safe sayacını ~1Hz events'e logla ──
        if now - self._son_kilit_log_t >= 1.0:
            self._son_kilit_log_t = now
            if self.logger is not None:
                try:
                    self.logger.log_event(
                        "kilit_durum",
                        durma_s=round(now - self._durma_baslangic, 1) if self._durma_baslangic else 0.0,
                        sol_girildi=self._kilit_sol_serite_girdi,
                        sag_serit=sag,
                        robot=[round(rx, 2), round(ry, 2)]
                    )
                except Exception:  # noqa: BLE001
                    pass

    def _kilit_ac(self, now, reason, mesaj):
        """Kilidi açar + o stabil konumdan tek temiz recalc (cooldown damgalar)."""
        self._hesap_kilitli = False
        self._kilit_sol_serite_girdi = False
        self._durma_baslangic = None
        self._durma_capa_xy = None
        self._son_kilit_recalc = now
        rospy.loginfo_throttle(2.0, f"[kilit] {mesaj} → hesaplama AÇILDI (temiz recalc)")
        self.recalculate_path_from_robot(reason=reason)

    def _engel_rotada(self, path, engeller):
        """Aktif engellerden HERHANGİ biri verilen (bloksuz baseline) rotanın
        ÜZERİNDE mi — SERT blok çemberi (max(r, BLOK_YARICAP_M)) bir rota segmentini
        kesiyor mu?
          • True  → engel kendi şeridimizi tıkıyor → karşı-şerit enjeksiyonu (sollama) gerek.
          • False → engel karşı şeritte / rota dışı (~2m yanal) → blokla ama SOLLAMA YOK.
        path None/boşsa False (baseline yoksa karar veremeyiz → güvenli: sollama yok)."""
        if not path or len(path) < 2 or not engeller:
            return False
        for (ox, oy, r) in engeller:
            yaricap = max(r, BLOK_YARICAP_M)
            for i in range(len(path) - 1):
                if _nokta_segment_mesafe(ox, oy, path[i][0], path[i][1],
                                         path[i + 1][0], path[i + 1][1]) <= yaricap:
                    return True
        return False

    def _blok_uygula(self, enjekte=True, engeller=None):
        """Aktif blokları grafa uygular + (enjekte=True ise) engel çevresine karşı-şerit
        crossing (ÇIKMA) ve boylamasına segment (KALMA) kenarlarını GEÇİCİ enjekte eder (§16).
        İki blok modu:
          • SERT (BLOK_SERT_AKTIF=True, VARSAYILAN): engel çemberindeki kenarları
            grafdan SİLER → ileri şerit kesilir, planlayıcı karşı şeritten dolanmak
            ZORUNDA (kullanıcı kararı 2026-06-26). Silinen kenarlar `removed_set`'e
            yazılır → enjeksiyon bunları GERİ EKLEMESİN (yoksa ileri şerit yeniden açılır).
          • AĞIRLIK (False): eski sonlu ceza-şişirme (fail-safe ama farklı-sokak riski).
        enjekte=False (SLALOM_YALNIZ_GEREKINCE, engel kendi rotada değil): SADECE
        blok uygulanır, crossing EKLENMEZ → engel karşı şeritte (rota dışı) → araç
        kendi şeridinde devam eder, karşı şeride GEÇMEZ (sollama YOK). Karar recalc'ta
        (_engel_rotada) verilir.
        engeller: çağıran _aktif_bloklar() snapshot'ını verirse onu kullanır; vermezse
        kendi çağırır. Çağıran (recalc) _engel_rotada ile AYNI snapshot'ı geçirmeli →
        D* araması sırasında /hedef_komut yeni blok eklerse enjekte_gerek↔blok seti
        tutarsızlığı (TOCTOU) oluşmaz (incele bulgusu). Döndürür: (saved, eklenen) —
        recalc finally'sinde geri yüklenir/kaldırılır."""
        saved = []
        eklenen = []
        if engeller is None:
            engeller = self._aktif_bloklar()
        if not engeller:
            return saved, eklenen
        # BASE predecessor snapshot — blok kenar SİLMEDEN ÖNCE al. "Bir waypoint
        # önce dön" enjeksiyonu engelin GERÇEK taban öncülünden başlamalı; SERT blok
        # öncülün kenarını sileceği için snapshot sonradan alınsa o öncül boş kalır
        # (predecessor crossing hiç enjekte edilmez). _slalom_enjekte'ye geçirilir.
        pred_snap = {k: list(v) for k, v in self.planner.pred_list.items()}
        if BLOK_SERT_AKTIF:
            saved, removed_set = self._sert_blok(engeller)
        else:
            saved, removed_set = self._agirlik_blok(engeller)
        if not enjekte:
            return saved, eklenen   # engel rota dışı → blokla, crossing enjekte etme (sollama yok)
        # Karşı-şerit enjeksiyonu (yalnız turn-aware açıkken → cusp kapısı sürülemez
        # U-dönüşlerini eler; klasik find_path'te bu güvenlik yok). removed_set:
        # SERT blokla silinmiş kenarları enjeksiyon geri eklemesin.
        if SLALOM_ENJEKSIYON_AKTIF and TURN_AWARE_AKTIF:
            eklenen = self._slalom_enjekte(engeller, removed_set, pred_snap)
        elif BLOK_SERT_AKTIF:
            # SERT blok ileri şeridi keser ama karşı-şerit alternatifi enjekte
            # edilmezse rota çoğu engelde None olur → her seferinde bloksuz fallback
            # (engele doğru düz rota). İstenen davranış DEĞİL → uyar.
            rospy.logwarn_throttle(
                10.0, "[blok] SERT blok açık ama karşı-şerit enjeksiyonu kapalı "
                      "(SLALOM_ENJEKSIYON_AKTIF/TURN_AWARE_AKTIF) → slalom üretilemez, "
                      "rota bloksuz fallback'a düşer.")
        return saved, eklenen

    def _sert_blok(self, engeller):
        """SERT blok (kullanıcı kararı 2026-06-26): her engelin etkin yarıçapındaki
        (`max(engel_r, BLOK_YARICAP_M)` ≈ 1m çember) TÜM kenarları grafdan GEÇİCİ
        SİLER → o çemberdeki waypoint'ler kullanılamaz, ileri şerit engelde KESİLİR.
        Böylece planlayıcı engeli yalnız karşı-şerit crossing'leriyle (ceza puanı
        altında) dolanabilir → dar/yerel slalom; ağırlık şişirme gibi 'alternatif
        yoksa düz geç' DEĞİL. Döndürür: (saved, removed_set); saved=[((p1,p2),w)]
        geri yükleme için, removed_set=silinen kenar kümesi (enjeksiyon kullanır).
        Geri alma _blok_geri_al'da (add_edge ile re-add)."""
        saved = []
        removed_set = set()
        for (p1, p2), w in list(self.planner.edge_weights.items()):
            for (ox, oy, r) in engeller:
                yaricap = max(r, BLOK_YARICAP_M)
                if _nokta_segment_mesafe(ox, oy, p1[0], p1[1], p2[0], p2[1]) <= yaricap:
                    saved.append(((p1, p2), w))
                    removed_set.add((p1, p2))
                    self.planner.remove_edge_directed(p1, p2)
                    self.planner.edge_weights.pop((p1, p2), None)
                    break
        return saved, removed_set

    def _agirlik_blok(self, engeller):
        """ESKİ ağırlık-şişirme bloğu (BLOK_SERT_AKTIF=False ile seçilir): engel
        yarıçapı + BLOK_MARJIN_M içindeki kenarlara çarpımsal (CEZA_TERS_YON) +
        toplamsal (BLOK_EK_CEZA) ceza → cone 'neredeyse geçilmez' ama SONLU
        (alternatif yoksa yine geçer = fail-safe). SERT blok bu fail-safe'i
        kaldırdığı (ve recalc path=None'da bloksuz fallback yaptığı) için artık
        varsayılan değil; çıktı kıyası/acil durumlar için korunuyor.
        Döndürür: (saved, set()) — _sert_blok ile aynı API (removed_set boş; ağırlık
        modunda kenar silinmez, yalnız ağırlık şişer). ÖNEMLİ: enjeksiyondan ÖNCE
        çağrılır → enjekte crossing'ler şişirme döngüsüne girmez (çift ceza yok)."""
        saved = []
        carp = ceza_carpani(CEZA_TERS_YON)
        for (p1, p2), w in list(self.planner.edge_weights.items()):
            for (ox, oy, r) in engeller:
                if _nokta_segment_mesafe(ox, oy, p1[0], p1[1], p2[0], p2[1]) <= r + BLOK_MARJIN_M:
                    saved.append(((p1, p2), w))
                    self.planner.edge_weights[(p1, p2)] = w * carp + BLOK_EK_CEZA
                    break
        return saved, set()

    def _slalom_enjekte(self, engeller, removed_set=None, pred_snap=None):
        """Aktif blokların SLALOM_ENJEKSIYON_R yakınına GEÇİCİ karşı-şerit kenarı ekler:
          • crossing (A↔B geçiş, "ÇIKMA")  → CEZA_TERS_CIKIS (PAHALI: az geçiş yap)
          • boylamasına segment EKSİK yönü (karşı şeritte ileri seyir, "KALMA")
                                           → CEZA_TERS_KALMA (UCUZ: bir kez çıkınca solda kal)
        Bu ayrım (kullanıcı içgörüsü) → tek giriş+çıkış + engel boyunca karşı şeritte
        kalma (yanal açıklık), zig-zag yerine. Döndürdüğü [(p1,p2),...] _blok_geri_al'da
        kaldırılır. Yalnız engel çevresinde + tek-yön taban grafı bozmadan.
        removed_set: SERT blokla silinmiş kenarlar — bunları enjeksiyon GERİ EKLEMEZ
        (yoksa engelde kesilen ileri şerit yeniden açılır)."""
        eklenen = []
        removed_set = removed_set or set()
        R = SLALOM_ENJEKSIYON_R

        def _engele_yakin(p1, p2):
            for (ox, oy, _r) in engeller:
                if (math.hypot(p1[0] - ox, p1[1] - oy) <= R or
                        math.hypot(p2[0] - ox, p2[1] - oy) <= R):
                    return True
            return False

        # 1) Crossing'ler (ÇIKMA — pahalı) + SIYIRMA cezası + predecessor (bir
        #    waypoint önce dön) enjeksiyonu. Cone'a yakın geçen giriş crossing'i
        #    cezalanır; bir önceki düğümden başlayan (engeli daha geniş geçen)
        #    versiyon da eklenir → planlayıcı erken-dönen, engel düğümünü atlayan
        #    girişi seçer (path engel düğümünden geçmez → karar engel_blokaj vermez).
        carp_c = ceza_carpani(CEZA_TERS_CIKIS)

        def _min_acik(a, b):
            """crossing [a,b] segmentinin en yakın aktif bloğa (cone) açıklığı (m)."""
            return min(_nokta_segment_mesafe(ox, oy, a[0], a[1], b[0], b[1])
                       for (ox, oy, _r) in engeller)

        def _siyirma_cezasi(a, b):
            """Cone'a r+KENAR_GUVENLI_M'den yakın geçen crossing'e yakınlıkla
            orantılı toplamsal ceza (geniş geçen tercih edilsin)."""
            ek = 0.0
            for (ox, oy, r) in engeller:
                acik = _nokta_segment_mesafe(ox, oy, a[0], a[1], b[0], b[1])
                ek = max(ek, CEZA_SIYIRMA_M * max(0.0, (r + KENAR_GUVENLI_M) - acik))
            return ek

        def _enjekte_crossing(a, b):
            if (a, b) in self.planner.edge_weights:   # zaten varsa dokunma
                return
            if (a, b) in removed_set:                  # SERT blokla silindi → geri ekleme
                return
            d_ab = math.hypot(b[0] - a[0], b[1] - a[1])
            self.planner.add_edge(a, b, d_ab * carp_c + _siyirma_cezasi(a, b))
            eklenen.append((a, b))

        # BASE predecessor snapshot — çağıran (_blok_uygula) blok kenar SİLMEDEN
        # ÖNCE çekip geçirir → "bir waypoint önce" enjeksiyonu engelin GERÇEK taban
        # öncülünden başlar (SERT blok öncül kenarını silmiş olsa bile). Geçirilmezse
        # (ör. doğrudan çağrı) anlık pred_list'ten al.
        if pred_snap is None:
            pred_snap = {k: list(v) for k, v in self.planner.pred_list.items()}
        esik_acik = min(r + KENAR_GUVENLI_M for (_x, _y, r) in engeller)
        for (p1, p2, d) in self._slalom_conns:
            if not _engele_yakin(p1, p2):
                continue
            _enjekte_crossing(p1, p2)
            # Sıyırıyorsa: bir önceki düğümden başlayan (daha geniş geçen) crossing'i
            # de ekle → "bir waypoint önce dön". p1'in BASE öncüllerinden cone'u
            # DAHA GENİŞ geçenleri enjekte et (planlayıcı en geniş geçeni seçer).
            acik_p1p2 = _min_acik(p1, p2)
            if acik_p1p2 < esik_acik:
                for pu in pred_snap.get(p1, []):
                    if pu == p2 or pu == p1:
                        continue
                    if _min_acik(pu, p2) > acik_p1p2:   # yalnız daha geniş geçeni
                        _enjekte_crossing(pu, p2)

        # 2) Karşı-şerit boylamasına segmentler (KALMA — ucuz); EKSİK yönü ekle →
        #    şerit yerel olarak çift-yönlü olur, araç karşı şeritte ileri seyreder.
        #    (ters yön sürülemez bir U-dönüşü gerektirirse turn-aware cusp eler.)
        carp_k = ceza_carpani(CEZA_TERS_KALMA)
        for (p1, p2, d) in self._slalom_segs:
            if not (_engele_yakin(p1, p2)):
                continue
            for (a, b) in ((p1, p2), (p2, p1)):
                if (a, b) in self.planner.edge_weights:
                    continue
                if (a, b) in removed_set:        # SERT blokla silindi → geri ekleme
                    continue
                self.planner.add_edge(a, b, d * carp_k)
                eklenen.append((a, b))
        return eklenen

    def _blok_geri_al(self, saved, eklenen=None):
        # saved=[((p1,p2), w)]: hem SERT blokla SİLİNEN taban kenarları (re-add)
        # hem AĞIRLIK modunda şişirilen kenarları (weight restore) geri yükler.
        # add_edge ikisini de doğru yapar: kenar yoksa adj/pred/weight ekler, varsa
        # yalnız ağırlığı w'ye geri çeker (mükerrer komşu eklemez → sızıntı yok).
        for (e, w) in saved:
            self.planner.add_edge(e[0], e[1], w)
        # Enjekte edilen karşı-şerit kenarlarını (crossing + segment) TAMAMEN kaldır
        # (adj_list + pred_list + edge_weights) → sızıntı yok. Düğümlere
        # dokunma (her iki uç da mevcut şerit düğümü). saved (taban kenar) ile
        # eklenen (enjekte kenar) ayrık olduğundan re-add/remove çakışmaz.
        for (p1, p2) in (eklenen or []):
            self.planner.remove_edge_directed(p1, p2)
            self.planner.edge_weights.pop((p1, p2), None)

    # ==========================================
    #   B PLANI — yaw-tabanlı sentetik slalom (varsayılan KAPALI)
    # ==========================================
    def _b_plan_yaw_rota(self, goal_node, engeller):
        """B PLANI (varsayılan KAPALI — B_PLAN_YAW_ROTA_AKTIF). GRAF-BAĞIMSIZ,
        yaw-tabanlı sentetik slalom: önümüzdeki engele aracın YAW'ına dik (sol =
        karşı şerit) yanal-offset'li üç waypoint (giriş/orta/çıkış) üretir, sonra
        çıkış noktasına en yakın graf düğümünden hedefe NORMAL rotayla devam eder.
        Başlangıç noktası aracın GERÇEK konumu (robot_x/y); start_node kullanılmaz.
        Açık-döngü; A planı (SERT blok + enjeksiyon) yeterken kullanılmaz. Dönüş:
        [(x,y),...] veya None (üretilemezse çağıran A planına düşer).

        ⚠ AÇILIRSA bilinen sınırlar (B planı KAPALI olduğundan şimdilik etkisiz):
          • Sentetik waypoint'ler (p_in/p_mid/p_out) graf-bağımsız → HARİTA/pist
            sınırını görmez; engel virajdaysa offset duvar/pist-dışına işaret
            edebilir. Açmadan önce saha/sim ile doğrula.
          • Devam rotası (`rejoin→goal`) blok UYGULAMADAN hesaplanır; rejoin engelin
            ÖTESİNDE seçildiği için normalde engele dönmez, ama garanti değil.
          • Graf okumaları (_rota_ara + nodes) _graf_lock altında → eşzamanlı
            A-planı blok mutasyonuyla yarış yok.

        Not: 'sol' tarafı sabit (yaw+90°). Gerekirse karar `taraf` bilgisinden
        türetilebilir; şimdilik tek-yön loop'ta karşı şerit hep solda."""
        if self.robot_x is None or self.robot_yaw is None or not engeller:
            return None
        rx, ry, yaw = self.robot_x, self.robot_y, self.robot_yaw
        fwd_x, fwd_y = math.cos(yaw), math.sin(yaw)
        # Aracın ÖNÜNDEki (ileri-projeksiyon pozitif) en yakın engel
        onde = [(ox, oy, r) for (ox, oy, r) in engeller
                if (ox - rx) * fwd_x + (oy - ry) * fwd_y > 0.0]
        if not onde:
            return None
        ox, oy, r = min(onde, key=lambda o: (o[0] - rx) * fwd_x + (o[1] - ry) * fwd_y)
        # Sol birim vektör (yaw + 90°) = karşı şerit yönü
        lat_x, lat_y = math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2)
        off = max(r + B_PLAN_OFFSET_MARJIN_M, B_PLAN_OFFSET_M)
        lead = B_PLAN_LEAD_M
        p_in  = (ox - lead * fwd_x + off * lat_x, oy - lead * fwd_y + off * lat_y)
        p_mid = (ox + off * lat_x, oy + off * lat_y)
        p_out = (ox + lead * fwd_x + off * lat_x, oy + lead * fwd_y + off * lat_y)
        # Çıkıştan sonra grafa geri dön: p_out'a en yakın düğümden hedefe rota.
        # Graf okuması _graf_lock altında (eşzamanlı A-planı blok mutasyonuna karşı).
        with self._graf_lock:
            rejoin = min(self.planner.nodes,
                         key=lambda n: (n[0] - p_out[0]) ** 2 + (n[1] - p_out[1]) ** 2)
            graf = self._rota_ara(rejoin, goal_node, yaw)
        if not graf:
            return None
        return [(rx, ry), p_in, p_mid, p_out] + list(graf)

    # ==========================================
    #   ROTA HESAPLAMA
    # ==========================================
    def _rota_ara(self, start_node, goal_node, yaw0):
        """Rota arama: turn-aware (bisiklet modeli) veya klasik forward-filtre.
        Karar blokları (ağırlık şişirme) çağırandan ÖNCE uygulanıp sonra geri
        alınır; bu fonksiyon güncel edge_weights üzerinden arar."""
        
        # ── YOLCU_ALMA_ETABI: Karşı şerit sınırlandırması ──
        yolcu_etabi_ve_sagda = (SECILEN_SENARYO == "YOLCU_ALMA_ETABI" and self._sag_seritte())
        saved_karsi_weights = []
        if yolcu_etabi_ve_sagda:
            for (p1, p2), w in list(self.planner.edge_weights.items()):
                is_p1_karsi = p1 in self._karsi_poz
                is_p2_karsi = p2 in self._karsi_poz
                if not is_p1_karsi and is_p2_karsi:
                    # Sağ şeritten karşı şeride geçiş (crossing)
                    saved_karsi_weights.append(((p1, p2), w))
                    self.planner.edge_weights[(p1, p2)] = w + CEZA_KARSIN_GECIS
                elif is_p1_karsi and is_p2_karsi:
                    # Karşı şerit içinde seyir
                    saved_karsi_weights.append(((p1, p2), w))
                    self.planner.edge_weights[(p1, p2)] = w + CEZA_KARSIN_SEYIR

        removed_fwd, removed_back = [], []
        try:
            if TURN_AWARE_AKTIF:
                # Turn-aware arama: yönlü-kenar durumları üzerinde, her köşede
                # δ=atan(L·Δθ/d) direksiyon cezasıyla. Başlangıç yaw'ından itibaren
                # keskin dönüş/cusp pahalı → start'tan geri-yön çıkışı zaten cezalı;
                # ayrı forward-filtre GEREKMEZ. İki-yönlü durakta yumuşak girişi seçer.
                return self.planner.find_path_turn_aware(start_node, goal_node, yaw0)

            # ── Klasik: ileri yönlü filtre + D* find_path (geçici kenar silme) ──
            if self.robot_yaw is not None:
                neighbors = list(self.planner.adj_list.get(start_node, []))
                candidates = []
                for n in neighbors:
                    dx, dy = n[0] - start_node[0], n[1] - start_node[1]
                    if dx == 0 and dy == 0:
                        continue
                    diff = (math.atan2(dy, dx) - self.robot_yaw + math.pi) \
                           % (2 * math.pi) - math.pi
                    if abs(diff) > YON_FILTRE_ACIISI:
                        candidates.append(n)
                if len(candidates) < len(neighbors):
                    for n in candidates:
                        if self.planner.remove_edge_directed(start_node, n):
                            removed_fwd.append(n)
                        if self.planner.remove_edge_directed(n, start_node):
                            removed_back.append(n)
            return self.planner.find_path(start_node, goal_node)
        finally:
            # Klasik yön filtresi kenarlarını geri yükle
            for n in removed_fwd:
                self.planner.restore_edge_directed(start_node, n)
            for n in removed_back:
                self.planner.restore_edge_directed(n, start_node)
            # Karşı şerit cezalarını geri yükle
            for (p1, p2), w in saved_karsi_weights:
                self.planner.edge_weights[(p1, p2)] = w

    def recalculate_path_from_robot(self, reason: str = "?",
                                    kilit_bypass: bool = False) -> None:
        # ── HESAPLAMA KİLİDİ (yol ayrımı + sağ-şerit/15s açma) ──
        # Burun bir yol ayrımını geçtiyse (_hesap_kilitli) ve elde committed path VARSA →
        # YENİDEN HESAPLAMA. Mid-manevra (sol şeritte) cusp/U-dönüşü re-plan'ı biter.
        # Kilit, araç SAĞ ŞERİDE dönünce ya da 15s durunca açılır; _kilit_guncelle o an
        # _hesap_kilitli'yi False yapıp recalc'ı çağırdığı için bu kapı geçilir.
        # İlk rota (committed yok) daima geçer (araç başlasın).
        # İSTİSNA — kilit_bypass (YENİ ENGEL): araç karşı şeritte kilitliyken YENİ bir
        # koni (kume_degisti append) belirirse reroute ÇALIŞMALI.
        if HESAP_KILIDI_AKTIF and self._hesap_kilitli and self.full_path_world:
            now = time.time()
            if kilit_bypass and now - self._son_bypass_t >= KILIT_BYPASS_COOLDOWN_SN:
                self._son_bypass_t = now
                rospy.loginfo(f"[kilit] KİLİT BYPASS (yeni engel) → recalc çalıştırılıyor ({reason})")
            else:
                rospy.loginfo_throttle(
                    2.0, f"[kilit] hesaplama kilitli (sol şerit / yol ayrımı) → recalc atlandı "
                         f"({reason}); sağ şerit veya 15s durma açar")
                return

        if (SADECE_ENGELDE_YENIDEN_PLANLA 
                and self.full_path_world 
                and (time.time() - self._path_creation_time > 3.0)
                and reason not in ("ilk_rota", "durak_tamamlandi") 
                and not reason.startswith("komut_")):
            rospy.loginfo_throttle(
                5.0, f"[kilit] SADECE_ENGELDE_YENIDEN_PLANLA aktif (ilk rotadan >3sn geçti) → "
                     f"recalc atlandı (neden: {reason})")
            return

        if not self.geo_targets_world or not self.planner.nodes:
            rospy.logwarn("[recalculate] geo_targets veya planner.nodes boş!")
            return

        if self.current_task_index >= len(self.geo_targets_world):
            print(f"{YESIL}>>> TÜM GÖREVLER BİTTİ!{SIFIRLA}")
            self.full_path_world = []
            return

        if self.robot_x is None or self.robot_y is None:
            rospy.logwarn("[recalculate] Robot konumu henüz yok!")
            return

        rx, ry = self.robot_x, self.robot_y

        # ── Yaw forward-projection start seçimi (Samed'in eski sürümünden) ──
        # Aracın ÖNÜNDE (yaw yönünde) ileride sanal bir nokta hesapla, start
        # düğümünü O noktaya en yakın düğüm yap. Böylece rota aracın BAKTIĞI yöne
        # göre başlar (yaw'a göre döner) — salt mesafe değil.
        # BLOK aktifken ileri-projeksiyon 0'a iner: 2m ileri-okuma, aracın o anki
        # konumundaki karşı-şerit GİRİŞ crossing'ini atlayıp girişi engele
        # yaklaştırıyordu (yanal açıklık engelin ilerisinde oluşuyordu, engel geç
        # algılanırsa çarpma). Blokta lead-in kritik → path gerçek konumdan başlasın.
        _bloklu = bool(HEDEF_KOMUT_AKTIF and self._aktif_bloklar())
        _ileri = ILERI_MESAFE_BLOK_M if _bloklu else ILERI_MESAFE_M
        if self.robot_yaw is not None:
            front_x = rx + _ileri * math.cos(self.robot_yaw)
            front_y = ry + _ileri * math.sin(self.robot_yaw)
        else:
            front_x, front_y = rx, ry

        # Heading-aware snapping cost: Mesafe + Açısal Uyum (dot product benzeri ceza)
        def snap_cost(n):
            nx, ny = n
            d2 = (nx - front_x) ** 2 + (ny - front_y) ** 2
            if self.robot_yaw is None:
                return d2
            neighbors = self.planner.adj_list.get(n, [])
            if not neighbors:
                return d2 + SNAP_YAW_AGIRLIK * math.pi
            diffs = []
            for nbr in neighbors:
                edge_yaw = math.atan2(nbr[1] - ny, nbr[0] - nx)
                diff = abs((edge_yaw - self.robot_yaw + math.pi) % (2 * math.pi) - math.pi)
                diffs.append(diff)
            return d2 + SNAP_YAW_AGIRLIK * min(diffs)

        start_node = min(self.planner.nodes, key=snap_cost)

        goal_node = self.geo_targets_world[self.current_task_index]
        rospy.loginfo(f"[recalculate] {start_node} → {goal_node}")

        yaw0 = self.robot_yaw if self.robot_yaw is not None else 0.0

        # ── B PLANI (varsayılan KAPALI): yaw-tabanlı sentetik slalom rotası ──
        # Slalom noktasına gelince karar engeli bildirince (aktif blok), graf-bağımsız
        # olarak aracın YAW'ına göre karşı şeride sentetik offset waypoint'leri üretip
        # engeli sollar. B_PLAN_YAW_ROTA_AKTIF=True ile açılır (A planı sahada
        # yetersiz kalırsa). KAPALIYKEN bu blok atlanır; aşağıdaki A planı çalışır.
        if B_PLAN_YAW_ROTA_AKTIF and HEDEF_KOMUT_AKTIF:
            engeller_b = self._aktif_bloklar()
            if engeller_b:
                b_path = self._b_plan_yaw_rota(goal_node, engeller_b)
                if b_path:
                    with self._wp_lock:
                        self.full_path_world    = b_path
                        self.current_wp_index   = 0
                        self.is_path_calculated = True
                    if self.logger is not None:
                        try:
                            self.logger.log_event(
                                "b_plan_yaw_rota", reason=reason, n_wp=len(b_path),
                                robot=[round(rx, 2), round(ry, 2)],
                                yaw_deg=round(math.degrees(yaw0), 2))
                        except Exception:  # noqa: BLE001
                            pass
                    print(f"{YESIL}>>> [B-PLAN] {len(b_path)} WP "
                          f"(yaw-tabanlı sentetik slalom).{SIFIRLA}")
                    return

        # ── Rota arama (A planı). Karar blokları: SERT blok (varsayılan) veya
        #    ağırlık-şişirme + karşı-şerit crossing enjeksiyonu; finally'de geri
        #    al/kaldır. _graf_lock graf mutasyonunu serialize eder (eşzamanlı
        #    recalc'lar ağırlık sızdırmasın). ──
        fallback_bloksuz = False
        with self._graf_lock:
            # SLALOM_YALNIZ_GEREKINCE (kullanıcı 2026-06-26 "duba sol şeritte → hep
            # sollamaya çalışıyorsun"): karşı-şerit crossing'i YALNIZ engel KENDİ
            # ROTAMIZI tıkıyorsa enjekte et. Önce engeli hesaba katmadan bloksuz
            # baseline rota çıkar; engelin çemberi bu rotayı KESMİYORSA (karşı şeritte,
            # ~2m yanal) sollama gereksiz → enjekte ETME (blokla, kendi şeritte devam).
            # KESİYORSA (kendi şeridimizde, ~0m) enjekte → karşı şeritten solla.
            # (Not: graf bağlı olduğundan kendi şerit silinse bile "farklı sokak" rotası
            # bulunur, path None olmaz → "None olunca enjekte et" yaklaşımı yetmez;
            # bu yüzden engel-rotada testi yapılır.)
            enjekte_gerek = True
            path_base = None
            # engeller_aktif TEK snapshot — _engel_rotada + _blok_uygula AYNI kümeyi
            # kullansın (TOCTOU: D* araması sırasında yeni blok eklenirse tutarsızlık).
            engeller_aktif = self._aktif_bloklar() if HEDEF_KOMUT_AKTIF else []
            if engeller_aktif and SLALOM_YALNIZ_GEREKINCE and SLALOM_ENJEKSIYON_AKTIF:
                path_base = self._rota_ara(start_node, goal_node, yaw0)
                enjekte_gerek = self._engel_rotada(path_base, engeller_aktif)
                if not enjekte_gerek:
                    rospy.loginfo_throttle(
                        2.0, "[slalom] engel kendi rotada DEĞİL (karşı şeritte) → "
                             "sollama YOK, yalnız blokla + kendi şeritte devam")

            blok_saved, blok_eklenen = ([], [])
            if path_base is not None and not enjekte_gerek:
                # Engel rota dışı (karşı şerit): SERT blok çemberi = _engel_rotada eşiği
                # olduğundan blok baseline rotayı DEĞİŞTİRMEZ → baseline'ı doğrudan kullan
                # (2. D* aramasını ATLA; engelin WP'si zaten rota dışı, blokla gerek yok).
                path = path_base
            else:
                blok_saved, blok_eklenen = (
                    self._blok_uygula(enjekte=enjekte_gerek, engeller=engeller_aktif)
                    if HEDEF_KOMUT_AKTIF else ([], []))
                if blok_eklenen:
                    rospy.loginfo_throttle(
                        2.0, f"[slalom] {len(blok_eklenen)} karşı-şerit crossing enjekte "
                             f"(engel kendi rotayı tıkıyor) → karşı şeritten reroute")
                try:
                    path = self._rota_ara(start_node, goal_node, yaw0)
                finally:
                    self._blok_geri_al(blok_saved, blok_eklenen)

            # ── Fail-safe: SERT blokla yol YOK (karşı şerit yok / giriş kapalı) ──
            # FARKLI BİR SOKAĞA SAPMA. Blok geri alındıktan SONRA (graf temizken)
            # bloksuz düz rotayı ver; araç engele yaklaşınca control latched e-stop
            # ile durur (plan §12.13). Wrong-street'ten (yarış cezası) iyidir.
            if path is None and blok_saved:
                rospy.logwarn_throttle(
                    2.0, "[blok] bloklu rota YOK → bloksuz düz rota (control e-stop); "
                         "farklı sokağa sapılmaz")
                path = self._rota_ara(start_node, goal_node, yaw0)
                fallback_bloksuz = True
            elif (path is None and not blok_saved and HEDEF_KOMUT_AKTIF
                  and BLOK_SERT_AKTIF and self._aktif_bloklar()):
                # Engel var ama hiçbir TABAN kenarı çembere girmedi (blok_saved boş)
                # ve yine de yol yok → engel koordinatı taban graftan uzak olabilir
                # (karar yanlış dünya konumu yolluyor?) ya da goal erişilemez. Tanı.
                rospy.logwarn_throttle(
                    5.0, "[blok] SERT blok aktif ama çembere giren taban kenar YOK "
                         "(engel koordinatı graftan uzak?) ve rota None.")

        # ── Dönüş kalitesi (bisiklet modeli): en keskin dönüş + min dönüş yarıçapı ──
        max_donus_deg, min_yaricap = rota_donus_metrigi(path, yaw0)
        if path and max_donus_deg >= math.degrees(DONUS_CUSP_ESIK):
            print(f"{SARI}>>> [DÖNÜŞ] rota maks dönüş {max_donus_deg:.0f}° "
                  f"(min yarıçap {min_yaricap:.1f}m) — sıkışık!{SIFIRLA}")

        # ── ÇIKTI KAPISI: sürülemez (cusp/U-dönüşü) rotayı COMMIT ETME ──
        # Kullanıcı (2026-06-26): "araç buna uymasa da yine de hatalı rota çizmemeli."
        # Tetikleyiciyi tahmin etmek (girdi-tarafı guard'lar) yerine ÇIKTIYI denetle:
        # rota bir cusp (≥DONUS_CUSP_ESIK ~150° tek dönüş = U-dönüşü) içeriyorsa ve elde
        # KORUNACAK committed path varsa → çizme, eskisini koru. Cusp net sinyaldir:
        # meşru rotalar (düz takip, geçerli sollama) <150°; cusp yalnız araç kötü konumda
        # (karşı şeritte) reroute zorlandığında çıkar (canlı 141631Z max-dönüş 167°,
        # off-road). Tetikleyiciden BAĞIMSIZ → eski _ters_yon_start_riski (başlangıç-düğümü
        # tahmini) yerine GERÇEK rotayı keser. İlk rota (committed yok) daima kabul edilir
        # (yoksa araç hiç başlamaz; engele yaklaşırsa control e-stop güvenlik ağı).
        rota_reddedildi = False
        if path and self.full_path_world and max_donus_deg >= math.degrees(DONUS_CUSP_ESIK):
            rota_reddedildi = True
            rospy.logwarn_throttle(
                2.0, f"[rota] SÜRÜLEMEZ rota reddedildi (maks dönüş {max_donus_deg:.0f}° "
                     f"≥ cusp; min yarıçap {min_yaricap:.2f}m) → committed path korunuyor "
                     f"({reason})")

        # ── Tanı logu: rota uzaktan mı çiziliyor? ───────────────────
        if self.logger is not None:
            try:
                task_name = (GOREV_GEOJSON['features'][self.current_task_index]
                             ['properties']['name'])
            except Exception:  # noqa: BLE001
                task_name = None
            # log buradan önce full_path_world hâlâ ESKİ rota → kıyas geçerli.
            # path_changed=False → rota değişmedi (boşa recalc / oscillation işareti)
            path_changed = (path != self.full_path_world) if path else None
            # Rotanın CEZALI (ağırlıklı) toplam maliyeti — ceza-puan etkisini gösterir.
            # log_recalc bunu ham path_len_m ile kıyaslayıp ceza_orani üretir (>1 = ceza uygulandı).
            weighted = None
            if path and len(path) > 1:
                try:
                    weighted = sum(self.planner.get_cost(path[i], path[i + 1])
                                   for i in range(len(path) - 1))
                except Exception:  # noqa: BLE001
                    weighted = None
            self.logger.log_recalc(
                reason=reason, rx=rx, ry=ry, yaw=self.robot_yaw,
                front=(front_x, front_y), start_node=start_node,
                goal_node=goal_node, task_idx=self.current_task_index,
                task_name=task_name, path=path, path_changed=path_changed,
                weighted_cost=weighted,
            )
            # Dönüş kalitesi izi (bisiklet modeli): rotanın en keskin dönüşü +
            # min dönüş yarıçapı — turn-aware aramanın etkisi loglardan görülür.
            if path:
                self.logger.log_event(
                    "donus_metrik", reason=reason,
                    turn_aware=TURN_AWARE_AKTIF,
                    iki_yonlu_durak=IKI_YONLU_DURAK_AKTIF,
                    max_donus_deg=round(max_donus_deg, 1),
                    min_yaricap_m=round(min_yaricap, 2),
                    arac_min_yaricap_m=round(ARAC_DINGIL_M / math.tan(ARAC_MAX_DIREKSIYON), 2),
                    bloklu_kenar=len(blok_saved),
                    blok_sert=BLOK_SERT_AKTIF,
                    fallback_bloksuz=fallback_bloksuz,
                    rota_reddedildi=rota_reddedildi,
                    task_idx=self.current_task_index,
                )

        if path and not rota_reddedildi:
            with self._wp_lock:
                self.full_path_world   = path
                self.current_wp_index = 0
                self.is_path_calculated = True
                self._path_creation_time = time.time()
            # ── HESAPLAMA KİLİDİ — committed rotanın overtake'liğine göre KUR/TEMİZLE ──
            # KUR: rota KARŞI (sol) şeride giriyorsa → SOLLAMA kararı verildi → bu AN kilitle
            #   (kullanıcı 2026-06-27). Sağ şeride dönünce / 15s durunca _kilit_guncelle açar.
            # TEMİZLE (stale-lock heal): kilitliyken commit edilen rota artık KARŞI şeride
            #   GİRMİYORSA (ör. kilit_bypass yeni-koni recalc'ı SAĞ-şerit rotası ürettiyse)
            #   VE araç SAĞ şeritteyse → ortada sollama YOK → temizle.
            overtake = self._overtake_rotasi_mi(path)
            if HESAP_KILIDI_AKTIF and overtake and not self._hesap_kilitli:
                self._hesap_kilitli = True
                self._kilit_sol_serite_girdi = False
                self._durma_capa_xy = None     # taze kilit → fail-safe çapasını sıfırla
                rospy.loginfo_throttle(
                    2.0, "[kilit] sollama kararı (rota karşı şeride girdi) → KİLİTLENDİ "
                         "(sağ şeride dönünce açılır)")
            elif (HESAP_KILIDI_AKTIF and self._hesap_kilitli and not overtake
                    and self._son_sag_serit):
                self._hesap_kilitli = False
                self._kilit_sol_serite_girdi = False
                self._durma_capa_xy = None
                rospy.loginfo_throttle(
                    2.0, "[kilit] committed rota + araç sağ şeritte → kilit TEMİZLENDİ (stale-lock heal)")
            print(f"{YESIL}>>> [ROTA] {len(path)} WP oluşturuldu.{SIFIRLA}")
            if fallback_bloksuz:
                self.pub_durum.publish("BLOCKED")
            else:
                self.pub_durum.publish("OK")
        elif rota_reddedildi:
            # committed path KORUNUR (overwrite yok) → araç eski sürülebilir rotayı sürer.
            print(f"{SARI}>>> [ROTA] sürülemez rota reddedildi, committed path korundu.{SIFIRLA}")
            self.pub_durum.publish("OK")
        else:
            print(f"{KIRMIZI}!!! [HATA] Rota bulunamadı! "
                  f"{start_node} → {goal_node}{SIFIRLA}")
            self.pub_durum.publish("BLOCKED")

    # ==========================================
    #   HEDEF YAYINI
    # ==========================================
    def publish_current_waypoint(self) -> None:
        if not self.is_path_calculated or not self.full_path_world:
            return

        with self._wp_lock:
            # Get up to 5 waypoints ahead, pad with the goal node if we are near the end
            points = []
            for idx in range(1, 6): # wp1, wp2, wp3, wp4, wp5
                wp_idx = min(self.current_wp_index + idx, len(self.full_path_world) - 1)
                points.append(self.full_path_world[wp_idx])

        msg_parts = []
        for p in points:
            wx, wy = p[0], p[1]
            ntype = self.planner.node_types.get(p, 'intermediate')
            msg_parts.append(f"{wx:.2f},{wy:.2f},{ntype}")

        self.pub_hedef.publish("|".join(msg_parts))

    # ==========================================
    #   ÇİZİM
    # ==========================================
    # ==========================================
    #   ANA DÖNGÜ
    # ==========================================
    def loop(self) -> None:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_path_calculated:
                self.publish_current_waypoint()
            if ENABLE_GUI and self.new_data_available:
                self.visualizer.draw()
                self.new_data_available = False
            if ENABLE_GUI:
                self.visualizer.flush()
            rate.sleep()
