# -*- coding: utf-8 -*-
import math
import heapq
try:
    import rospy
except ImportError:
    rospy = None
from hedef_son.config import (
    ARAC_DINGIL_M, ARAC_MAX_DIREKSIYON, DONUS_CEZA_AGIRLIK,
    DONUS_CEZA_MAX, DONUS_CUSP_ESIK, DONUS_CUSP_CEZA, CEZA_ETKI
)

def ceza_carpani(ceza_puani: float) -> float:
    """0-100 ceza puanını D* kenar ağırlık çarpanına çevirir (1.0 + p/100 * CEZA_ETKI).
    Puan 0-100'e clamp'lenir (dinamik akışta bozuk/aşırı çarpan oluşmasın)."""
    p = max(0.0, min(100.0, ceza_puani))
    return 1.0 + (p / 100.0) * CEZA_ETKI


# ── Bisiklet-modeli dönüş cezası yardımcıları ────────────────────────────
def _yon_farki(a: float, b: float) -> float:
    """İki açının [-pi, pi] sarmalı MUTLAK farkı (rad)."""
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def _nokta_segment_mesafe(px, py, ax, ay, bx, by) -> float:
    """(px,py) noktasının [a,b] doğru parçasına en kısa mesafesi (m).
    Engel bloğunun bir kenarı (a→b) kapsayıp kapsamadığını doğru ölçer
    (orta-nokta yaklaşımı kısa kenarda engeli kaçırabilir)."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def bisiklet_donus_cezasi(dtheta: float, d_local: float) -> float:
    """Bir köşedeki dönüşün bisiklet-modeli cezası (metre-eşdeğeri).
    dtheta: baş açısı değişimi (rad, mutlak). d_local: dönüşün yayıldığı yerel
    mesafe (m). Eğrilik κ=dtheta/d_local → gereken direksiyon δ=atan(L·κ).
      • Yumuşak dönüş (δ küçük) → ucuz (oran²·ağırlık).
      • Keskin dönüş (δ→δ_max) → pahalı ("kolay dönülebilir" rota tercih edilir).
      • Aracın çevirebileceğinden keskin (δ>δ_max) → ağır ek ceza.
      • Cusp (dtheta≥eşik, U-dönüşü) → DONUS_CUSP_CEZA (pratikte yasak).
    Aynı açı kısa mesafede (60°/3m) uzun mesafeden (60°/10m) ÇOK daha pahalı."""
    if dtheta < 1e-3:
        return 0.0
    if dtheta >= DONUS_CUSP_ESIK or d_local < 1e-3:
        return DONUS_CUSP_CEZA
    kappa = dtheta / d_local
    delta = math.atan(ARAC_DINGIL_M * kappa)
    oran = delta / ARAC_MAX_DIREKSIYON          # 0..~ (1 = maks direksiyon)
    # Yumuşak, ÜST SINIRLI ceza: keskin dönüşü mesafeye göre cezalandırır ama
    # tek bir dönüş büyük bir mesafe kazancını ezemez (DONUS_CEZA_MAX ile sınırlı).
    # Gerçekten imkânsız U-dönüşleri yukarıdaki cusp bloğu yakalar.
    return min(DONUS_CEZA_AGIRLIK * oran * oran, DONUS_CEZA_MAX)


def rota_donus_metrigi(path, yaw0: float):
    """Rota boyunca en keskin dönüş (derece) + min dönüş yarıçapı (m).
    Köşe yerel mesafesi = komşu segmentlerin ortalaması (densify artefaktını
    sönümler). yaw0: ilk segmente giriş dönüşü için aracın başlangıç yaw'ı."""
    if not path or len(path) < 2:
        return 0.0, float('inf')
    max_deg = 0.0
    min_r = float('inf')
    prev_h = yaw0
    prev_seg = None
    for i in range(len(path) - 1):
        dx, dy = path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]
        seg = math.hypot(dx, dy)
        if seg < 1e-9:
            continue
        h = math.atan2(dy, dx)
        dtheta = _yon_farki(h, prev_h)
        d_local = seg if prev_seg is None else 0.5 * (prev_seg + seg)
        if math.degrees(dtheta) > max_deg:
            max_deg = math.degrees(dtheta)
        if dtheta > 1e-3:
            r = d_local / dtheta
            if r < min_r:
                min_r = r
        prev_h = h
        prev_seg = seg
    return max_deg, min_r


# ==========================================
#   D* LITE PLANNER
# ==========================================
class DLitePlanner:
    def __init__(self):
        self.adj_list: dict[tuple, list] = {}
        self.pred_list: dict[tuple, list] = {}  # Yönlendirilmiş graf için ters komşuluk listesi
        self.edge_weights: dict[tuple[tuple, tuple], float] = {}
        self.node_types: dict[tuple, str] = {}
        self.nodes: set[tuple] = set()
        self.g:   dict[tuple, float] = {}
        self.rhs: dict[tuple, float] = {}
        self.U:   list = []
        self.s_start = None
        self.s_goal  = None

    # ── Graph yönetimi ──────────────────────────────────────────────
    def add_edge(self, p1: tuple, p2: tuple, weight: float = None) -> None:
        self.adj_list.setdefault(p1, [])
        if p2 not in self.adj_list[p1]:
            self.adj_list[p1].append(p2)
        
        self.pred_list.setdefault(p2, [])
        if p1 not in self.pred_list[p2]:
            self.pred_list[p2].append(p1)

        self.nodes.add(p1)
        self.nodes.add(p2)
        if weight is not None:
            self.edge_weights[(p1, p2)] = weight
        else:
            self.edge_weights[(p1, p2)] = self.dist(p1, p2)

    def get_cost(self, u: tuple, v: tuple) -> float:
        return self.edge_weights.get((u, v), self.dist(u, v))

    def remove_edge_directed(self, src: tuple, dst: tuple) -> bool:
        """Tek yönlü kenar siler; kenar yoksa False döner."""
        try:
            self.adj_list[src].remove(dst)
            if dst in self.pred_list:
                try:
                    self.pred_list[dst].remove(src)
                except ValueError:
                    pass
            return True
        except (KeyError, ValueError):
            return False

    def restore_edge_directed(self, src: tuple, dst: tuple) -> None:
        """Tek yönlü kenarı geri ekler (varsa tekrar eklemez)."""
        self.adj_list.setdefault(src, [])
        if dst not in self.adj_list[src]:
            self.adj_list[src].append(dst)
        
        self.pred_list.setdefault(dst, [])
        if src not in self.pred_list[dst]:
            self.pred_list[dst].append(src)

    # ── Yardımcılar ─────────────────────────────────────────────────
    @staticmethod
    def dist(p1: tuple, p2: tuple) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def calculate_key(self, s: tuple) -> tuple:
        # NOT: Klasik D* Lite'taki km (key modifier) burada YOK. km yalnızca start
        # hareket ederken g/rhs'yi koruyup artımlı replan yapan sürümde gerekir.
        # Bu planlayıcı her find_path'te sıfırdan arıyor (aşağıdaki açıklamaya bak),
        # dolayısıyla km her zaman 0 olurdu → ölü terim, kaldırıldı (Faz4-lite).
        g_val  = self.g.get(s,   float('inf'))
        rhs_val = self.rhs.get(s, float('inf'))
        min_val = min(g_val, rhs_val)
        return (min_val + self.dist(self.s_start, s), min_val)

    # ── Çekirdek D* Lite ────────────────────────────────────────────
    def update_vertex(self, u: tuple) -> None:
        if u != self.s_goal:
            min_rhs = float('inf')
            for nb in self.adj_list.get(u, []):
                val = self.get_cost(u, nb) + self.g.get(nb, float('inf'))
                if val < min_rhs:
                    min_rhs = val
            self.rhs[u] = min_rhs
        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def compute_shortest_path(self) -> None:
        """
        Orijinal D* Lite erken çıkış koşulu:
          U boş DEĞİLSE ve
          (U.top_key < calculate_key(s_start)) VEYA rhs[s_start] != g[s_start]
        koşulu sağlandığı sürece çalış.
        """
        visited: set[tuple] = set()
        while self.U:
            # Heap'in güncel tepesine bak (pop etme)
            top_key, _ = self.U[0]
            start_key   = self.calculate_key(self.s_start)

            # Erken çıkış: s_start çözüldü ve heap'te daha iyi düğüm yok
            if (top_key >= start_key and
                    self.rhs.get(self.s_start, float('inf')) ==
                    self.g.get(self.s_start, float('inf'))):
                break

            k_old, u = heapq.heappop(self.U)
            if u in visited:
                continue

            k_new = self.calculate_key(u)
            if k_old < k_new:          # anahtar eskimiş → yeniden ekle
                heapq.heappush(self.U, (k_new, u))
                continue

            visited.add(u)
            if self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                for s in self.pred_list.get(u, []):
                    self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for s in self.pred_list.get(u, []):
                    self.update_vertex(s)

    def find_path(self, start: tuple, goal: tuple):
        if start not in self.adj_list or goal not in self.adj_list:
            rospy.logwarn(f"[D*Lite] start veya goal graph'ta yok! "
                          f"start:{start} goal:{goal}")
            return None

        # ── Soğuk (sıfırdan) arama — bilinçli tercih (Faz4-lite) ────────
        # Klasik D* Lite g/rhs/U/km'yi replanlar arası korur ve km += h ile
        # start'ı kaydırarak artımlı (ucuz) replan yapar. Burada her find_path
        # sıfırdan arıyor. Gerekçe: (1) graf statik ve küçük (644 düğüm) — ölçüm:
        # cold find_path medyan 0.88ms / p95 3.18ms (20Hz=50ms bütçe içinde önemsiz);
        # (2) ileri-yön filtresi her replan'da start kenarlarını geçici silip geri
        # ekliyor → artımlı durumu korumak her seferinde update_vertex yayılımı
        # gerektirir, küçük kazancı yer ve risk getirir. Bu yüzden artımlı
        # sürüme geçilmedi (Faz4 atlandı).
        self.s_start = start
        self.s_goal  = goal
        self.U  = []
        self.g  = {}
        self.rhs = {}
        self.rhs[self.s_goal] = 0.0
        heapq.heappush(self.U, (self.calculate_key(self.s_goal), self.s_goal))
        self.compute_shortest_path()

        if self.g.get(self.s_start, float('inf')) == float('inf'):
            rospy.logwarn("[D*Lite] Yol bulunamadı (g=inf)")
            return None

        # Yolu geri iz sür
        path  = [self.s_start]
        curr  = self.s_start
        seen  = {self.s_start}              # O(1) döngü tespiti
        max_steps = len(self.nodes) + 10

        for _ in range(max_steps):
            if curr == self.s_goal:
                break
            neighbors = self.adj_list.get(curr, [])
            if not neighbors:
                rospy.logwarn("[D*Lite] Çıkışsız düğüme ulaşıldı.")
                return None             # yarım rota gönderme

            best_next = min(
                neighbors,
                key=lambda n: self.get_cost(curr, n) + self.g.get(n, float('inf'))
            )

            if best_next in seen:
                rospy.logwarn("[D*Lite] Döngü tespit edildi — rota geçersiz.")
                return None             # yarım rota yerine None dön

            seen.add(best_next)
            path.append(best_next)
            curr = best_next

        return path if len(path) > 1 and curr == self.s_goal else None

    # ── Turn-aware arama (bisiklet-modeli dönüş cezası - D* Lite) ──
    def find_path_turn_aware(self, start: tuple, goal: tuple,
                             start_yaw: float = 0.0):
        """Yönlü-kenar durumları üzerinde D* Lite + bisiklet-modeli dönüş cezası.

        Durum = (u, v): "v'ye u'dan gelindi" (u→v yönlü kenarı). Geçiş
        (u,v)→(v,w) maliyeti = get_cost(v,w) + bisiklet_donus_cezasi(Δθ, d_yerel),
        Δθ = giriş başı (u→v) ile çıkış başı (v→w) arasındaki açı.

        D* Lite ile geri-yön arama (backward search) yapılır:
        Arama hedef(goal) düğümünden başlar ve start yönüne doğru ilerler.
        Maliyet g[s], s durumundan goal'e olan maliyeti temsil eder.
        """
        if start not in self.adj_list or goal not in self.adj_list:
            if rospy:
                rospy.logwarn(f"[turn-aware D*] start/goal graph'ta yok: {start} {goal}")
            return None
        if start == goal:
            return [start]

        def _dir(a, b):
            return math.atan2(b[1] - a[1], b[0] - a[0])

        g = {}
        rhs = {}
        U = []
        visited = set()

        def calculate_key(s):
            u, v = s
            g_val = g.get(s, float('inf'))
            rhs_val = rhs.get(s, float('inf'))
            min_val = min(g_val, rhs_val)
            h_val = self.dist(v, start)
            return (min_val + h_val, min_val)

        # Goal'ün predecessors'larını al
        preds_of_goal = self.pred_list.get(goal, [])
        if not preds_of_goal:
            if rospy:
                rospy.logwarn("[turn-aware D*] Goal düğümüne gelen kenar yok!")
            return None

        # Goal durumlarını sıfırla ve heap'e ekle
        for u in preds_of_goal:
            s_g = (u, goal)
            rhs[s_g] = 0.0
            heapq.heappush(U, (calculate_key(s_g), s_g))

        # Start durumları
        start_neighbors = self.adj_list.get(start, [])
        if not start_neighbors:
            if rospy:
                rospy.logwarn("[turn-aware D*] Start düğümünden çıkan kenar yok!")
            return None
        S_start = {(start, w) for w in start_neighbors}

        def update_vertex(s):
            u, v = s
            if v != goal:
                min_rhs = float('inf')
                for w in self.adj_list.get(v, []):
                    s_next = (v, w)
                    d_out = self.dist(v, w)
                    dtheta = _yon_farki(_dir(v, w), _dir(u, v))
                    d_local = 0.5 * (self.dist(u, v) + d_out)
                    c = self.get_cost(v, w) + bisiklet_donus_cezasi(dtheta, d_local)
                    val = c + g.get(s_next, float('inf'))
                    if val < min_rhs:
                        min_rhs = val
                rhs[s] = min_rhs

            if g.get(s, float('inf')) != rhs.get(s, float('inf')):
                heapq.heappush(U, (calculate_key(s), s))

        # D* Lite shortest path hesaplama
        while U:
            min_start_key = (float('inf'), float('inf'))
            all_consistent = True
            for s_s in S_start:
                if g.get(s_s, float('inf')) != rhs.get(s_s, float('inf')):
                    all_consistent = False
                key_s = calculate_key(s_s)
                if key_s < min_start_key:
                    min_start_key = key_s

            top_key, _ = U[0]
            if top_key >= min_start_key and all_consistent:
                break

            k_old, u_v = heapq.heappop(U)
            if u_v in visited:
                continue

            k_new = calculate_key(u_v)
            if k_old < k_new:
                heapq.heappush(U, (k_new, u_v))
                continue

            visited.add(u_v)
            u, v = u_v
            
            if g.get(u_v, float('inf')) > rhs.get(u_v, float('inf')):
                g[u_v] = rhs[u_v]
                for t in self.pred_list.get(u, []):
                    update_vertex((t, u))
            else:
                g[u_v] = float('inf')
                update_vertex(u_v)
                for t in self.pred_list.get(u, []):
                    update_vertex((t, u))

        # En iyi başlangıç kenarını seç: c_init(w) + g[(start, w)] minimum olmalı
        best_w = None
        min_total_cost = float('inf')
        for w in start_neighbors:
            s_s = (start, w)
            g_val = g.get(s_s, float('inf'))
            if g_val == float('inf'):
                continue
            d_out = self.dist(start, w)
            dtheta = _yon_farki(_dir(start, w), start_yaw)
            c_init = self.get_cost(start, w) + bisiklet_donus_cezasi(dtheta, d_out)
            total_cost = c_init + g_val
            if total_cost < min_total_cost:
                min_total_cost = total_cost
                best_w = w

        if best_w is None:
            if rospy:
                rospy.logwarn("[turn-aware D*] Yol bulunamadı.")
            return None

        # Yolu ileri yönde oluştur (start -> best_w -> ...)
        path = [start, best_w]
        curr_state = (start, best_w)
        seen = {curr_state}
        max_steps = len(self.nodes) * 2

        for _ in range(max_steps):
            u, v = curr_state
            if v == goal:
                break
            
            neighbors = self.adj_list.get(v, [])
            if not neighbors:
                break
            
            best_s_next = None
            min_val = float('inf')
            for w in neighbors:
                s_next = (v, w)
                d_out = self.dist(v, w)
                dtheta = _yon_farki(_dir(v, w), _dir(u, v))
                d_local = 0.5 * (self.dist(u, v) + d_out)
                c = self.get_cost(v, w) + bisiklet_donus_cezasi(dtheta, d_local)
                val = c + g.get(s_next, float('inf'))
                if val < min_val:
                    min_val = val
                    best_s_next = s_next

            if best_s_next is None or min_val == float('inf') or best_s_next in seen:
                break
            
            seen.add(best_s_next)
            path.append(best_s_next[1])
            curr_state = best_s_next

        return path if path[-1] == goal else None



