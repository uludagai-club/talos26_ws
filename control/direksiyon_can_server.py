#!/usr/bin/env python3
"""
Uzak direksiyon -> CAN sunucusu (Linux/sim tarafı).

Windows PC'deki direksiyon ham eksen verisini SSH (stdin) veya UDP üzerinden
alır, mapping + CAN framing'i BURADA yapar ve vcan0'a control.py ile birebir
aynı frame'leri basar:
    0x100 gaz/fren/vites, 0x201 direksiyon, 0x102 el freni.

Tüm cihaz-özgü mapping (hangi eksen direksiyon, pedal yönü, deadzone) tek yerde
(burada) yaşar -> Windows tarafı "aptal okuyucu", kalibrasyon için onu
değiştirmek gerekmez.

Protokol (Windows okuyucudan gelen her satır, boşlukla ayrılmış tam sayılar):
    X Y Z R U V BUTTONS [GEAR] [HANDBRAKE] [AUTO]
joyGetPosEx ham değerleri: eksenler 0..65535 (merkez ~32767), BUTTONS bitmask.
Discrete kontroller (vites/elfreni/otonom) reader'ın açık alanlarından gelir
(GEAR=1/2/3, HANDBRAKE=0/1, AUTO=0/1) — buton bitmask'i kırılgan (idle taban 0x0F)
olduğu için bunlara güvenilmez.

Kullanım (Linux'ta SSH pipe hedefi olarak):
    # Windows tarafı:  reader | ssh hilmi@192.168.1.234 "python3 .../direksiyon_can_server.py"
    python3 direksiyon_can_server.py                 # stdin'den oku (SSH pipe)
    python3 direksiyon_can_server.py --udp 5005      # UDP 5005'ten oku
    python3 direksiyon_can_server.py --swap-pedals --invert-steer  # kalibrasyon

GÜVENLİK:
  * Ağ-kontrol linki -> WATCHDOG: --timeout saniye veri gelmezse tam fren+boş.
  * Bu MANUEL sürüş. control.py (otonom) ile aynı anda çalıştırma (frame çakışır).
"""

import argparse
import select
import socket
import sys
import time

import can

# control.py ile birebir sabitler
MAX_STEER_ANGLE = 30.0
GEAR_NEUTRAL = 1
GEAR_FORWARD = 2
GEAR_REVERSE = 3

# Otonom devir BIRINCIL yol: reader'ın 10. alanı (AUTO 0/1). Aşağıdaki bit-tabanlı
# yol sadece field9 GELMEZSE devreye girer (yedek). idle byte6=0x0F olduğundan
# bit-tabanlı yol kırılgandır; OTONOM_BIT idle'da 0 olan bir bit OLMALI (4/5 vites,
# 0-3 idle taban -> KULLANILAMAZ). field9 varken bu önemsiz. -1 = bit yolu kapalı.
OTONOM_BIT = -1

AXIS_CENTER = 32767.0   # joyGetPosEx ekseni merkez
AXIS_HALF = 32767.0     # merkez -> uç


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def apply_expo(x, expo):
    """Expo eğrisi: uçları korur (tam=tam), merkezi yumuşatır -> ince kontrol.
    expo=0 lineer, expo=1 saf kübik. x in [-1,1] veya [0,1]."""
    expo = clamp(expo, 0.0, 1.0)
    return (1.0 - expo) * x + expo * (x ** 3)


class State:
    """Windows'tan gelen son ham değerler + zaman damgası."""
    def __init__(self):
        self.axes = {"X": AXIS_CENTER, "Y": AXIS_CENTER, "Z": AXIS_CENTER,
                     "R": AXIS_CENTER, "U": AXIS_CENTER, "V": AXIS_CENTER}
        self.buttons = 0
        self.gear_cmd = None    # akıştan gelen vites (1=boş, 2=ileri, 3=geri); None=yok
        self.hb_cmd = None      # akıştan gelen el freni (9. alan): 0/1; None=yok
        self.auto_cmd = None    # akıştan gelen otonom devir (10. alan): 0/1; None=yok
        self.last_rx = 0.0


AXIS_ORDER = ["X", "Y", "Z", "R", "U", "V"]


def parse_line(line, state, now):
    """'X Y Z R U V BUTTONS [GEAR] [HANDBRAKE] [AUTO]' satırını State'e işle.

    8. alan (ops.) vites: 1=boş, 2=ileri, 3=geri (reader geri butonundan 3 üretir).
    9. alan (ops.) el freni: 0/1.
    10. alan (ops.) otonom devir: 0/1 (reader otonom butonunda toggle edip durumu yollar).
    Yoksa ilgili komut buton-bit/varsayılan kalır.
    """
    parts = line.split()
    if len(parts) < 7:
        return False
    try:
        for i, name in enumerate(AXIS_ORDER):
            state.axes[name] = float(parts[i])
        state.buttons = int(parts[6])
        if len(parts) >= 8:
            g = int(parts[7])
            state.gear_cmd = g if g in (GEAR_NEUTRAL, GEAR_FORWARD, GEAR_REVERSE) else None
        if len(parts) >= 9:
            state.hb_cmd = 1 if int(parts[8]) else 0
        if len(parts) >= 10:
            state.auto_cmd = 1 if int(parts[9]) else 0
        state.last_rx = now
        return True
    except (ValueError, IndexError):
        return False


def map_controls(state, cfg):
    """Ham eksen/buton -> (steer_deg, throttle_pct, brake_pct)."""
    # Direksiyon
    steer_n = (state.axes[cfg.steer_source] - AXIS_CENTER) / AXIS_HALF
    if abs(steer_n) < cfg.steer_deadzone:
        steer_n = 0.0
    if cfg.invert_steer:
        steer_n = -steer_n
    # His ayarı: expo (merkez yumuşatma) + gain (max kısma). gain=1 -> tam ±37°.
    steer_n = apply_expo(steer_n, cfg.steer_expo) * cfg.steer_gain
    steer_deg = clamp(steer_n * MAX_STEER_ANGLE, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)

    # Pedallar
    if cfg.pedal_mode == "split":
        # Tek birleşik eksen: merkezden bir yön gaz, diğer yön fren
        y = state.axes[cfg.pedal_source]
        gas = clamp((AXIS_CENTER - y) / AXIS_HALF, 0.0, 1.0)   # eksen DÜŞÜNCE gaz
        brk = clamp((y - AXIS_CENTER) / AXIS_HALF, 0.0, 1.0)   # eksen ÇIKINCA fren
        if cfg.swap_pedals:
            gas, brk = brk, gas
    else:  # separate
        gas = clamp(state.axes[cfg.throttle_source] / 65535.0, 0.0, 1.0)
        brk = clamp(state.axes[cfg.brake_source] / 65535.0, 0.0, 1.0)
        if cfg.invert_throttle:
            gas = 1.0 - gas
        if cfg.invert_brake:
            brk = 1.0 - brk

    if gas < cfg.pedal_deadzone:
        gas = 0.0
    if brk < cfg.pedal_deadzone:
        brk = 0.0
    gas = apply_expo(gas, cfg.throttle_expo)   # düşük gazda ince kontrol
    throttle_pct = gas * 100.0 * cfg.throttle_scale
    brake_pct = brk * 100.0
    return steer_deg, throttle_pct, brake_pct


def send_can(bus, throttle_pct, brake_pct, steer_deg, gear, handbrake):
    """control.py ile birebir frame'ler."""
    throttle_raw = int(clamp(throttle_pct, 0, 100) * 100)
    brake_raw = int(clamp(brake_pct, 0, 100))
    data_ctrl = throttle_raw.to_bytes(2, "little") + bytes([gear]) \
        + brake_raw.to_bytes(1, "little") + bytes(4)
    steer_raw = int((clamp(steer_deg, -MAX_STEER_ANGLE, MAX_STEER_ANGLE) + 500) * 10)
    data_steer = steer_raw.to_bytes(2, "little") + bytes(6)
    try:
        bus.send(can.Message(arbitration_id=0x100, data=data_ctrl, is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x201, data=data_steer, is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x102,
                             data=bytes([1 if handbrake else 0]) + bytes(7),
                             is_extended_id=False))
    except can.CanError as e:
        print(f"CAN hatası: {e}", file=sys.stderr)


def main():
    p = argparse.ArgumentParser(description="Uzak direksiyon -> vcan0 CAN sunucusu")
    p.add_argument("--channel", default="vcan0")
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--timeout", type=float, default=0.5,
                   help="bu kadar sn veri gelmezse fail-safe (tam fren+boş)")
    p.add_argument("--udp", type=int, default=None, help="stdin yerine bu UDP portunu dinle")
    p.add_argument("--verbose", action="store_true")
    # Eksen mapping
    p.add_argument("--steer-source", default="X", choices=AXIS_ORDER)
    p.add_argument("--pedal-mode", default="split", choices=["split", "separate"])
    p.add_argument("--pedal-source", default="Y", choices=AXIS_ORDER, help="split modda birleşik pedal ekseni")
    p.add_argument("--throttle-source", default="Y", choices=AXIS_ORDER, help="separate modda gaz ekseni")
    p.add_argument("--brake-source", default="Z", choices=AXIS_ORDER, help="separate modda fren ekseni")
    p.add_argument("--swap-pedals", action="store_true", help="split modda gaz/fren yönünü değiştir")
    p.add_argument("--invert-steer", action="store_true")
    p.add_argument("--invert-throttle", action="store_true")
    p.add_argument("--invert-brake", action="store_true")
    p.add_argument("--steer-deadzone", type=float, default=0.05)
    p.add_argument("--pedal-deadzone", type=float, default=0.03)
    p.add_argument("--steer-gain", type=float, default=1.0,
                   help="direksiyon max kısma 0..1 (1=tam ±37°, 0.8≈±30°)")
    p.add_argument("--steer-expo", type=float, default=0.0,
                   help="direksiyon merkez yumuşatma 0..1 (0=lineer, 0.3-0.5 daha az hassas)")
    p.add_argument("--throttle-expo", type=float, default=0.0,
                   help="gaz merkez yumuşatma 0..1 (düşük gazda ince kontrol)")
    p.add_argument("--throttle-scale", type=float, default=0.3,
                   help="gaz tavanı 0..1 (ilk testte düşük tut!)")
    # Buton mapping (bit index; -1 = devre dışı). Butonlar çalışmazsa vites İLERİ kalır.
    p.add_argument("--gear-fwd-bit", type=int, default=-1)
    p.add_argument("--gear-rev-bit", type=int, default=-1)
    p.add_argument("--gear-neutral-bit", type=int, default=-1)
    p.add_argument("--handbrake-bit", type=int, default=-1)
    p.add_argument("--estop-bit", type=int, default=-1)
    cfg = p.parse_args()

    try:
        bus = can.interface.Bus(channel=cfg.channel, interface="socketcan")
    except Exception as e:
        print(f"HATA: CAN bus açılamadı ({cfg.channel}): {e}", file=sys.stderr)
        sys.exit(1)

    state = State()
    sock = None
    if cfg.udp is not None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", cfg.udp))
        sock.setblocking(False)
        rx_fd = sock
        print(f"UDP {cfg.udp} dinleniyor...", file=sys.stderr)
    else:
        rx_fd = sys.stdin
        print("stdin dinleniyor (SSH pipe)...", file=sys.stderr)

    period = 1.0 / cfg.rate
    gear = GEAR_FORWARD
    handbrake = False
    otonom = False          # True iken bus control.py'ye bırakılır (set frame yazmaz)
    prev_buttons = 0
    print(f"Sunucu çalışıyor: {cfg.channel} @ {cfg.rate}Hz, watchdog {cfg.timeout}s, "
          f"gaz tavanı {cfg.throttle_scale*100:.0f}%", file=sys.stderr)

    def bit_rose(bit):
        if bit < 0:
            return False
        mask = 1 << bit
        return bool(state.buttons & mask) and not (prev_buttons & mask)

    try:
        while True:
            loop_start = time.time()

            # Gelen tüm satırları/paketleri çek, en sonuncuyu kullan
            while True:
                r, _, _ = select.select([rx_fd], [], [], 0)
                if not r:
                    break
                if sock is not None:
                    try:
                        data, _ = sock.recvfrom(256)
                    except BlockingIOError:
                        break
                    for ln in data.decode(errors="ignore").splitlines():
                        parse_line(ln, state, loop_start)
                else:
                    ln = sys.stdin.readline()
                    if ln == "":      # EOF -> SSH/pipe kapandı
                        raise KeyboardInterrupt
                    parse_line(ln, state, loop_start)

            # Otonom devir kararı: field9 (AUTO) öncelikli — reader toggle'ı tutar,
            # biz durumu MİRROR ederiz. field9 yoksa OTONOM_BIT bit-kenarı (yedek).
            # Güvenlik: geri viteste otonoma GEÇME (control.py ileri sürer).
            in_reverse = (state.gear_cmd == GEAR_REVERSE
                          if state.gear_cmd is not None else gear == GEAR_REVERSE)
            if state.auto_cmd is not None:
                desired = bool(state.auto_cmd) and not in_reverse
            elif bit_rose(OTONOM_BIT):
                desired = (not otonom) and not in_reverse
            else:
                desired = otonom
            if desired != otonom:
                otonom = desired
                bus.send(can.Message(
                    arbitration_id=0x500,
                    data=bytes([1 if otonom else 0]) + bytes(7),
                    is_extended_id=False))
                print(f"\n>>> {'OTONOM devraldı (set sustu)' if otonom else 'MANUEL devraldı (otonom durdu)'} (0x500={1 if otonom else 0})",
                      file=sys.stderr)

            # Otonom aktif: bus'ı control.py'ye bırak, hiçbir frame gönderme.
            # Sadece toggle butonunu izlemeye devam et.
            if otonom:
                prev_buttons = state.buttons
                time.sleep(period)
                continue

            # Watchdog: link koptu -> fail-safe
            stale = (loop_start - state.last_rx) > cfg.timeout
            if stale or state.last_rx == 0.0:
                send_can(bus, 0, 100, 0, GEAR_NEUTRAL, True)
                if cfg.verbose:
                    sys.stderr.write("\r[WATCHDOG] veri yok -> FREN          ")
                    sys.stderr.flush()
                time.sleep(period)
                continue

            steer_deg, throttle_pct, brake_pct = map_controls(state, cfg)

            # Buton geçişleri
            if bit_rose(cfg.gear_fwd_bit):
                gear = GEAR_FORWARD
            if bit_rose(cfg.gear_rev_bit):
                gear = GEAR_REVERSE
            if bit_rose(cfg.gear_neutral_bit):
                gear = GEAR_NEUTRAL
            # Akıştan gelen klavye vitesi buton-bitlerini geçersiz kılar
            if state.gear_cmd is not None:
                gear = state.gear_cmd
            if bit_rose(cfg.handbrake_bit):
                handbrake = not handbrake
            # Akıştan gelen el freni (9. alan) buton-toggle'ı geçersiz kılar
            if state.hb_cmd is not None:
                handbrake = bool(state.hb_cmd)
            estop = cfg.estop_bit >= 0 and bool(state.buttons & (1 << cfg.estop_bit))
            prev_buttons = state.buttons

            # El freni aktifken Y'den gelen ayak frenini bastır -> saf el freni
            # (yoksa el freni + tam ayak freni birlikte uygulanır)
            if handbrake:
                brake_pct = 0.0

            if estop:
                throttle_pct, brake_pct, gear = 0.0, 100.0, GEAR_NEUTRAL

            send_can(bus, throttle_pct, brake_pct, steer_deg, gear,
                     handbrake or estop)

            if cfg.verbose:
                sys.stderr.write(
                    f"\rdir:{steer_deg:+6.1f}° gaz:{throttle_pct:5.1f}% "
                    f"fren:{brake_pct:5.1f}% vites:{gear} elf:{int(handbrake)} "
                    f"estop:{int(bool(estop))}    ")
                sys.stderr.flush()

            dt = time.time() - loop_start
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nKapanıyor: fren 100%, boş vites...", file=sys.stderr)
        for _ in range(10):
            send_can(bus, 0, 100, 0, GEAR_NEUTRAL, True)
            time.sleep(0.02)
        bus.shutdown()
        if sock:
            sock.close()


if __name__ == "__main__":
    main()
