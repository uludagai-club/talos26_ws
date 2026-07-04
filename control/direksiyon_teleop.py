#!/usr/bin/env python3
"""
Snopy V5H (ve benzeri generic USB HID) direksiyon seti -> CAN köprüsü.

Direksiyon setini Linux joystick API'si (/dev/input/jsX) üzerinden okur ve
control.py ile BİREBİR aynı CAN frame formatında komut gönderir:

    0x100  Gaz/Fren/Vites   byte0-1: gaz%*100 (LE u16), byte2: vites, byte3: fren%
    0x201  Direksiyon       byte0-1: (açı+500)*10 (LE u16)
    0x102  El freni         byte0: 1=çek / 0=bırak

Bağımlılık: python-can. Joystick okuması saf stdlib (pygame GEREKMEZ) ->
NoMachine USB forwarding üzerinden daha güvenilir.

ÖNEMLİ:
  * Bu MANUEL sürüş aracıdır. control.py release modunda beklerken kullanılabilir.
    --auto-toggle-btn ile otonoma devir verilince teleop 0x100/0x201 yazmayı
    bırakır; tekrar basınca manuel sürüş devralır.
  * Eksen/buton index'leri cihaza özgüdür. Linux js tarafında butonlar 0-based,
    fiziksel "buton 3" genelde `b2` olarak görünür. Önce kalibrasyon modunu çalıştır:
        python3 direksiyon_teleop.py --kalibre
    ve direksiyonu/pedalları oynatıp hangi eksenin ne olduğunu gör, sonra
    --steer-axis / --throttle-axis / --brake-axis ... flag'leriyle ayarla.

Örnek:
    # vcan0'a (sim), varsayılan mapping ile:
    python3 direksiyon_teleop.py
    # özel eksen/buton mapping + ters direksiyon:
    python3 direksiyon_teleop.py --steer-axis 0 --throttle-axis 2 \
        --brake-axis 5 --invert-steer --gear-fwd-btn 4 --gear-rev-btn 5 \
        --estop-btn 0 --handbrake-btn 1
"""

import argparse
import os
import select
import struct
import sys
import time

import can

# control.py ile uyumlu sabitler (oradan kopyalandı)
MAX_STEER_ANGLE = 30.0   # derece; full-lock'ta gönderilecek tepe açı
GEAR_NEUTRAL = 1
GEAR_FORWARD = 2
GEAR_REVERSE = 3         # can_decoder: 3=Reverse

# Linux joystick olay yapısı: struct js_event { u32 time; s16 value; u8 type; u8 number; }
JS_EVENT_FMT = "<IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80     # init olayları type ile OR'lanır

AXIS_MAX = 32767.0       # joystick ekseni tam ölçek


class JoystickReader:
    """/dev/input/jsX cihazını bloklamadan okur; eksen/buton durumunu tutar."""

    def __init__(self, dev_path):
        self.fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
        self.axes = {}     # number -> int (-32767..32767)
        self.buttons = {}  # number -> 0/1
        self.last_event_num = {"axis": None, "button": None}

    def poll(self):
        """Bekleyen tüm olayları işle (init olayları dahil)."""
        while True:
            r, _, _ = select.select([self.fd], [], [], 0)
            if not r:
                break
            try:
                buf = os.read(self.fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break
            if len(buf) < JS_EVENT_SIZE:
                break
            _t, value, etype, number = struct.unpack(JS_EVENT_FMT, buf)
            etype &= ~JS_EVENT_INIT  # init bayrağını at, tipi normalize et
            if etype == JS_EVENT_AXIS:
                self.axes[number] = value
                self.last_event_num["axis"] = number
            elif etype == JS_EVENT_BUTTON:
                self.buttons[number] = value
                self.last_event_num["button"] = number

    def axis(self, number, default=0):
        return self.axes.get(number, default)

    def button(self, number):
        return self.buttons.get(number, 0)

    def close(self):
        os.close(self.fd)


def norm_axis(raw):
    """Ham ekseni -1..1'e ölçekle."""
    return max(-1.0, min(1.0, raw / AXIS_MAX))


def pedal_value(raw, rest_raw, invert):
    """
    Pedal eksenini 0..1'e çevirir.
    Çoğu pedal serbestte bir uçta (rest_raw), basılıyken diğer uçtadır.
    rest -1 (-32767), basılı +1 ise: (norm - (-1)) / 2 = (norm+1)/2.
    """
    n = norm_axis(raw)
    rest = norm_axis(rest_raw)
    # rest noktasından tam basılıya kadar lineer
    span = (1.0 - rest) if rest <= 0 else (-1.0 - rest)
    if abs(span) < 1e-6:
        return 0.0
    val = (n - rest) / span
    if invert:
        val = 1.0 - val
    return max(0.0, min(1.0, val))


def kalibrasyon_modu(reader):
    """Canlı eksen/buton değerlerini bas. Ctrl+C ile çık."""
    print("=" * 60)
    print("  KALİBRASYON MODU - direksiyonu/pedalları/butonları oynat")
    print("  Hangi eksenin ne olduğunu not al, sonra --*-axis flag'leriyle ver.")
    print("  Son değişen tuş/eksen ayrıca aşağıda satır olarak yazılır.")
    print("  Çıkış: Ctrl+C")
    print("=" * 60)
    prev_axes = {}
    prev_buttons = {}
    try:
        while True:
            reader.poll()
            for n, v in sorted(reader.axes.items()):
                old = prev_axes.get(n)
                if old is not None and abs(v - old) > 2500:
                    print(f"\n  DEGISTI axis a{n}: {old:+6d} -> {v:+6d}")
                prev_axes[n] = v
            for n, v in sorted(reader.buttons.items()):
                old = prev_buttons.get(n, 0)
                if v != old:
                    print(f"\n  DEGISTI button b{n}: {old} -> {v}")
                prev_buttons[n] = v
            axes = " ".join(f"a{n}:{v:+6d}" for n, v in sorted(reader.axes.items()))
            btns = " ".join(f"b{n}:{v}" for n, v in sorted(reader.buttons.items()) if v)
            sys.stdout.write(f"\r{axes}   [{btns}]      ")
            sys.stdout.flush()
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nKalibrasyon bitti.")


def send_loop(reader, bus, cfg):
    """Ana köprü döngüsü: joystick oku -> CAN frame gönder."""
    period = 1.0 / cfg.rate
    gear = GEAR_FORWARD
    handbrake = False
    otonom = False
    prev_btn = {}

    def edge(btn):
        """Butonun basılı->basıldı geçişini (rising edge) yakala."""
        if btn is None or btn < 0:
            return False
        cur = reader.button(btn)
        rose = cur and not prev_btn.get(btn, 0)
        prev_btn[btn] = cur
        return rose

    print(f"Köprü çalışıyor: {cfg.channel} @ {cfg.rate}Hz. Çıkış: Ctrl+C")
    if cfg.pedal_mode == "split":
        print(
            "Mapping: "
            f"steer=a{cfg.steer_axis}, pedal(split)=a{cfg.pedal_axis} (neg=gaz, pos=fren), "
            f"ileri=b{cfg.gear_fwd_btn}, geri=b{cfg.gear_rev_btn}, "
            f"elfren=b{cfg.handbrake_btn}, estop=b{cfg.estop_btn}, "
            f"otonom=b{cfg.auto_toggle_btn}"
        )
    else:
        print(
            "Mapping: "
            f"steer=a{cfg.steer_axis}, gaz=a{cfg.throttle_axis}, fren=a{cfg.brake_axis}, "
            f"ileri=b{cfg.gear_fwd_btn}, geri=b{cfg.gear_rev_btn}, "
            f"elfren=b{cfg.handbrake_btn}, estop=b{cfg.estop_btn}, "
            f"otonom=b{cfg.auto_toggle_btn}"
        )
    try:
        while True:
            loop_start = time.time()
            reader.poll()

            # --- Direksiyon ---
            steer_n = norm_axis(reader.axis(cfg.steer_axis))
            if abs(steer_n) < cfg.steer_deadzone:
                steer_n = 0.0
            if cfg.invert_steer:
                steer_n = -steer_n
            steer_deg = steer_n * MAX_STEER_ANGLE

            # --- Pedallar ---
            if cfg.pedal_mode == "split":
                # Birleşik pedal: tek eksen; negatif=gaz, pozitif=fren
                raw = reader.axis(cfg.pedal_axis)
                n = norm_axis(raw)   # -1..1
                if cfg.swap_pedals:
                    n = -n
                throttle = max(0.0, -n)  # negatif yöne basınca gaz
                brake = max(0.0, n)      # pozitif yöne basınca fren
            else:
                throttle = pedal_value(reader.axis(cfg.throttle_axis, cfg.throttle_rest),
                                       cfg.throttle_rest, cfg.invert_throttle)
                brake = pedal_value(reader.axis(cfg.brake_axis, cfg.brake_rest),
                                    cfg.brake_rest, cfg.invert_brake)
            if throttle < cfg.pedal_deadzone:
                throttle = 0.0
            if brake < cfg.pedal_deadzone:
                brake = 0.0
            throttle_pct = throttle * 100.0 * cfg.throttle_scale
            brake_pct = brake * 100.0

            # --- Buton geçişleri: vites / el freni / e-stop ---
            if edge(cfg.gear_fwd_btn):
                gear = GEAR_FORWARD
                print(" -> Vites: İLERİ")
            if edge(cfg.gear_rev_btn):
                gear = GEAR_REVERSE
                print(" -> Vites: GERİ")
            if edge(cfg.gear_neutral_btn):
                gear = GEAR_NEUTRAL
                print(" -> Vites: BOŞ")
            if edge(cfg.handbrake_btn):
                handbrake = not handbrake
                print(f" -> El freni: {'ÇEKİLİ' if handbrake else 'SERBEST'}")

            in_reverse = gear == GEAR_REVERSE
            if edge(cfg.auto_toggle_btn):
                if in_reverse and not otonom:
                    print(" -> Otonom devre reddedildi: geri viteste")
                else:
                    otonom = not otonom
                    bus.send(can.Message(
                        arbitration_id=0x500,
                        data=bytes([1 if otonom else 0]) + bytes(7),
                        is_extended_id=False))
                    print(f" -> {'OTONOM devraldı (teleop sustu)' if otonom else 'MANUEL devraldı'}")

            estop = cfg.estop_btn is not None and cfg.estop_btn >= 0 \
                and reader.button(cfg.estop_btn)
            if estop:
                # Acil: tam fren, boş vites, gaz kes
                throttle_pct, brake_pct, gear = 0.0, 100.0, GEAR_NEUTRAL

            if otonom:
                if cfg.verbose:
                    sys.stdout.write("\rOTONOM aktif: teleop CAN komutu yazmıyor      ")
                    sys.stdout.flush()
                dt = time.time() - loop_start
                if dt < period:
                    time.sleep(period - dt)
                continue

            # --- CAN frame'leri (control.py ile birebir) ---
            throttle_raw = int(max(0, min(100, throttle_pct)) * 100)
            brake_raw = int(max(0, min(100, brake_pct)))
            data_ctrl = throttle_raw.to_bytes(2, "little") + bytes([gear]) \
                + brake_raw.to_bytes(1, "little") + bytes(4)

            steer_clamped = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_deg))
            steer_raw = int((steer_clamped + 500) * 10)
            data_steer = steer_raw.to_bytes(2, "little") + bytes(6)

            try:
                bus.send(can.Message(arbitration_id=0x100, data=data_ctrl,
                                     is_extended_id=False))
                bus.send(can.Message(arbitration_id=0x201, data=data_steer,
                                     is_extended_id=False))
                bus.send(can.Message(arbitration_id=0x102,
                                     data=bytes([1 if (handbrake or estop) else 0]) + bytes(7),
                                     is_extended_id=False))
            except can.CanError as e:
                print(f"CAN gönderim hatası: {e}", file=sys.stderr)

            if cfg.verbose:
                sys.stdout.write(
                    f"\rdir:{steer_clamped:+6.1f}° gaz:{throttle_pct:5.1f}% "
                    f"fren:{brake_pct:5.1f}% vites:{gear} "
                    f"el-fr:{int(handbrake)} estop:{int(bool(estop))}   ")
                sys.stdout.flush()

            dt = time.time() - loop_start
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\nDurduruluyor: fren 100%, boş vites...")
        # Güvenli kapanış: birkaç frame tam fren + boş
        for _ in range(10):
            stop_ctrl = (0).to_bytes(2, "little") + bytes([GEAR_NEUTRAL]) \
                + (100).to_bytes(1, "little") + bytes(4)
            try:
                bus.send(can.Message(arbitration_id=0x100, data=stop_ctrl,
                                     is_extended_id=False))
            except can.CanError:
                pass
            time.sleep(0.02)


def main():
    p = argparse.ArgumentParser(description="PC Game Controller / Snopy V5H -> CAN direksiyon köprüsü")
    p.add_argument("--dev", default="/dev/input/js0", help="joystick cihazı")
    p.add_argument("--channel", default="vcan0", help="CAN arayüzü (sim:vcan0, gerçek:can0)")
    p.add_argument("--rate", type=float, default=50.0, help="gönderim frekansı (Hz)")
    p.add_argument("--kalibre", action="store_true", help="kalibrasyon modu (eksenleri göster)")
    p.add_argument("--verbose", action="store_true", help="canlı durum bas")
    # Eksen mapping
    p.add_argument("--steer-axis", type=int, default=0)
    # Pedal modu: "split" = tek eksen (negatif=gaz, pozitif=fren); "separate" = ayrı eksen
    p.add_argument("--pedal-mode", default="split", choices=["split", "separate"],
                   help="split: gaz/fren tek eksende (neg=gaz, pos=fren); separate: ayrı eksenler")
    p.add_argument("--pedal-axis", type=int, default=1,
                   help="split modda birleşik pedal ekseni")
    p.add_argument("--swap-pedals", action="store_true",
                   help="split modda gaz/fren yönünü ters çevir")
    p.add_argument("--throttle-axis", type=int, default=1,
                   help="separate modda gaz ekseni")
    p.add_argument("--brake-axis", type=int, default=2,
                   help="separate modda fren ekseni")
    p.add_argument("--throttle-rest", type=int, default=-32767,
                   help="separate modda gaz pedalı serbestteki ham değer")
    p.add_argument("--brake-rest", type=int, default=-32767,
                   help="separate modda fren pedalı serbestteki ham değer")
    p.add_argument("--invert-steer", action="store_true")
    p.add_argument("--invert-throttle", action="store_true",
                   help="separate modda gaz yönünü ters çevir")
    p.add_argument("--invert-brake", action="store_true",
                   help="separate modda fren yönünü ters çevir")
    p.add_argument("--steer-deadzone", type=float, default=0.05)
    p.add_argument("--pedal-deadzone", type=float, default=0.03)
    p.add_argument("--throttle-scale", type=float, default=1.0,
                   help="gaz tavanı (0..1); örn 0.3 = max %%30 güç (güvenli test)")
    # Buton mapping (-1 = devre dışı)
    p.add_argument("--gear-fwd-btn", type=int, default=0,
                   help="vites ileri butonu (b0=paddle UP)")
    p.add_argument("--gear-rev-btn", type=int, default=1,
                   help="vites geri butonu (b1=paddle DOWN)")
    p.add_argument("--gear-neutral-btn", type=int, default=-1)
    p.add_argument("--handbrake-btn", type=int, default=3,
                   help="el freni toggle butonu (b3)")
    p.add_argument("--estop-btn", type=int, default=-1)
    p.add_argument("--auto-toggle-btn", type=int, default=2,
                   help="otonom/manuel devir butonu; b2 = fiziksel buton 3, -1 = kapalı")
    cfg = p.parse_args()

    if not os.path.exists(cfg.dev):
        print(f"HATA: {cfg.dev} yok. Direksiyon takılı mı? "
              f"NoMachine ile uzak makineye forward ettin mi? "
              f"`ls /dev/input/js*` ve `jstest {cfg.dev}` ile kontrol et.",
              file=sys.stderr)
        sys.exit(1)

    reader = JoystickReader(cfg.dev)

    if cfg.kalibre:
        kalibrasyon_modu(reader)
        reader.close()
        return

    try:
        bus = can.interface.Bus(channel=cfg.channel, interface="socketcan")
    except Exception as e:
        print(f"HATA: CAN bus açılamadı ({cfg.channel}): {e}\n"
              f"vcan0 yoksa: ./setup-vcan.sh veya baslat.sh çalıştır.",
              file=sys.stderr)
        reader.close()
        sys.exit(1)

    try:
        send_loop(reader, bus, cfg)
    finally:
        bus.shutdown()
        reader.close()


if __name__ == "__main__":
    main()
