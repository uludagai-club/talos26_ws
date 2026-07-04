#!/usr/bin/env python3
"""
Otomatik kalibrasyon: Her kontrol için yönerge verir, 5 saniye okur,
min/max/rest/buton tespit eder.
"""
import os, struct, select, sys, time

JS_EVENT_FMT = "<IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
AXIS_MAX = 32767

def read_js(dev_path, duration, label):
    """duration saniye boyunca joystick oku, eksen min/max ve basılan butonları döndür."""
    fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
    axes = {}
    axes_min = {}
    axes_max = {}
    buttons_pressed = set()
    
    start = time.time()
    print(f"\n{'='*60}")
    print(f"  >>> {label}")
    print(f"  >>> {duration} saniye süren var, BAŞLA!")
    print(f"{'='*60}")
    
    while time.time() - start < duration:
        r, _, _ = select.select([fd], [], [], 0.01)
        if not r:
            remaining = duration - (time.time() - start)
            sys.stdout.write(f"\r  Kalan: {remaining:.0f}s  ")
            sys.stdout.flush()
            continue
        try:
            buf = os.read(fd, JS_EVENT_SIZE * 64)
        except BlockingIOError:
            continue
        offset = 0
        while offset + JS_EVENT_SIZE <= len(buf):
            _t, value, etype, number = struct.unpack(JS_EVENT_FMT, buf[offset:offset+JS_EVENT_SIZE])
            offset += JS_EVENT_SIZE
            etype &= 0x7F  # init bayrağını at
            if etype == 0x02:  # axis
                if number not in axes:
                    axes[number] = value
                    axes_min[number] = value
                    axes_max[number] = value
                else:
                    axes[number] = value
                    axes_min[number] = min(axes_min[number], value)
                    axes_max[number] = max(axes_max[number], value)
            elif etype == 0x01:  # button
                if value:
                    buttons_pressed.add(number)
    
    os.close(fd)
    print(f"\r  Bitti!{'':30}")
    return axes, axes_min, axes_max, buttons_pressed


def main():
    dev = "/dev/input/js0"
    if not os.path.exists(dev):
        print(f"HATA: {dev} bulunamadı!")
        sys.exit(1)
    
    print("\n" + "="*60)
    print("  TALOS DİREKSİYON SETİ OTOMATİK KALİBRASYON")
    print("  PC Game Controller (VID:11ff PID:3331)")
    print("="*60)
    
    results = {}
    
    # 1) Boşta bekleme (rest durumu)
    _, _, _, _ = read_js(dev, 3, "HİÇBİR ŞEYE DOKUNMA - boşta bırak (rest tespiti)")
    rest_axes, rest_min, rest_max, _ = read_js(dev, 3, "HİÇBİR ŞEYE DOKUNMA - devam (rest tespiti)")
    print("\n  Rest değerleri:")
    for n in sorted(rest_axes.keys()):
        print(f"    a{n}: {rest_axes[n]:+6d} (min={rest_min[n]:+6d} max={rest_max[n]:+6d})")
    results['rest'] = dict(rest_axes)
    
    # 2) Direksiyon
    _, smin, smax, _ = read_js(dev, 6, "DİREKSİYONU SOLA ve SAĞA TAM ÇEVİR")
    print("\n  Direksiyon tespiti:")
    for n in sorted(smin.keys()):
        rng = smax[n] - smin[n]
        if rng > 1000:
            print(f"    a{n}: min={smin[n]:+6d} max={smax[n]:+6d} range={rng}  <<< MUHTEMELEN DİREKSİYON")
    results['steer'] = {'min': dict(smin), 'max': dict(smax)}
    
    # 3) Gaz pedalı
    _, gmin, gmax, _ = read_js(dev, 5, "GAZ PEDALINA TAM BAS, sonra BIRAK")
    print("\n  Gaz tespiti:")
    for n in sorted(gmin.keys()):
        rng = gmax[n] - gmin[n]
        if rng > 1000:
            print(f"    a{n}: min={gmin[n]:+6d} max={gmax[n]:+6d} range={rng}  <<< MUHTEMELEN GAZ")
    results['throttle'] = {'min': dict(gmin), 'max': dict(gmax)}
    
    # 4) Fren pedalı
    _, fmin, fmax, _ = read_js(dev, 5, "FREN PEDALINA TAM BAS, sonra BIRAK")
    print("\n  Fren tespiti:")
    for n in sorted(fmin.keys()):
        rng = fmax[n] - fmin[n]
        if rng > 1000:
            print(f"    a{n}: min={fmin[n]:+6d} max={fmax[n]:+6d} range={rng}  <<< MUHTEMELEN FREN")
    results['brake'] = {'min': dict(fmin), 'max': dict(fmax)}
    
    # 5) Butonlar
    _, _, _, btns = read_js(dev, 8, "TÜM BUTONLARI TEK TEK BAS (paddle shift, otonom, vs)")
    print(f"\n  Basılan butonlar: {sorted(btns)}")
    results['buttons'] = sorted(btns)
    
    # Sonuç
    print("\n" + "="*60)
    print("  KALİBRASYON SONUCU")
    print("="*60)
    
    # En büyük range'a sahip ekseni direksiyon olarak bul
    steer_axis = None
    steer_range = 0
    for n in results['steer']['min']:
        rng = results['steer']['max'][n] - results['steer']['min'][n]
        if rng > steer_range:
            steer_range = rng
            steer_axis = n
    
    # Gaz: rest'ten en çok farklılaşan, direksiyon olmayan
    gas_axis = None
    gas_range = 0
    for n in results['throttle']['min']:
        if n == steer_axis:
            continue
        rng = results['throttle']['max'][n] - results['throttle']['min'][n]
        if rng > gas_range:
            gas_range = rng
            gas_axis = n
    
    # Fren: rest'ten en çok farklılaşan, direksiyon ve gaz olmayan
    brake_axis = None
    brake_range = 0
    for n in results['brake']['min']:
        if n in (steer_axis, gas_axis):
            continue
        rng = results['brake']['max'][n] - results['brake']['min'][n]
        if rng > brake_range:
            brake_range = rng
            brake_axis = n
    
    print(f"  Direksiyon: a{steer_axis} (range={steer_range})")
    print(f"  Gaz:        a{gas_axis} (range={gas_range})")
    print(f"  Fren:       a{brake_axis} (range={brake_range})")
    print(f"  Butonlar:   {results['buttons']}")
    
    # Rest değerleri (pedal idle konumu)
    gas_rest = results['rest'].get(gas_axis, 0) if gas_axis is not None else 0
    brake_rest = results['rest'].get(brake_axis, 0) if brake_axis is not None else 0
    steer_rest = results['rest'].get(steer_axis, 0) if steer_axis is not None else 0
    
    print(f"\n  Gaz rest:   {gas_rest}")
    print(f"  Fren rest:  {brake_rest}")
    print(f"  Dir rest:   {steer_rest}")
    
    # Yön tespiti
    if gas_axis is not None:
        gas_max_v = results['throttle']['max'][gas_axis]
        gas_min_v = results['throttle']['min'][gas_axis]
        if abs(gas_max_v - gas_rest) > abs(gas_min_v - gas_rest):
            print(f"  Gaz yönü:   basınca ARTIYOR ({gas_rest} -> {gas_max_v})")
            gas_invert = False
        else:
            print(f"  Gaz yönü:   basınca AZALIYOR ({gas_rest} -> {gas_min_v})")
            gas_invert = True
    
    if brake_axis is not None:
        brake_max_v = results['brake']['max'][brake_axis]
        brake_min_v = results['brake']['min'][brake_axis]
        if abs(brake_max_v - brake_rest) > abs(brake_min_v - brake_rest):
            print(f"  Fren yönü:  basınca ARTIYOR ({brake_rest} -> {brake_max_v})")
            brake_invert = False
        else:
            print(f"  Fren yönü:  basınca AZALIYOR ({brake_rest} -> {brake_min_v})")
            brake_invert = True
    
    # Önerilen komut
    print(f"\n{'='*60}")
    print("  ÖNERİLEN KOMUT:")
    print(f"{'='*60}")
    
    invert_steer = ""
    if steer_axis is not None and steer_rest != 0:
        print(f"  NOT: Direksiyon rest={steer_rest}, merkez 0 olmalı. jscal lazım olabilir.")
    
    cmd_parts = [
        "python3 direksiyon_teleop.py",
        f"--dev {dev}",
        "--channel vcan0",
        "--verbose",
    ]
    if steer_axis is not None:
        cmd_parts.append(f"--steer-axis {steer_axis}")
    if gas_axis is not None:
        cmd_parts.append(f"--throttle-axis {gas_axis}")
        cmd_parts.append(f"--throttle-rest {gas_rest}")
    if brake_axis is not None:
        cmd_parts.append(f"--brake-axis {brake_axis}")
        cmd_parts.append(f"--brake-rest {brake_rest}")
    
    # Buton önerisi
    if results['buttons']:
        cmd_parts.append(f"--auto-toggle-btn {results['buttons'][2] if len(results['buttons']) > 2 else results['buttons'][-1]}")
        if len(results['buttons']) >= 2:
            cmd_parts.append(f"--gear-fwd-btn {results['buttons'][0]}")
            cmd_parts.append(f"--gear-rev-btn {results['buttons'][1]}")
    
    print("  " + " \\\n    ".join(cmd_parts))
    print()

if __name__ == "__main__":
    main()
