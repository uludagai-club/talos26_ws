#!/usr/bin/env bash
# Canlı ROS üzerinde karar_bt smoke test.
#
# Önkoşul:
#   - roscore ayakta
#   - karar_bt container'ı çalışıyor (docker compose up karar-node)
#
# Çalıştır:
#   ./test/smoke_test.sh
#
# Her senaryoda /karar topic'inden 2 saniye dinler ve beklenen kararın
# bir kez basıldığını kontrol eder.

set -u

pass=0
fail=0

assert_karar () {
    local senaryo="$1"
    local beklenen="$2"
    # 2sn boyunca /karar dinle ve uniq'le
    local goz
    goz=$(timeout 2 rostopic echo -n 20 /karar 2>/dev/null | grep -E '^data:' | sort -u)
    if echo "$goz" | grep -q "\"$beklenen\""; then
        echo "  PASS [$senaryo] -> $beklenen"
        pass=$((pass+1))
    else
        echo "  FAIL [$senaryo] beklenen=$beklenen | gözlem:"
        echo "$goz" | sed 's/^/         /'
        fail=$((fail+1))
    fi
}

pub () {
    local topic="$1"; shift
    local msg="$1"
    rostopic pub -1 "$topic" std_msgs/String -- "$msg" >/dev/null 2>&1
}

# Önce ortamı temizle (none basarak)
pub /trafik_levha "none"
pub /yaya_gecidi  "none"
sleep 0.5

# ----- Senaryolar -----
echo "S1: cruise (boş ortam)"
sleep 1
assert_karar "S1" "normal"

echo "S2: Yaya 1.5m → acildurus"
for i in 1 2 3; do pub /yaya_gecidi "1.5,0.0"; sleep 0.3; done
assert_karar "S2" "acildurus"

pub /yaya_gecidi "none"; sleep 1.5

echo "S3: Yaya 3m → dur"
for i in 1 2 3; do pub /yaya_gecidi "3.0,0.0"; sleep 0.3; done
assert_karar "S3" "dur"

pub /yaya_gecidi "none"; sleep 1.0

echo "S4: Yaya 8m → slow"
for i in 1 2 3; do pub /yaya_gecidi "8.0,0.5"; sleep 0.3; done
assert_karar "S4" "slow"

pub /yaya_gecidi "none"; sleep 1.0

echo "S5: DUR 7m → slow (approach)"
pub /trafik_levha "DUR,7.0,0.0"
sleep 1.0
assert_karar "S5" "slow"

echo "S6: DUR 2.5m → dur (hold)"
pub /trafik_levha "DUR,2.5,0.0"
sleep 1.0
assert_karar "S6" "dur"

echo "S7: DUR bekleme 3s → normal"
# bekleme süresi (params: 3s) dolsun
sleep 3.5
pub /trafik_levha "DUR,2.5,0.0"
sleep 1.0
assert_karar "S7" "normal"

pub /trafik_levha "none"; sleep 1.0

echo "S8: 30 levhası → slow"
pub /trafik_levha "30,6.0,0.5"
sleep 1.0
assert_karar "S8" "slow"

pub /trafik_levha "none"; sleep 1.0

echo "S9: SAG levhası → sag"
pub /trafik_levha "SAG,3.0,0.0"
sleep 1.0
assert_karar "S9" "sag"

pub /trafik_levha "none"

echo ""
echo "===== Smoke özet: pass=$pass fail=$fail ====="
exit $fail
