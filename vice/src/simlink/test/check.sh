#!/bin/sh
# Gesamttest simlink: IEEE (xpet + ieeedev), IEC (xdrive + iechost) und
# Relais (xpet + relay + xdrive).
#
#   src/simlink/test/check.sh [VICE-Baumverzeichnis]   (Standard: ../../..)
#
# Erwartet einen gebauten VICE-Baum (xpet, xdrive, c1541) mit data/.
# Jeder Test laeuft zweimal; die Busspuren muessen gleich sein.

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
SL=$(cd "$HERE/.." && pwd)
TREE=$(cd "${1:-$SL/../..}" && pwd)
SRC=$TREE/src
DATA=$TREE/data
WORK=$(mktemp -d "${TMPDIR:-/tmp}/slcheck.XXXXXX")
FAIL=0

ok()   { echo "  ok    $*"; }
bad()  { echo "  FEHLER $*"; FAIL=1; }

cc -O2 -std=c11 -I"$SL" -o "$WORK/sltest"  "$SL/test/sltest.c"  "$SL/simlink.c" -lpthread || exit 1
cc -O2 -std=c11 -I"$SL" -o "$WORK/ieeedev" "$SL/test/ieeedev.c" "$SL/simlink.c" -lpthread || exit 1
cc -O2 -std=c11 -I"$SL" -o "$WORK/iechost" "$SL/test/iechost.c" "$SL/simlink.c" -lpthread || exit 1
cc -O2 -std=c11 -I"$SL" -o "$WORK/relay"   "$SL/test/relay.c"   "$SL/simlink.c" -lpthread || exit 1

cd "$WORK" || exit 1

# Testdaten: kleines und groesseres BASIC-Programm
printf '\001\004\017\004\012\000\231"HALLO"\000\000\000' > test
python3 - <<'EOF'
addr = 0x0401
out = bytearray([1, 4])
for n in range(1, 120):
    body = bytes([0x8f]) + (' ZEILE %d ABCDEFGHIJKLMNOPQRSTUVWXYZ' % n).encode() + b'\0'
    nxt = addr + 4 + len(body)
    out += nxt.to_bytes(2, 'little') + n.to_bytes(2, 'little') + body
    addr = nxt
out += b'\0\0'
open('big', 'wb').write(out)
EOF
"$SRC/c1541" -format "check,01" d64 disk.d64 -write test test -write big big >/dev/null 2>&1

shm_rm() { rm -f "/dev/shm/simlink-$1"; }

echo "Bibliothek:"
"$WORK/sltest" listener lib1 2 20000 > l.txt 2>&1 & L=$!
sleep 0.3
"$WORK/sltest" talker lib1 1 20000 > t.txt 2>&1
wait $L
if grep -q "Fehler 0," l.txt; then ok "Handshake 20000 Bytes"; else bad "Handshake"; cat l.txt; fi
shm_rm lib1

echo "IEEE (xpet + ieeedev):"
for i in 1 2; do
    shm_rm chk488
    rm -f bigcopy
    "$WORK/ieeedev" -v -v -s 500 chk488 > ieee$i.trace 2>&1 &
    D=$!
    sleep 0.3
    timeout 300 "$SRC/xpet" -default -directory "$DATA" +sound -model 4032 \
        -drive8type 0 +busdevice8 -ieeesimlink chk488 -warp -limitcycles 30000000 \
        -keybuf 'load"big",8\nsave"bigcopy",8\n' > ieee$i.log 2>&1
    kill $D 2>/dev/null; wait $D 2>/dev/null
    if cmp -s big bigcopy; then ok "Lauf $i: LOAD/SAVE 5013 Bytes bytegleich"; else bad "Lauf $i: Kopie weicht ab"; fi
done
if cmp -s ieee1.trace ieee2.trace; then ok "Busspuren gleich ($(wc -l < ieee1.trace) Wechsel)"; else bad "Busspuren verschieden"; fi
shm_rm chk488

echo "IEEE gemischt (xpet + 2031 auf 8 + ieeedev auf 9):"
for i in 1 2; do
    shm_rm chkmix
    cp disk.d64 mix$i.d64
    rm -f mixcopy$i
    "$WORK/ieeedev" -v -v -u 9 chkmix > mix$i.trace 2>&1 &
    D=$!
    sleep 0.3
    timeout 300 "$SRC/xpet" -default -directory "$DATA" +sound -model 4032 \
        -drive8type 2031 -drive8truedrive -8 mix$i.d64 +busdevice8 +busdevice9 \
        -ieeesimlink chkmix -warp -limitcycles 60000000 \
        -keybuf 'load"big",8\nsave"m",9\nload"m",9\nsave"back",8\n' > mix$i.log 2>&1
    kill $D 2>/dev/null; wait $D 2>/dev/null
    "$SRC/c1541" -attach mix$i.d64 -read back mixcopy$i >/dev/null 2>&1
    if cmp -s big m && cmp -s big mixcopy$i; then ok "Lauf $i: 8 -> 9 -> 8 bytegleich"; else bad "Lauf $i"; fi
    rm -f m
done
if cmp -s mix1.trace mix2.trace; then ok "Busspuren gleich ($(wc -l < mix1.trace) Wechsel)"; else bad "Busspuren verschieden"; fi
shm_rm chkmix

echo "IEC (xdrive + 1541 + iechost):"
for i in 1 2; do
    shm_rm chkiec
    cp disk.d64 run$i.d64
    rm -f back$i
    "$WORK/iechost" -v -v -w 3 chkiec status load test t$i save big neu load neu back$i \
        > iec$i.out 2> iec$i.trace &
    H=$!
    sleep 0.3
    timeout 900 "$SRC/xdrive" -default -directory "$DATA" +sound -iecsimlink chkiec \
        -8 run$i.d64 -warp -limitcycles 45000000 > iec$i.log 2>&1
    wait $H
    if cmp -s test t$i && cmp -s big back$i; then
        ok "Lauf $i: LOAD, SAVE, LOAD bytegleich"
    else
        bad "Lauf $i"; cat iec$i.out
    fi
done
if cmp -s iec1.trace iec2.trace; then ok "Busspuren gleich ($(wc -l < iec1.trace) Wechsel)"; else bad "Busspuren verschieden"; fi
shm_rm chkiec

echo "Relais (xpet + relay + xdrive mit 1541):"
# LOAD, SAVE, Fehlerkanal nach fehlender Datei. xdrive endet von selbst,
# damit VICE die Spur zurueckschreibt (sonst fehlt der Verzeichniseintrag).
for i in 1 2; do
    shm_rm chkri; shm_rm chkrc
    cp disk.d64 rel$i.d64
    "$WORK/relay" -v chkri chkrc > rel$i.trace 2>&1 &
    R=$!
    sleep 0.3
    timeout 900 "$SRC/xdrive" -default -directory "$DATA" +sound -iecsimlink chkrc \
        -8 rel$i.d64 -warp -limitcycles 48000000 > reld$i.log 2>&1 &
    X=$!
    sleep 0.3
    timeout 900 "$SRC/xpet" -default -directory "$DATA" +sound -model 4032 -drive8type 0 \
        +busdevice8 -ieeesimlink chkri -warp -limitcycles 45000000 \
        -keybuf 'load"big",8\nsave"neu",8\n10 open1,8,15:input#1,a,b$:close1\nload"nix",8\nrun\n' \
        > relp$i.log 2>&1
    wait $X
    sleep 0.5
    kill $R 2>/dev/null
    wait $R 2>/dev/null
    rm -f neu$i
    "$SRC/c1541" rel$i.d64 -read neu neu$i >/dev/null 2>&1
    if cmp -s big neu$i && grep -q "LOAD) geholt: $(wc -c < big) Bytes" rel$i.trace \
       && grep -q "Kanal 15 geholt: 25 Bytes" rel$i.trace; then
        ok "Lauf $i: LOAD, SAVE bytegleich, Fehlerkanal nach FILE NOT FOUND"
    else
        bad "Lauf $i"; grep -v "ATN" rel$i.trace
    fi
done
if cmp -s rel1.trace rel2.trace; then ok "Relais-Ablauf gleich ($(wc -l < rel1.trace) Zeilen)"; else bad "Relais-Ablauf verschieden"; fi
shm_rm chkri; shm_rm chkrc

if [ $FAIL = 0 ]; then
    echo "Alle Tests bestanden."
    rm -rf "$WORK"
else
    echo "Fehler; Arbeitsverzeichnis: $WORK"
fi
exit $FAIL
