#!/bin/sh
# run-avr.sh - petsdsim (petSD+ in simavr) und xpet am selben simlink-Bus
#
#   VICE=/pfad/zum/vice-baum test/run-avr.sh name NODISKEMU.elf zyklen keybuf sd.img
#
# name    Busname (zugleich Namensteil der Logs und des Bildschirmfotos)
# zyklen  xpet endet nach so vielen Takten (-limitcycles) mit s_<name>.png
# keybuf  Eingabe fuer den Tastaturpuffer, z. B. 'dload"calc result",d0\r'
# sd.img  SD-Image; gearbeitet wird auf einer Kopie sd_<name>.img
#
# Umgebung: MODEL (Standard 8032), XARGS (weitere xpet-Optionen),
#           PSARGS (petsdsim-Optionen), PETSDSIM (Standard ./petsdsim),
#           TMO (Zeitgrenze in s, Standard 2400)
# Hinweis: NODISKEMU 1.3 zeigt 5 s lang den Startbildschirm, bevor es am Bus
# antwortet - die Eingabe daher z. B. mit "fori=1to15000:next:" beginnen.
# Das EEPROM ist bei jedem Lauf frisch; ohne PETSD.CFG meldet sich der
# petSD+ daher unter seiner Vorgabeadresse 11 (CONFIG_DEFAULT_ADDR), nicht 8.
set -e
[ $# -eq 5 ] || { sed -n '2,19p' "$0"; exit 1; }
N=$1; ELF=$2; C=$3; K=$4; IMG=$5
: "${VICE:?VICE=Pfad zum VICE-Baum (mit src/xpet und data) setzen}"
PETSDSIM=${PETSDSIM:-./petsdsim}; TMO=${TMO:-2400}
cp "$IMG" "sd_$N.img"
rm -f "/dev/shm/simlink-$N"
timeout "$TMO" "$PETSDSIM" $PSARGS "$ELF" "sd_$N.img" "$N" > "b_$N.log" 2>&1 &
B=$!
sleep 0.5
timeout "$TMO" "$VICE/src/xpet" -default -directory "$VICE/data" +sound \
    -model "${MODEL:-8032}" $XARGS -drive8type 0 +busdevice8 \
    -ieeesimlink "$N" -warp -limitcycles "$C" -exitscreenshot "s_$N.png" \
    -keybuf "$K" > "x_$N.log" 2>&1 || true
kill $B 2>/dev/null || true; wait $B 2>/dev/null || true
echo "Logs: b_$N.log (petsdsim), x_$N.log (xpet); Bild: s_$N.png"
