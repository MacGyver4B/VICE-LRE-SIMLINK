#!/bin/sh
# Zwei Prozesse: Talker 1 MHz (PET), Listener 2 MHz (Laufwerk); fuenf Laeufe,
# die emulierte Zeit muss jedes Mal identisch sein.
set -e
cd "$(dirname "$0")/.."
[ -x ./sltest ] || cc -O2 -std=c11 -I. -o sltest test/sltest.c simlink.c -lpthread
for k in 1 2 3 4 5; do
  ./sltest listener r$k 2 20000 > /tmp/sl_l$k &
  L=$!
  ./sltest talker r$k 1 20000
  wait $L && cat /tmp/sl_l$k || { cat /tmp/sl_l$k; echo "FEHLER"; exit 1; }
done
# Ringpuffer: Leser seltener als der Ring gross ist
for every in 1 50 200; do
  ./sltest counter p$every 5000 $every > /tmp/sl_c &
  C=$!
  ./sltest pulser p$every 5000
  wait $C && cat /tmp/sl_c || { cat /tmp/sl_c; echo "FEHLER"; exit 1; }
done
echo "alle Tests bestanden"
