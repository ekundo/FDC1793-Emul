#!/bin/sh
# Стенды ядра ВГ93 (Icarus Verilog):
#   tb_fdc1793 -- ядро через fdc_emul.v против даташита WD1793, команды по носителю;
#   tb_fd179x  -- сверка с паспортом FD179X-01 раздел за разделом, через fdc_core.v
#                 и кодер mfm_wr.v. Долгий: секунды модельного времени, минуты счёта.
# Волны tb_fdc1793 -- по +vcd в вызове vvp. Выход не 0 -- есть провал.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
cd "$HERE/../rtl/fdc"           # DPLL.v грузит DPLL.hex по имени, из текущего каталога
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
CORE="AMD.v CRC16_D8.v DPLL.v MFMDEC.v Main_CTRL.v"
fail=0

iverilog -g2012 -o "$OUT/tb_fdc1793" ../../sim/tb_fdc1793.v ../fdc_emul.v MFMCDR.v $CORE
vvp -n "$OUT/tb_fdc1793" | tee "$OUT/tb_fdc1793.log"
grep -q "ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ" "$OUT/tb_fdc1793.log" || fail=1

iverilog -g2012 -o "$OUT/tb_fd179x" ../../sim/tb_fd179x.v ../fdc_core.v mfm_wr.v $CORE
vvp -n "$OUT/tb_fd179x" | tee "$OUT/tb_fd179x.log"
grep -q "ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ" "$OUT/tb_fd179x.log" || fail=1

exit $fail
