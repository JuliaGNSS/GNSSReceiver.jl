#!/bin/bash
# usage: analyze_run.sh <bits.log>
L=$1; R=$1.records
O=$(awk '$1=="T" {print $3; exit}' "$L")
echo "first_boundary=$O  T:$(grep -c '^T ' "$L") A:$(grep -c '^A ' "$L") R:$(grep -c '^R ' "$R") C:$(grep -c '^C ' "$L")"
echo "== A lines, non-noise channels (stream_s ch old->new)"
grep "^A " "$L" | awk -v o=$O '$2!=1 {printf("%8.3f ch%s %s -> %s\n", ($5-o)/4e6, $2, $3, $4)}' | head -30
echo "== T gaps > 80 ms (stream_s gap_s dumps [compile_s gc_s])"
grep "^T " "$L" | awk -v o=$O '{if (p>0){g=($2-p)/1e9; if (g>0.08) printf("%8.3f %7.3f %5d %s %s\n", ($3-o)/4e6, g, $4, (NF>=7)?sprintf("%.3f",($7-pc)/1e9):"", (NF>=8)?sprintf("%.3f",($8-pg)/1e9):"")}; p=$2; pc=$7; pg=$8}' | head -40
echo "== L lines (every 30th)"
grep "^L " "$L" | awk -v o=$O 'NR%30==1 {printf("%8.3f gaps %s stale %s dropped %s skipped %s\n", ($2-o)/4e6, $3, $4, $5, $6)}' | head -10
echo "== C lines: lost per channel (every 20th)"
grep "^C " "$L" | awk -v o=$O 'NR%20==1 {printf("%8.3f ch%s prn %s lost %s\n", ($4-o)/4e6, $2, $3, $5)}' | head -12
echo "== per-assignment first 100 dumps |P|/|EL| (ch prn stream_s)"
grep "^A " "$L" | awk '$2!=1 && $4>0 {print $2, $4, $5}' | head -12 | while read ch prn b; do
  awk -v ch=$ch -v prn=$prn -v b=$b -v o=$O '$1=="R" && $2==ch && $3==prn && $4>=b {n++; P=sqrt($9*$9+$10*$10); EL=(sqrt($7*$7+$8*$8)+sqrt($11*$11+$12*$12))/2; p+=P;e+=EL; if(n==100)exit} END{if(n>0 && e>0) printf("ch%d prn %2d at %8.3f s: ratio %.2f  |P| %.2fM (n=%d)\n", ch, prn, (b-o)/4e6, p/e, p/n/1e6, n); else printf("ch%d prn %2d at %8.3f s: no dumps\n", ch, prn, (b-o)/4e6)}' "$R"
done
echo "== timing totals (after first T record)"
awk '$1=="T" {if(!n){t0=$2;c0=$7;g0=$8} n++;t=$2;c=$7;g=$8;fields=NF} END {if(n) {printf "wall %.3f s; chunks %d",(t-t0)/1e9,n; if(fields>=8) printf "; compile %.6f s; GC %.6f s",(c-c0)/1e9,(g-g0)/1e9; else printf "; compile/GC counters unavailable"; print ""}}' "$L"
