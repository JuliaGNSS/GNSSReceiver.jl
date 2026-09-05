#!/bin/bash
# usage: stall_ctx.sh <bits.log> <min_gap_s> ; prints T-line context around each gap > min_gap
L=$1; G=$2
grep "^T " $L | awk -v G=$G '
{ n++; w[n]=$2; b[n]=$3; d[n]=$4; li[n]=$5; sc[n]=$6; cc[n]=(NF>=7)?$7:0 }
END {
  for (i=2;i<=n;i++) if ((w[i]-w[i-1])/1e9 > G) {
    printf("---- gap %.3f s at chunk %d (consumed %.3f s)\n", (w[i]-w[i-1])/1e9, i, sc[i]/4e6);
    for (j=i-6;j<=i+8 && j<=n;j++) if (j>=2)
      printf("  %+8.3f s wall | dumps %4d | latest_idx +%9d | consumed +%6d | boundary +%9d\n",
        (w[j]-w[i-1])/1e9, d[j], li[j]-li[j-1], sc[j]-sc[j-1], b[j]-b[j-1]);
  }
}' | head -${3:-120}
