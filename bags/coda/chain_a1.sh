cd /home/duccio/Desktop/jo_sim_sins
for T in A1 A1r2 A1r3; do STEREO=1 bash eval/scripts/coda_ab.sh $T w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5; done
echo "A1 DONE"
