# gnuplot draw_all_robocom.gp | eps2eps draw_all_robocom.eps draw_all_robocom_fixed.eps | convert -density 600 draw_all_robocom_fixed.eps draw_all_robocom.png | eog draw_all_robocom.png &

#set term wxt 
set terminal postscript eps color enhanced "Helvetica" 16
set output "draw_trackerrors.eps"

set datafile separator ","

set lmargin 5
set rmargin 1

set size 0.65,0.4

set border linewidth 2

set key inside top right horizontal
set key font ",14"

set style line 1 lt 1 lw 3 lc rgb "red"
set style line 2 lt 1 lw 3 lc rgb "blue"
set style line 3 lt 1 lw 3 lc rgb "green"

set style line 100 lt 1 lc rgb "gray" lw 2
set style line 101 lt 0.5 lc rgb "gray" lw 1

set xrange [0.5:23.5]

set ytics autofreq 5
set xtics autofreq 

set mxtics 5

set grid mxtics xtics ls 101, ls 101

set autoscale y

set format y "%.0f"
set format x "%.0f"

set xlabel "landmark [#]" offset 0, 0.5
set ylabel "error [cm]" offset 1, 0


plot "../csv/deltaTracksFHPnorm.csv" using 1:($2*100) with lines ls 1 title "FHP", \
     "../csv/deltaTracksARnorm.csv" using 1:($2*100) with lines ls 2 title "FSP"
