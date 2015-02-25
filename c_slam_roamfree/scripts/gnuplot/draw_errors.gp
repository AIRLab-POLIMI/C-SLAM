# gnuplot draw_all_robocom.gp | eps2eps draw_all_robocom.eps draw_all_robocom_fixed.eps | convert -density 600 draw_all_robocom_fixed.eps draw_all_robocom.png | eog draw_all_robocom.png &

#set term wxt 
set terminal postscript eps color enhanced "Helvetica" 16
set output "draw_errors.eps"

set datafile separator ","

set lmargin 6
set rmargin 1

set size 0.65,1.2

set multiplot

set border linewidth 2

set key outside top right horizontal
set key font ",14"

set style line 1 lt 1 lw 3 lc rgb "red"
set style line 2 lt 1 lw 3 lc rgb "blue"
set style line 3 lt 1 lw 3 lc rgb "green"

set autoscale xy

set format y "%.1f"

########## TRANSLATION ERROR ##########

set size 0.65,0.5
set origin 0,0.65
set tmargin 0
set bmargin 0

#set format x ""

set ylabel "translation error [m]" offset 1, 0

unset xtics
set ytics autofreq 


plot "../csv/deltaDeltaFHPnorm.csv" using 1:($2) with filledcurve x1 ls 1 title "FHP", \
     "../csv/deltaDeltaARnorm.csv" using 1:($2) with filledcurve x1 ls 2 title "FSP"


########## ANGULAR ERROR ##########

set key off

set size 0.65,0.5
set origin 0,0.15
set tmargin 0
set bmargin 0

set format x "%.0f"
set xlabel "t [s]"

set ylabel "angular error [rad]" offset 1, 0

set ytics autofreq
set xtics autofreq 10

plot "../csv/deltaDeltaFHPnorm.csv" using 1:($3) with filledcurve x1 ls 1 title "FHP", \
     "../csv/deltaDeltaARnorm.csv" using 1:($3) with filledcurve x1 ls 2 title "FSP"
  
     
unset multiplot

