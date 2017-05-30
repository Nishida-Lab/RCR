# set terminal wxt enhanced font "Arial,12"

set terminal postscript eps enhanced color font "Arial,16"
set output "./minus_atanh.eps"

set size square

set xtics nomirror
set xrange [-0.2:2.2]
set xlabel "distance [cm]"

set ytics nomirror
# set yrange [-2:2]
set ylabel "repulsive force"

set key right top reverse
set key spacing 1.5
set key width -1

set arrow 1 from -0.0,-2.0 to -0.0,+2.0 nohead dashtype "."
set arrow 2 from  2.0,-2.0 to  2.0,+2.0 nohead dashtype "."

plot -atanh(x-1) title " -atanh(x-1)"
