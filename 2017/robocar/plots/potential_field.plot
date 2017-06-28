set terminal wxt enhanced font "Arial,12"

# set terminal postscript eps enhanced color font "Arial,16"
# set output ""

set isosamples 50
set hidden3d
set contour

range_max = 90
range_mid = range_max / 2

set xrange [-range_max:range_max]
set yrange [-range_max:range_max]
set zrange [-10:10]

U(x,y) = -atanh((x**2+y**2) / (range_max**2) - 1)

splot abs(U(x,y))

