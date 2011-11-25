set terminal postscript eps
set output "rmse_particles.eps"
set multiplot
set xrange [0 : 12.3]
set lmargin 15
set rmargin 2
set size 1,0.5
set origin 0,0.5
unset key
set xlabel "Time"
set ylabel "RMSE"
plot "data.dat" using 1:3 with lines
set origin 0,0
set log y
set ylabel "Number of Particles"
plot "data.dat" using 1:2 with lines
set nomultiplot