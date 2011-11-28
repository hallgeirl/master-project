set term postscript eps enhanced
#set output "astar_vs_dijkstra.eps"
set   autoscale                        # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
#set logscale y 2
set datafile missing '-'
set yrange [1 : 1.7]
set key left top
set ytic auto                          # set ytics automatically
set title "Effect of grid coarsening on optimal path cost"
set xlabel "Density"
set ylabel "Cost increase relative to full resultion"
plot "densities.dat" using 2:xtic(1) title "helens\\_before" with lp lc rgb "black", \
     "densities.dat" using 3:xtic(1) title "helens\\_after" with lp lc rgb "black", \
     "densities.dat" using 4:xtic(1) title "mountains" with lp lc rgb "black", \
     "densities.dat" using 5:xtic(1) title "trondheim" with lp lc rgb "black"


