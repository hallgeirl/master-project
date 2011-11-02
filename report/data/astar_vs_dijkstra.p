set term postscript eps enhanced
#set output "astar_vs_dijkstra.eps"
set   autoscale                        # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set logscale y 2
set boxwidth 0.9 absolute
set style fill solid 1.00 border -1
set style histogram clustered gap 1 title offset character 0, 0, 0
set datafile missing '-'
#set yrange [0 : 8]
set style data histograms
set xtics border in scale 1,0.5 nomirror rotate by -70 offset character 0, 0, 0
set ytic auto                          # set ytics automatically
set title "A* vs Dijkstra"
set xlabel "# of nodes, # of processes per node, n"
set ylabel "Running Time (sec)"
plot "astar_vs_dijkstra.dat" using 2:xtic(1) title "A*" lc rgb "black" fill pattern 1 border rgb "black", \
     "astar_vs_dijkstra.dat" using 3 title "Dijkstra" lc rgb "black" fill pattern 2 border rgb "black"
