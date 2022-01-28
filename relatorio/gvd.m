T = readtable('data/data_gvd.csv');
p1 = plot(T.q0_x(T.isVertice==1),T.q0_y(T.isVertice==1),'ob');
hold on
p2 = plot(T.q0_x(T.isD1==0),T.q0_y(T.isD1==0));

legend([p1,p2],'GVD','Vertices','Location','northeastOutside')
title('Generalized Voronoi Diagram')

