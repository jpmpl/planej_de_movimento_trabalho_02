T = readtable('data/data_td_trap.csv');
a = T(6,:).v3_y;
T(6,:).v3_y = T(6,:).v2_y;
T(6,:).v2_y = a;
for i = 1:size(T,1)
    pgon = polyshape([T(i,:).v0_x T(i,:).v1_x T(i,:).v2_x T(i,:).v3_x],...
        [T(i,:).v0_y T(i,:).v1_y T(i,:).v2_y T(i,:).v3_y]);
    plot(pgon)
    hold on
end


%%

rbt = readtable('data/data_td.csv');
m_gtb = matches(rbt.is_go_to_base, 'True');
m_tm = matches(rbt.is_tangent_movement, 'True');
m_vm = matches(rbt.is_vertical_movement, 'True');
m_gtnc = matches(rbt.is_go_to_next_cell, 'True');

a = plot(rbt.x(m_gtb),rbt.y(m_gtb),'ob');%,'DisplayName','GoToBase');
hold on
b = plot(rbt.x(m_tm),rbt.y(m_tm),'og');%,'DisplayName','TangentMov');
hold on
c = plot(rbt.x(m_vm),rbt.y(m_vm),'or');%,'DisplayName','VertMov');
hold on
d = plot(rbt.x(m_gtnc),rbt.y(m_gtnc),'om');%,'DisplayName','GoToNextCell');

%lgd = legend;
%lgd.NumColumns = 2;
legend([a,b,c,d],'GoToBase','TangentMov','VertMov','GoToNextCell','Location','northeastoutside')
title('Trapezoidal Decomposition + Coverage')