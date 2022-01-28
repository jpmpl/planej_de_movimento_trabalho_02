T = readtable('data/data_prd_vertices_and_edges.csv');
figure
for i = 1:size(T,1)
    plot([T(i,:).v0_x T(i,:).v1_x],[T(i,:).v0_y T(i,:).v1_y],'b')
    hold on
end

vertices = unique([T.v0_x T.v0_y T.order],'rows');
for i = 1:size(vertices,1)
    if ~isnan(vertices(i,3))
        plot(vertices(i,1),vertices(i,2),'og','MarkerFaceColor','g')
        text(vertices(i,1),vertices(i,2),string(max(vertices(:,3))-vertices(i,3)),...
            'FontSize',14,'FontWeight','bold')
    else
        plot(vertices(i,1),vertices(i,2),'or','MarkerFaceColor','r')
    end
    hold on
end

title('PRD Graph')

%%

T = readtable('data/data_prd_27_27.csv');
figure

for i = 1:size(vertices,1)
    if ~isnan(vertices(i,3))
        plot(vertices(i,1),vertices(i,2),'og','MarkerFaceColor','g')
        text(vertices(i,1),vertices(i,2),string(max(vertices(:,3))-vertices(i,3)),...
            'FontSize',14,'FontWeight','bold')
        hold on
    end
end

s = plot(T(1,:).x,T(1,:).y,'dm','MarkerFaceColor','m');
hold on
e = plot(T(end,:).x,T(end,:).y,'dr','MarkerFaceColor','r');
hold on
plot(T(2:end-1,:).x,T(2:end-1,:).y,'--b')
legend([s,e],'Start Point','End Point','Location','northwest')
title('Robot Path')
