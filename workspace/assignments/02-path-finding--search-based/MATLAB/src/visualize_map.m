function visualize_map(map,path,visit_nodes)
%This function visualizes the 2D grid map 
%consist of obstacles/start point/target point/optimal path.
    set(gcf, 'Renderer', 'painters');
    set(gcf, 'Position', [500, 50, 700, 700]);
    hold on
    
    sz_map = max(max(map));
    
    % obst
    obst_sz = max(2500/sz_map, 36);
    obst_cnt = 2: size(map, 1) - 1;
    obst_color = [55,184,157]/255;
    scatter(map(obst_cnt, 1)-0.5,map(obst_cnt, 2)-0.5,obst_sz,obst_color,'filled');

    % start point
    scatter(map(1, 1)-0.5, map(1, 2)-0.5,'b','*');

    % target point
	scatter(map(size(map, 1), 1)-0.5, map(size(map, 1), 2)-0.5, 'r','*');

    % optimal path
    if size(path,1) > 1
        path_cnt = 2:size(path,1)-1;
        scatter(path(path_cnt,1)-0.5,path(path_cnt,2)-0.5,'b');
        plot(path(:,1)-0.5,path(:,2)-0.5,'b')
    end
    
    if size(visit_nodes,1) > 1
        % openlist
        node_sz = 10000/sz_map;
        node = visit_nodes(visit_nodes(:,1)==1, 2:3);
        scatter1 = scatter(node(:,1)-0.5,node(:,2)-0.5,node_sz,'gs','filled');
        alpha(scatter1,.1)

        % closelist
        node = visit_nodes(visit_nodes(:,1)==0, 2:3);
        scatter2 = scatter(node(:,1)-0.5,node(:,2)-0.5,node_sz,'bs','filled');
        alpha(scatter2,.1)
    end

    grid on 
    set(gca,'xtick',0:1:200)
    set(gca,'ytick',0:1:200)
end
