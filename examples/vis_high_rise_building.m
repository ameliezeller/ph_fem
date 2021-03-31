function vis_high_rise_building(nodes,elems)



%% associate numbers to nodes
len_no = length(nodes(:,1));
coord_nodes = [nodes, [1:1:len_no]']

%%
figure
hold all
grid on
len_el = length(elems(:,1));
for idx_el = 1:len_el
    plot3(coord_nodes(elems(idx_el,1:2),1)',...
        coord_nodes(elems(idx_el,1:2),2)',...
        coord_nodes(elems(idx_el,1:2),3)')
end
view(15,30)
end

