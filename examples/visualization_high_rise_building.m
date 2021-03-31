



%% generate map: elems_mod -> coordinates
len_no = length(nodes(:,1));
coord_nodes = [nodes, [1:1:len_no]']

%%
figure
hold all
grid on
len_el = length(elems(:,1))
for idx_el = 1:len_el
    plot3(coord_nodes(elems(idx_el,1:2),1)',...
        coord_nodes(elems(idx_el,1:2),2)',...
        coord_nodes(elems(idx_el,1:2),3)')

end




