function vis_high_rise_building(nodes,elems)



%% associate numbers to nodes
len_no = length(nodes(:,1));
coord_nodes = [nodes, [1:1:len_no]'];

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

%% 
pos_x = coord_nodes(:,1);
pos_y = coord_nodes(:,2);
pos_z = coord_nodes(:,3);
dx = 0.1; dy = dx; dz = dx;
labelNodes_ = num2str(coord_nodes(:,4));
labelNodes = cellstr(labelNodes_);
text(pos_x+dx, pos_y+dy, pos_z+dz, labelNodes)

xlabel('x')
ylabel('y')
zlabel('z')

end

