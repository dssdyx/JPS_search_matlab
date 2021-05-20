function nodes = get_inter_node(node,map,MAX_X,MAX_Y) %从开集中取出
%node格式如下
nodes = []; % |x|y|force neighbor flag | force neigbor1 x | force neighbor1 y| force neighbour2 x| force neighbour2 y 
            % | parent_dir|
nodes_count = 0;
x_idx = round(node(1,1));
y_idx = round(node(1,2));
flag_dia_stop = 0;
%%
%沿着parent dir方向发展，如果没有，说明是起点，4个方向都发展
parent_dir = node(1,8);
if(parent_dir == 0) %说明是起点
    nodes = [single_ori_expand(node,map,MAX_X,MAX_Y,1);single_ori_expand(node,map,MAX_X,MAX_Y,2);
             single_ori_expand(node,map,MAX_X,MAX_Y,3);single_ori_expand(node,map,MAX_X,MAX_Y,4);];
else
    nodes=single_ori_expand(node,map,MAX_X,MAX_Y,parent_dir);
end
%%
%沿着force neighbour方向发展
while(1)
    %当前node x node y是否存在force neighbour
    if(node(1,3)==0)
        break;
    end
    if(node(1,3)==1)
        %沿着该方向拓展
        force_dir = get_force_dir(node(1,1),node(1,2),node(1,4),node(1,5));
        force_node(1,1) = node(1,4);
        force_node(1,2) = node(1,5);
        force_node(1,3) = 0;
        force_node(1,4) = 0;
        force_node(1,5) = 0;
        force_node(1,6) = 0;
        force_node(1,7) = 0;
        force_node(1,8) = force_dir;
        result_nodes = single_ori_expand(force_node,map,MAX_X,MAX_Y,force_dir);
        %如果对角有扩展，而两边无拓展，就直接用对角的
        if(size(result_nodes,1)==1 && ( abs((result_nodes(1,1) - force_node(1,1))/(result_nodes(1,2) - force_node(1,2)))) == 1 && parent_dir ~= force_dir  )
            result_node = result_nodes(1,:);
            nodes = [nodes;result_node];
            break;
        end
        if(size(result_nodes,1)~=0 && parent_dir ~= force_dir)  
           nodes = [nodes ; force_node;];
        end
        break;
    end
    if(node(1,3)==2)
        %沿着两个方向拓展
        force_dir_1 = get_force_dir(node(1,1),node(1,2),node(1,4),node(1,5));
        force_dir_2 = get_force_dir(node(1,1),node(1,2),node(1,6),node(1,7));
        force_node_1(1,1) = node(1,4);
        force_node_1(1,2) = node(1,5);
        force_node_1(1,3) = 0;
        force_node_1(1,4) = 0;
        force_node_1(1,5) = 0;
        force_node_1(1,6) = 0;
        force_node_1(1,7) = 0;
        force_node_1(1,8) = force_dir_1;
        
        force_node_2(1,1) = node(1,6);
        force_node_2(1,2) = node(1,7);
        force_node_2(1,3) = 0;
        force_node_2(1,4) = 0;
        force_node_2(1,5) = 0;
        force_node_2(1,6) = 0;
        force_node_2(1,7) = 0;
        force_node_2(1,8) = force_dir_2;
        
        result_nodes_1 = single_ori_expand(force_node_1,map,MAX_X,MAX_Y,force_dir_1);
        result_nodes_2 = single_ori_expand(force_node_2,map,MAX_X,MAX_Y,force_dir_2);

        if(parent_dir == force_dir_1 && size(result_nodes_2,1)~=0)
           %如果对角有扩展，而两边无拓展，就直接用对角的
           if(size(result_nodes_2,1)==1 && ( abs(result_nodes_2(1,1) - force_node_2(1,1))/(result_nodes_2(1,2) - force_node_2(1,2))) == 1  )
                result_node_2 = result_nodes_2(1,:);
                nodes = [nodes;result_node_2];
                break;
           end
            nodes = [ nodes; force_node_2];
        end
        if(parent_dir == force_dir_2 && size(result_nodes_1,1)~=0)
           %如果对角有扩展，而两边无拓展，就直接用对角的
           if(size(result_nodes_1,1)==1 && ( abs(result_nodes_1(1,1) - force_node_1(1,1))/(result_nodes_1(1,2) - force_node_1(1,2))) == 1  )
                result_node_1 = result_nodes_1(1,:);
                nodes = [nodes;result_node_1];
                break;
           end
            nodes = [ nodes; force_node_1];    
        end
        if(parent_dir ~= force_dir_1 && parent_dir ~= force_dir_2 && size(result_nodes_1,1)~=0 && size(result_nodes_2,1)~=0)
          if(size(result_nodes_1,1)==1 && ( abs(result_nodes_1(1,1) - force_node_1(1,1))/(result_nodes_1(1,2) - force_node_1(1,2))) == 1  )
                result_node_1 = result_nodes_1(1,:);
                nodes = [nodes;result_node_1];
                if(size(result_nodes_2,1)==1 && ( abs(result_nodes_2(1,1) - force_node_2(1,1))/(result_nodes_2(1,2) - force_node_2(1,2))) == 1  )
                    result_node_2 = result_nodes_2(1,:);
                    nodes = [nodes;result_node_2];
                end
                break;
          end
          if(size(result_nodes_2,1)==1 && ( abs(result_nodes_2(1,1) - force_node_2(1,1))/(result_nodes_2(1,2) - force_node_2(1,2))) == 1  )
                result_node_2 = result_nodes_2(1,:);
                nodes = [nodes;result_node_2];
                if(size(result_nodes_1,1)==1 && ( abs(result_nodes_1(1,1) - force_node_1(1,1))/(result_nodes_1(1,2) - force_node_1(1,2))) == 1  )
                    result_node_1 = result_nodes_1(1,:);
                    nodes = [nodes;result_node_1];
                end
                break;
           end
           nodes = [ nodes;force_node_1;force_node_2];
        end
        break;
    end
end
nodes = delete_same_nodes(nodes);
    
end