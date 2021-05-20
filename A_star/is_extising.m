function [i,nodes] = is_extising(nodes,new_node)
i =0;
nodes = nodes;
if(size(nodes,1) == 0)
    i=0;
else
    for j = 1:size(nodes,1)
        if(nodes(j,1) == new_node(1,1) && nodes(j,2) == new_node(1,2))
            %force 可能不一样 force最多有两个 
            if(new_node(1,3) == 0) %虽然找到一样的点，但是新点没有force，直接结束
                i = 1;
                break;
            end
            if(nodes(j,3) == 2) %在nodes里已经有2个force了，不可能再有第三个
                i = 1;
                break;
            end
            if(nodes(j,3) == 1)%同时有force，对比，如果一样，不管，不一样，加入
                if(nodes(j,4) ~= new_node(1,4) || nodes(j,5) ~= new_node(1,5))
                    nodes(j,3) = 2;
                    i = 1;
                    nodes(j,6) = new_node(1,4);
                    nodes(j,7) = new_node(1,5);
                end
            end
            if(nodes(j,3) == 0)
               nodes(j,3) =1;
               i=1;
               nodes(j,4) = new_node(1,4);
               nodes(j,5) = new_node(1,5);
            end
        end
    end
end
end