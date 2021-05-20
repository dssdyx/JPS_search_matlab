function nodes = get_interest_node(node_x,node_y,map,MAX_X,MAX_Y)
    nodes = [];
    nodes_count =0;
    x_idx = round(node_x);
    y_idx = round(node_y);
    %右上角 
    flag_r_up =0;
    
    while(1)   
    
        for x_up = x_idx:MAX_X
        if(obs_map(x_up,y_idx,map,MAX_X,MAX_Y)== -1 || x_up == MAX_X) %遇到边界或者障碍物直接结束
            break;
        end
        if(obs_map(x_up,y_idx,map,MAX_X,MAX_Y)==0)
               if(search_nodes(x_up,y_idx,nodes) == -1)
                	nodes_count = nodes_count +1;
                    nodes(nodes_count,1)= x_up;
                    nodes(nodes_count,2)= y_idx;
                end   
                if(search_nodes(x_idx,y_idx,nodes) == -1)
                    nodes_count = nodes_count+1;
                    nodes(nodes_count,1)= x_idx;
                    nodes(nodes_count,2)= y_idx;
                end
                break;
        end
        if(obs_map(x_up,y_idx+1,map,MAX_X,MAX_Y) == -1)
            if(obs_map(x_up+1,y_idx+1,map,MAX_X,MAX_Y)~=-1 && obs_map(x_up+1,y_idx,map,MAX_X,MAX_Y)~=-1)
               if(search_nodes(x_up,y_idx,nodes) == -1)
                	nodes_count = nodes_count +1;
                    nodes(nodes_count,1)= x_up;
                    nodes(nodes_count,2)= y_idx;
                end   
                if(search_nodes(x_idx,y_idx,nodes) == -1)
                    nodes_count = nodes_count+1;
                    nodes(nodes_count,1)= x_idx;
                    nodes(nodes_count,2)= y_idx;
                end
                break;
            end
        end
        if(obs_map(x_up,y_idx-1,map,MAX_X,MAX_Y) == -1)
            if(obs_map(x_up+1,y_idx-1,map,MAX_X,MAX_Y)~=-1 && obs_map(x_up+1,y_idx,map,MAX_X,MAX_Y)~=-1)
                %关键点
                if(search_nodes(x_up,y_idx,nodes) == -1)
                	nodes_count = nodes_count +1;
                    nodes(nodes_count,1)= x_up;
                    nodes(nodes_count,2)= y_idx;
                end                
                if(search_nodes(x_idx,y_idx,nodes) == -1)
                    nodes_count = nodes_count+1;
                    nodes(nodes_count,1)= x_idx;
                    nodes(nodes_count,2)= y_idx;
                end
                break;
            end
        end
            
        end
        for y_up = y_idx:MAX_Y
            if(obs_map(x_idx,y_up,map,MAX_X,MAX_Y) || y_up == MAX_Y) %遇到边界或者障碍物直接结束
                break;
            end
            if(obs_map(x_idx,y_up,map,MAX_X,MAX_Y)==0)
               if(search_nodes(x_idx,y_up,nodes) == -1)
                	nodes_count = nodes_count +1;
                    nodes(nodes_count,1)= x_idx;
                    nodes(nodes_count,2)= y_up;
                end   
                if(search_nodes(x_idx,y_idx,nodes) == -1)
                    nodes_count = nodes_count+1;
                    nodes(nodes_count,1)= x_idx;
                    nodes(nodes_count,2)= y_idx;
                end
                break;
        end
            if(obs_map(x_idx+1,y_up,map,MAX_X,MAX_Y) == -1)
                if(obs_map(x_idx+1,y_up+1,map,MAX_X,MAX_Y)~=-1 && obs_map(x_idx,y_up+1,map,MAX_X,MAX_Y)~=-1)
                    %关键点
                    if(search_nodes(x_idx,y_up,nodes) == -1)
                        nodes_count = nodes_count +1;
                        nodes(nodes_count,1)= x_idx;
                        nodes(nodes_count,2)= y_up;
                    end
                    if(search_nodes(x_idx,y_idx,nodes) == -1)
                        nodes_count = nodes_count+1;
                        nodes(nodes_count,1)= x_idx;
                        nodes(nodes_count,2)= y_idx;
                        flag_r_up =1;
                    end
                    break;
                end
            end
            if(obs_map(x_idx-1,y_up,map,MAX_X,MAX_Y) == -1)
                if(obs_map(x_idx-1,y_up+1,map,MAX_X,MAX_Y)~=-1 && obs_map(x_idx,y_up+1,map,MAX_X,MAX_Y)~=-1)
                    %关键点
                    if(search_nodes(x_idx,y_up,nodes) == -1)
                        nodes_count = nodes_count +1;
                        nodes(nodes_count,1)= x_idx;
                        nodes(nodes_count,2)= y_up;
                    end
                    if(search_nodes(x_idx,y_idx,nodes) == -1)
                        nodes_count = nodes_count+1;
                        nodes(nodes_count,1)= x_idx;
                        nodes(nodes_count,2)= y_idx;
                        flag_r_up = 1;
                    end
                    break;
                end
            end
        end
        
       if(flag_r_up == 1)
           break;
       end
       x_idx = x_idx +1;
       y_idx = y_idx +1;
       if(x_idx==MAX_X || y_idx == MAX_Y)
           break;
       end
    end
    %右下角

end