function exp_array=expand_array_JPS(node,gn,xTarget,yTarget,CLOSED,MAX_X,MAX_Y,map)
    %Function to return an expanded array
    %This function takes a node and returns the expanded list
    %of successors,with the calculated fn values.
    %The criteria being none of the successors are on the CLOSED list.
    %
    %Copyright 2009-2010 The MathWorks, Inc.
    % |x|y|force neighbor flag | force neigbor1 x | force neighbor1 y| force neighbour2 x| force neighbour2 y 
            % | parent_dir|
    %EXPANDED ARRAY FORMAT
    %--------------------------------
    %|X val |Y val ||h(n) |g(n)|f(n)|force neighbor flag 
    % | force neigbor1 x | force neighbor1 y| force neighbour2 x| force neighbour2 y 
    % | parent_dir|
    %--------------------------------
    node_x = node(1,1);
    node_y = node(1,2);
    exp_array=[];
    exp_count=1;
    c2=size(CLOSED,1);%Number of elements in CLOSED including the zeros
    expand_nodes = get_inter_node(node,map,MAX_X,MAX_Y);
    for k = 1: size(expand_nodes,1)
         s_x = expand_nodes(k,1);
         s_y = expand_nodes(k,2);
         f_flag = expand_nodes(k,3);
         f_1_x = expand_nodes(k,4);
         f_1_y = expand_nodes(k,5);
         f_2_x = expand_nodes(k,6);
         f_2_y = expand_nodes(k,7);
         parent_dir = expand_nodes(k,8);

         if(s_x == node_x && s_y ==node_y)
             continue;
         end
         if( (s_x >0 && s_x <=MAX_X) && (s_y >0 && s_y <=MAX_Y))%node within array bound
             flag=1;                    
             for c1=1:c2
                if(s_x == CLOSED(c1,1) && s_y == CLOSED(c1,2))
                    flag=0;
                end
             end%End of for loop to check if a successor is on closed list.
             if (flag == 1)
                        exp_array(exp_count,1) = s_x;
                        exp_array(exp_count,2) = s_y;
                        exp_array(exp_count,3) = distance(xTarget,yTarget,s_x,s_y);%distance between node and goal,hn
                        exp_array(exp_count,4) = gn+distance(node_x,node_y,s_x,s_y);%cost of travelling to node，gn
                        exp_array(exp_count,5) = exp_array(exp_count,3)+exp_array(exp_count,4);%fn
                        exp_array(exp_count,6) = f_flag;
                        exp_array(exp_count,7) = f_1_x;
                        exp_array(exp_count,8) = f_1_y;
                        exp_array(exp_count,9) = f_2_x;
                        exp_array(exp_count,10) = f_2_y;
                        exp_array(exp_count,11) = parent_dir;
                        exp_count=exp_count+1;
             end%Populate the exp_array list!!!
          end% End of node within array bound
    end%End of if node is not its own successor loop
end   