function nodes = single_ori_expand(node,map,MAX_X,MAX_Y,dir)
%右上角 1 ，右下角 2， 左下角 3，左上角 4
nodes = []; % |x|y|force neighbor flag | force neigbor1 x | force neighbor1 y| force neighbour2 x| force neighbour2 y 
            % | parent_dir|
nodes_count = 0;
flag_dia_plus = 0; %表明dia递增没有
flag_dia_reach_bound = 0; %表明dia有没有边界
x_idx = round(node(1,1));
y_idx = round(node(1,2));
x0 = round(node(1,1));
y0 = round(node(1,2));
flag_dia_stop = 0;
%%
%右上角
if(dir == 1)
    while(1)
        %x+ expansion 拓展停止条件优先级：存在force；到达目标；遇到障碍物；到达边界。
        for x_exp = x_idx : MAX_X
            %情况 -1 到达目标点
            if(obs_map(x_exp,y_idx,map,MAX_X,MAX_Y) == 0 )
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 0;
                new_node(1,4) = 0;
                new_node(1,5) = 0;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 1;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 1;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else %说明还未对角移动，如果发生对角移动，就不加入，不然会重复拓展，而且也无法直接到达啊
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end
            %情况 0 x+已经到达边界，或者下一步被堵住，无论是上面什么情况，都无法拓展
            if(x_exp == MAX_X || obs_map(x_exp+1,y_idx,map,MAX_X,MAX_Y)==-1)
                break;
            end
            
            %情况1 有force neighbour，表现：x_exp的上下存在障碍物，但对应force
            %neighbor没障碍物，且原本的有效neighbor也没有障碍物(情况0已讨论)
            if(obs_map(x_exp,y_idx+1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp+1,y_idx+1,map,MAX_X,MAX_Y)~= -1 &&(x_exp ~= x_idx))
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 1;
                new_node(1,4) = x_exp+1;
                new_node(1,5) = y_idx+1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 1;
                if(flag_dia_plus ==1 )
                    %父对角拓展节点也需要加入,此时的对角节点
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 1;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0) 
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_exp,y_idx-1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp+1,y_idx-1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_exp+1;
                        new_node(1,7) = y_idx-1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break; %x 拓展结束?
            end
            if(obs_map(x_exp,y_idx-1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp+1,y_idx-1,map,MAX_X,MAX_Y)~= -1&&(x_exp ~= x_idx))
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 1;
                new_node(1,4) = x_exp+1;
                new_node(1,5) = y_idx-1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 1;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 1;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0) %当xy方向都有force，这个点会被重复加
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                 else
                    if(obs_map(x_exp,y_idx+1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp+1,y_idx+1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_exp+1;
                        new_node(1,7) = y_idx+1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node; 
                end
                break;
            end 
        end
        %y+ expansion
        for y_exp = y_idx : MAX_Y
            %情况 -1 到达目标点
            if(obs_map(x_idx,y_exp,map,MAX_X,MAX_Y) == 0)
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_exp;
                new_node(1,3) = 0;
                new_node(1,4) = 0;
                new_node(1,5) = 0;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 1;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 1;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end
            %情况 0 x+已经到达边界，或者下一步被堵住，无论是上面什么情况，都无法拓展
            if(y_exp == MAX_Y || obs_map(x_idx,y_exp+1,map,MAX_X,MAX_Y)==-1)
                break;
            end
            %情况1 有force neighbour，表现：y_exp的左右存在障碍物，但对应force
            %neighbor没障碍物，且原本的有效neighbor也没有障碍物(情况0已讨论)
            %
            if(obs_map(x_idx-1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx-1,y_exp+1,map,MAX_X,MAX_Y)~= -1&&( y_exp ~= y_idx))
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_exp;
                new_node(1,3) = 1;
                new_node(1,4) = x_idx-1;
                new_node(1,5) = y_exp+1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 1;
                if(flag_dia_plus ==1 )
                    %父对角拓展节点也需要加入,此时的对角节点
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 1;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_idx+1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx+1,y_exp+1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_idx + 1;
                        new_node(1,7) = y_exp + 1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;                    
                end
                break; %x 拓展结束
            end
            if(obs_map(x_idx+1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx+1,y_exp+1,map,MAX_X,MAX_Y)~= -1&&( y_exp ~= y_idx))
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_exp;
                new_node(1,3) = 1;
                new_node(1,4) = x_idx+1;
                new_node(1,5) = y_exp+1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 1;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 1;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_idx-1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx-1,y_exp+1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_idx - 1;
                        new_node(1,7) = y_exp + 1;
                    end
                    
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end 
        end
       
        %对角拓展,只有对角是障碍物或者自己存在force,或者对应xy上存在force，或者是到达目标，才停止拓展
        %情况-2 x+ y+方向被堵死 或者 对角方向堵死 或者到达边界
        if((obs_map(x_idx+1,y_idx,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx,y_idx+1,map,MAX_X,MAX_Y) == -1)|| obs_map(x_idx+1,y_idx+1,map,MAX_X,MAX_Y)== -1 || x_idx == MAX_X || y_idx ==MAX_Y)
            flag_dia_stop = 1;
            flag_dia_reach_bound = 1;
        end
        
        %情况 -1 到达目标，但是对于必须x+ 和y+方向上不能同时有障碍物(情况-2)
        if(obs_map(x_idx+1,y_idx+1,map,MAX_X,MAX_Y) == 0 && flag_dia_reach_bound == 0)
            new_node= [];
            new_node(1,1) = x_idx+1;
            new_node(1,2) = y_idx+1;
            new_node(1,3) = 0;
            new_node(1,4) = 0;
            new_node(1,5) = 0;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 1;
            nodes_count = nodes_count + 1;
            nodes(nodes_count,:) = new_node;  
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        %情况 0 自己存在force neighbor 
        %1. 4有障碍物，且2没障碍物，1为force
        %2. 7有障碍物，5没有障碍物，8为force
        if(flag_dia_plus==1 && obs_map(x_idx - 1, y_idx+1 ,map, MAX_X,MAX_Y) ~= -1 && obs_map(x_idx -1, y_idx ,map, MAX_X,MAX_Y) == -1 && obs_map(x_idx, y_idx+1 ,map, MAX_X,MAX_Y) ~= -1 )
            new_node= [];
            new_node(1,1) = x_idx;
            new_node(1,2) = y_idx;
            new_node(1,3) = 1;
            new_node(1,4) = x_idx - 1;
            new_node(1,5) = y_idx + 1;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 1;
            [sign,nodes] = is_extising(nodes,new_node);
            if(sign==0) %可能在xy方向上有结果，对force的处理，在函数内部进行
                nodes_count = nodes_count + 1;
                nodes(nodes_count,:) = new_node;
            end
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        if(flag_dia_plus==1 && obs_map(x_idx + 1, y_idx-1 ,map, MAX_X,MAX_Y) ~= -1 && obs_map(x_idx , y_idx-1 ,map, MAX_X,MAX_Y) == -1 && obs_map(x_idx+1, y_idx ,map, MAX_X,MAX_Y) ~= -1 )
            new_node= [];
            new_node(1,1) = x_idx;
            new_node(1,2) = y_idx;
            new_node(1,3) = 1;
            new_node(1,4) = x_idx + 1;
            new_node(1,5) = y_idx - 1;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 1;
            [sign,nodes] = is_extising(nodes,new_node);
            if(sign==0) %可能在xy方向上有结果，对force的处理，在函数内部进行
                nodes_count = nodes_count + 1;
                nodes(nodes_count,:) = new_node;
            end 
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        if(flag_dia_stop ==1)
            break;
        end
        x_idx = x_idx + 1;
        y_idx = y_idx + 1;
        flag_dia_plus = 1;
    end
end
%%
%右下角
if(dir == 2)
    while(1)
        %x+ expansion 拓展停止条件优先级：存在force；到达目标；遇到障碍物；到达边界。
        for x_exp = x_idx : MAX_X
            %情况 -1 到达目标点
            if(obs_map(x_exp,y_idx,map,MAX_X,MAX_Y) == 0 )
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 0;
                new_node(1,4) = 0;
                new_node(1,5) = 0;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 2;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 2;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else %说明还未对角移动，如果发生对角移动，就不加入，不然会重复拓展，而且也无法直接到达啊
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end
            %情况 0 x+已经到达边界，或者下一步被堵住，无论是上面什么情况，都无法拓展
            if(x_exp == MAX_X || obs_map(x_exp+1,y_idx,map,MAX_X,MAX_Y)==-1 )
                break;
            end
            
            %情况1 有force neighbour，表现：x_exp的上下存在障碍物，但对应force
            %neighbor没障碍物，且原本的有效neighbor也没有障碍物(情况0已讨论)
            if(obs_map(x_exp,y_idx+1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp+1,y_idx+1,map,MAX_X,MAX_Y)~= -1&&(x_exp ~= x_idx))
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 1;
                new_node(1,4) = x_exp+1;
                new_node(1,5) = y_idx+1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 2;
                if(flag_dia_plus ==1 )
                    %父对角拓展节点也需要加入,此时的对角节点
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 2;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0) 
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_exp,y_idx-1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp+1,y_idx-1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_exp+1;
                        new_node(1,7) = y_idx-1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break; %x 拓展结束?
            end
            if(obs_map(x_exp,y_idx-1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp+1,y_idx-1,map,MAX_X,MAX_Y)~= -1&&(x_exp ~= x_idx))
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 1;
                new_node(1,4) = x_exp+1;
                new_node(1,5) = y_idx-1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 2;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 2;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0) %当xy方向都有force，这个点会被重复加
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                 else
                    if(obs_map(x_exp,y_idx+1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp+1,y_idx+1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_exp+1;
                        new_node(1,7) = y_idx+1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node; 
                end
                break;
            end 
        end
        %y+ expansion
        for y_exp = y_idx : -1 : 1
            %情况 -1 到达目标点
            if(obs_map(x_idx,y_exp,map,MAX_X,MAX_Y) == 0)
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_exp;
                new_node(1,3) = 0;
                new_node(1,4) = 0;
                new_node(1,5) = 0;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 2;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 2;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end
            %情况 0 y-已经到达边界，或者下一步被堵住，无论是上面什么情况，都无法拓展
            if(y_exp == 1 || obs_map(x_idx,y_exp-1,map,MAX_X,MAX_Y)==-1 )
                break;
            end
            %情况1 有force neighbour，表现：y_exp的左右存在障碍物，但对应force
            %neighbor没障碍物，且原本的有效neighbor也没有障碍物(情况0已讨论)
            if(obs_map(x_idx-1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx-1,y_exp-1,map,MAX_X,MAX_Y)~= -1&&( y_exp ~= y_idx))
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_exp;
                new_node(1,3) = 1;
                new_node(1,4) = x_idx-1;
                new_node(1,5) = y_exp-1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 2;
                if(flag_dia_plus ==1 )
                    %父对角拓展节点也需要加入,此时的对角节点
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 2;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_idx+1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx+1,y_exp-1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_idx + 1;
                        new_node(1,7) = y_exp - 1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;                    
                end
                break; %x 拓展结束
            end
            if(obs_map(x_idx+1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx+1,y_exp-1,map,MAX_X,MAX_Y)~= -1&&( y_exp ~= y_idx))
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_idx;
                new_node(1,3) = 1;
                new_node(1,4) = x_idx+1;
                new_node(1,5) = y_exp-1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 2;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 2;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_idx-1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx-1,y_exp-1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_idx - 1;
                        new_node(1,7) = y_exp - 1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end 
        end
       
        %对角拓展,只有对角是障碍物或者自己存在force,或者对应xy上存在force，或者是到达目标，才停止拓展
        %情况-2 x+ y-方向被堵死 或者 对角方向堵死 或者到达边界
        if((obs_map(x_idx+1,y_idx,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx,y_idx-1,map,MAX_X,MAX_Y) == -1)|| obs_map(x_idx+1,y_idx-1,map,MAX_X,MAX_Y)== -1 || x_idx == MAX_X || y_idx ==1)
            flag_dia_stop = 1;
            flag_dia_reach_bound = 1;
        end
        
        %情况 -1 到达目标，但是对于必须x+ 和y-方向上不能同时有障碍物(情况-2)
        if(obs_map(x_idx+1,y_idx-1,map,MAX_X,MAX_Y) == 0 && flag_dia_reach_bound == 0)
            new_node= [];
            new_node(1,1) = x_idx+1;
            new_node(1,2) = y_idx-1;
            new_node(1,3) = 0;
            new_node(1,4) = 0;
            new_node(1,5) = 0;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 2;
            nodes_count = nodes_count + 1;
            nodes(nodes_count,:) = new_node;  
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        %情况 0 自己存在force neighbor 
        %1. 4有障碍物，且7没障碍物，6为force
        %2. 2有障碍物，5没有障碍物，3为force
        if(flag_dia_plus==1 && obs_map(x_idx - 1, y_idx - 1 ,map, MAX_X,MAX_Y) ~= -1 && obs_map(x_idx -1, y_idx ,map, MAX_X,MAX_Y) == -1 && obs_map(x_idx, y_idx-1 ,map, MAX_X,MAX_Y) ~= -1 )
            new_node= [];
            new_node(1,1) = x_idx;
            new_node(1,2) = y_idx;
            new_node(1,3) = 1;
            new_node(1,4) = x_idx - 1;
            new_node(1,5) = y_idx - 1;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 2;
            [sign,nodes] = is_extising(nodes,new_node);
            if(sign==0) %可能在xy方向上有结果，对force的处理，在函数内部进行
                nodes_count = nodes_count + 1;
                nodes(nodes_count,:) = new_node;
            end
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        if(flag_dia_plus==1 && obs_map(x_idx + 1, y_idx + 1 ,map, MAX_X,MAX_Y) ~= -1 && obs_map(x_idx , y_idx+1 ,map, MAX_X,MAX_Y) == -1 && obs_map(x_idx+1, y_idx ,map, MAX_X,MAX_Y) ~= -1 )
            new_node= [];
            new_node(1,1) = x_idx;
            new_node(1,2) = y_idx;
            new_node(1,3) = 1;
            new_node(1,4) = x_idx + 1;
            new_node(1,5) = y_idx + 1;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 2;
            [sign,nodes] = is_extising(nodes,new_node);
            if(sign==0) %可能在xy方向上有结果，对force的处理，在函数内部进行
                nodes_count = nodes_count + 1;
                nodes(nodes_count,:) = new_node;
            end 
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        if(flag_dia_stop ==1)
            break;
        end
        x_idx = x_idx + 1;
        y_idx = y_idx - 1;
        flag_dia_plus = 1;
    end
end
%%
%左下角 x- y-
if(dir == 3)
    while(1)
        %x+ expansion 拓展停止条件优先级：存在force；到达目标；遇到障碍物；到达边界。
        for x_exp = x_idx :-1: 1
            %情况 -1 到达目标点
            if((x_exp ~= x_idx) && obs_map(x_exp,y_idx,map,MAX_X,MAX_Y) == 0)
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 0;
                new_node(1,4) = 0;
                new_node(1,5) = 0;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 3;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 3;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else %说明还未对角移动，如果发生对角移动，就不加入，不然会重复拓展，而且也无法直接到达啊
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end
            %情况 0 x+已经到达边界，或者下一步被堵住，无论是上面什么情况，都无法拓展
            if(x_exp == 1 || obs_map(x_exp-1,y_idx,map,MAX_X,MAX_Y)==-1 )
                break;
            end
            
            %情况1 有force neighbour，表现：x_exp的上下存在障碍物，但对应force
            %neighbor没障碍物，且原本的有效neighbor也没有障碍物(情况0已讨论)
            if((x_exp ~= x_idx) && obs_map(x_exp,y_idx+1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp-1,y_idx+1,map,MAX_X,MAX_Y)~= -1)
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 1;
                new_node(1,4) = x_exp-1;
                new_node(1,5) = y_idx+1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 3;
                if(flag_dia_plus ==1 )
                    %父对角拓展节点也需要加入,此时的对角节点
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 3;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0) 
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_exp,y_idx-1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp-1,y_idx-1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_exp-1;
                        new_node(1,7) = y_idx-1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break; %x 拓展结束?
            end
            if((x_exp ~= x_idx) && obs_map(x_exp,y_idx-1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp-1,y_idx-1,map,MAX_X,MAX_Y)~= -1)
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 1;
                new_node(1,4) = x_exp-1;
                new_node(1,5) = y_idx-1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 3;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 3;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0) %当xy方向都有force，这个点会被重复加
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                 else
                    if(obs_map(x_exp,y_idx+1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp-1,y_idx+1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_exp-1;
                        new_node(1,7) = y_idx+1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node; 
                end
                break;
            end 
        end
        %y+ expansion
        for y_exp = y_idx : -1 : 1
            %情况 -1 到达目标点
            if(obs_map(x_idx,y_exp,map,MAX_X,MAX_Y) == 0)
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_exp;
                new_node(1,3) = 0;
                new_node(1,4) = 0;
                new_node(1,5) = 0;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 3;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 3;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end
            %情况 0 y-已经到达边界，或者下一步被堵住，无论是上面什么情况，都无法拓展
            if(y_exp == 1 || obs_map(x_idx,y_exp-1,map,MAX_X,MAX_Y)==-1 )
                break;
            end
            %情况1 有force neighbour，表现：y_exp的左右存在障碍物，但对应force
            %neighbor没障碍物，且原本的有效neighbor也没有障碍物(情况0已讨论)
            if(( y_exp ~= y_idx)&&obs_map(x_idx-1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx-1,y_exp-1,map,MAX_X,MAX_Y)~= -1)
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_exp;
                new_node(1,3) = 1;
                new_node(1,4) = x_idx-1;
                new_node(1,5) = y_exp-1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 3;
                if(flag_dia_plus ==1 )
                    %父对角拓展节点也需要加入,此时的对角节点
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 3;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_idx+1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx+1,y_exp-1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_idx + 1;
                        new_node(1,7) = y_exp - 1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;                    
                end
                break; %x 拓展结束
            end
            if(( y_exp ~= y_idx) && obs_map(x_idx+1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx+1,y_exp-1,map,MAX_X,MAX_Y)~= -1)
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_idx;
                new_node(1,3) = 1;
                new_node(1,4) = x_idx+1;
                new_node(1,5) = y_exp-1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 3;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 3;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_idx-1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx-1,y_exp-1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_idx - 1;
                        new_node(1,7) = y_exp - 1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end 
        end
       
        %对角拓展,只有对角是障碍物或者自己存在force,或者对应xy上存在force，或者是到达目标，才停止拓展
        %情况-2 x- y-方向被堵死 或者 对角方向堵死 或者到达边界
        if((obs_map(x_idx-1,y_idx,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx,y_idx-1,map,MAX_X,MAX_Y) == -1)|| obs_map(x_idx-1,y_idx-1,map,MAX_X,MAX_Y)== -1 || x_idx == 1 || y_idx ==1)
            flag_dia_stop = 1;
            flag_dia_reach_bound = 1;
        end
        
        %情况 -1 到达目标，但是对于必须x- 和y-方向上不能同时有障碍物(情况-2)
        if(obs_map(x_idx-1,y_idx-1,map,MAX_X,MAX_Y) == 0 && flag_dia_reach_bound == 0)
            new_node= [];
            new_node(1,1) = x_idx-1;
            new_node(1,2) = y_idx-1;
            new_node(1,3) = 0;
            new_node(1,4) = 0;
            new_node(1,5) = 0;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 3;
            nodes_count = nodes_count + 1;
            nodes(nodes_count,:) = new_node;  
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        %情况 0 自己存在force neighbor 
        %1. 2有障碍物，且4没障碍物，1为force
        %2. 5有障碍物，7没有障碍物，8为force
        if(flag_dia_plus==1 && obs_map(x_idx - 1, y_idx + 1 ,map, MAX_X,MAX_Y) ~= -1 && obs_map(x_idx , y_idx+1 ,map, MAX_X,MAX_Y) == -1 && obs_map(x_idx-1, y_idx ,map, MAX_X,MAX_Y) ~= -1 )
            new_node= [];
            new_node(1,1) = x_idx;
            new_node(1,2) = y_idx;
            new_node(1,3) = 1;
            new_node(1,4) = x_idx - 1;
            new_node(1,5) = y_idx + 1;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 3;
            [sign,nodes] = is_extising(nodes,new_node);
            if(sign==0) %可能在xy方向上有结果，对force的处理，在函数内部进行
                nodes_count = nodes_count + 1;
                nodes(nodes_count,:) = new_node;
            end
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        if(flag_dia_plus==1 && obs_map(x_idx + 1, y_idx - 1 ,map, MAX_X,MAX_Y) ~= -1 && obs_map(x_idx+1 , y_idx ,map, MAX_X,MAX_Y) == -1 && obs_map(x_idx, y_idx-1 ,map, MAX_X,MAX_Y) ~= -1 )
            new_node= [];
            new_node(1,1) = x_idx;
            new_node(1,2) = y_idx;
            new_node(1,3) = 1;
            new_node(1,4) = x_idx + 1;
            new_node(1,5) = y_idx - 1;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 3;
            [sign,nodes] = is_extising(nodes,new_node);
            if(sign==0) %可能在xy方向上有结果，对force的处理，在函数内部进行
                nodes_count = nodes_count + 1;
                nodes(nodes_count,:) = new_node;
            end 
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        if(flag_dia_stop ==1)
            break;
        end
        x_idx = x_idx - 1;
        y_idx = y_idx - 1;
        flag_dia_plus = 1;
    end
end
%%
%左上角 x- y+
if(dir == 4)
    while(1)
        %x+ expansion 拓展停止条件优先级：存在force；到达目标；遇到障碍物；到达边界。
        for x_exp = x_idx :-1: 1
            %情况 -1 到达目标点
            if((x_exp ~= x_idx) && obs_map(x_exp,y_idx,map,MAX_X,MAX_Y) == 0)
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 0;
                new_node(1,4) = 0;
                new_node(1,5) = 0;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 4;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 4;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else %说明还未对角移动，如果发生对角移动，就不加入，不然会重复拓展，而且也无法直接到达啊
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end
            %情况 0 x+已经到达边界，或者下一步被堵住，无论是上面什么情况，都无法拓展
            if(x_exp == 1 || obs_map(x_exp-1,y_idx,map,MAX_X,MAX_Y)==-1 )
                break;
            end
            
            %情况1 有force neighbour，表现：x_exp的上下存在障碍物，但对应force
            %neighbor没障碍物，且原本的有效neighbor也没有障碍物(情况0已讨论)
            if((x_exp ~= x_idx) && obs_map(x_exp,y_idx+1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp-1,y_idx+1,map,MAX_X,MAX_Y)~= -1)
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 1;
                new_node(1,4) = x_exp-1;
                new_node(1,5) = y_idx+1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 4;
                if(flag_dia_plus ==1 )
                    %父对角拓展节点也需要加入,此时的对角节点
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 4;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0) 
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_exp,y_idx-1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp-1,y_idx-1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_exp-1;
                        new_node(1,7) = y_idx-1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break; %x 拓展结束?
            end
            if((x_exp ~= x_idx) && obs_map(x_exp,y_idx-1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp-1,y_idx-1,map,MAX_X,MAX_Y)~= -1)
                new_node= [];
                new_node(1,1) = x_exp;
                new_node(1,2) = y_idx;
                new_node(1,3) = 1;
                new_node(1,4) = x_exp-1;
                new_node(1,5) = y_idx-1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 4;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 4;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0) %当xy方向都有force，这个点会被重复加
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                 else
                    if(obs_map(x_exp,y_idx+1,map,MAX_X,MAX_Y)== -1 && obs_map(x_exp-1,y_idx+1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_exp-1;
                        new_node(1,7) = y_idx+1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node; 
                end
                break;
            end 
        end
        %y+ expansion
         for y_exp = y_idx : MAX_Y
            %情况 -1 到达目标点
            if(( y_exp ~= y_idx) && obs_map(x_idx,y_exp,map,MAX_X,MAX_Y) == 0)
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_exp;
                new_node(1,3) = 0;
                new_node(1,4) = 0;
                new_node(1,5) = 0;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 4;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 4;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end
            %情况 0 x+已经到达边界，或者下一步被堵住，无论是上面什么情况，都无法拓展
            if(y_exp == MAX_Y || obs_map(x_idx,y_exp+1,map,MAX_X,MAX_Y)==-1 )
                break;
            end
            %情况1 有force neighbour，表现：y_exp的左右存在障碍物，但对应force
            %neighbor没障碍物，且原本的有效neighbor也没有障碍物(情况0已讨论)
            if(( y_exp ~= y_idx) && obs_map(x_idx-1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx-1,y_exp+1,map,MAX_X,MAX_Y)~= -1)
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_exp;
                new_node(1,3) = 1;
                new_node(1,4) = x_idx-1;
                new_node(1,5) = y_exp+1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 4;
                if(flag_dia_plus ==1 )
                    %父对角拓展节点也需要加入,此时的对角节点
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 4;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_idx+1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx+1,y_exp+1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_idx + 1;
                        new_node(1,7) = y_exp + 1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;                    
                end
                break; %x 拓展结束
            end
            if(( y_exp ~= y_idx) && obs_map(x_idx+1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx+1,y_exp+1,map,MAX_X,MAX_Y)~= -1)
                new_node= [];
                new_node(1,1) = x_idx;
                new_node(1,2) = y_exp;
                new_node(1,3) = 1;
                new_node(1,4) = x_idx+1;
                new_node(1,5) = y_exp+1;
                new_node(1,6) = 0;
                new_node(1,7) = 0;
                new_node(1,8) = 4;
                if(flag_dia_plus ==1 )
                    parent_dia_node = [];
                    parent_dia_node(1,1) = x_idx;
                    parent_dia_node(1,2) = y_idx;
                    parent_dia_node(1,3) = 0;%在is extingsing函数里更改
                    parent_dia_node(1,4) = 0;
                    parent_dia_node(1,5) = 0;
                    parent_dia_node(1,6) = 0;
                    parent_dia_node(1,7) = 0;
                    parent_dia_node(1,8) = 4;
                    flag_dia_stop = 1;
                    [sign,nodes] = is_extising(nodes,parent_dia_node);
                    if(sign==0)
                        nodes_count = nodes_count + 1;
                        nodes(nodes_count,:) = parent_dia_node;
                    end
                else
                    if(obs_map(x_idx-1,y_exp,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx-1,y_exp+1,map,MAX_X,MAX_Y)~= -1)
                        new_node(1,3) = 2;
                        new_node(1,6) = x_idx - 1;
                        new_node(1,7) = y_exp + 1;
                    end
                    nodes_count = nodes_count + 1;
                    nodes(nodes_count,:) = new_node;
                end
                break;
            end 
        end
       
        %对角拓展,只有对角是障碍物或者自己存在force,或者对应xy上存在force，或者是到达目标，才停止拓展
        %情况-2 x- y+方向被堵死 或者 对角方向堵死 或者到达边界
        if((obs_map(x_idx-1,y_idx,map,MAX_X,MAX_Y)== -1 && obs_map(x_idx,y_idx+1,map,MAX_X,MAX_Y) == -1)|| obs_map(x_idx-1,y_idx+1,map,MAX_X,MAX_Y)== -1 || x_idx == 1 || y_idx ==MAX_Y)
            flag_dia_stop = 1;
            flag_dia_reach_bound = 1;
        end
        
        %情况 -1 到达目标，但是对于必须x- 和y+方向上不能同时有障碍物(情况-2)
        if(obs_map(x_idx-1,y_idx+1,map,MAX_X,MAX_Y) == 0 && flag_dia_reach_bound == 0)
            new_node= [];
            new_node(1,1) = x_idx-1;
            new_node(1,2) = y_idx+1;
            new_node(1,3) = 0;
            new_node(1,4) = 0;
            new_node(1,5) = 0;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 4;
            nodes_count = nodes_count + 1;
            nodes(nodes_count,:) = new_node;  
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        %情况 0 自己存在force neighbor 
        %1. 5有障碍物，且2没障碍物，3为force
        %2. 7有障碍物，4没有障碍物，6为force
        if(flag_dia_plus==1 && obs_map(x_idx + 1, y_idx + 1 ,map, MAX_X,MAX_Y) ~= -1 && obs_map(x_idx+1 , y_idx ,map, MAX_X,MAX_Y) == -1 && obs_map(x_idx, y_idx+1 ,map, MAX_X,MAX_Y) ~= -1 )
            new_node= [];
            new_node(1,1) = x_idx;
            new_node(1,2) = y_idx;
            new_node(1,3) = 1;
            new_node(1,4) = x_idx + 1;
            new_node(1,5) = y_idx + 1;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 4;
            [sign,nodes] = is_extising(nodes,new_node);
            if(sign==0) %可能在xy方向上有结果，对force的处理，在函数内部进行
                nodes_count = nodes_count + 1;
                nodes(nodes_count,:) = new_node;
            end
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        if(flag_dia_plus==1 && obs_map(x_idx - 1, y_idx - 1 ,map, MAX_X,MAX_Y) ~= -1 && obs_map(x_idx , y_idx-1 ,map, MAX_X,MAX_Y) == -1 && obs_map(x_idx-1, y_idx ,map, MAX_X,MAX_Y) ~= -1 )
            new_node= [];
            new_node(1,1) = x_idx;
            new_node(1,2) = y_idx;
            new_node(1,3) = 1;
            new_node(1,4) = x_idx - 1;
            new_node(1,5) = y_idx - 1;
            new_node(1,6) = 0;
            new_node(1,7) = 0;
            new_node(1,8) = 4;
            [sign,nodes] = is_extising(nodes,new_node);
            if(sign==0) %可能在xy方向上有结果，对force的处理，在函数内部进行
                nodes_count = nodes_count + 1;
                nodes(nodes_count,:) = new_node;
            end 
            %对角拓展是没有父节点的因为可以直接到达
            flag_dia_stop = 1;
        end
        if(flag_dia_stop ==1)
            break;
        end
        x_idx = x_idx - 1;
        y_idx = y_idx + 1;
        flag_dia_plus = 1;
    end
end
end