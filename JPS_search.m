function path = JPS_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset

    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n)
    %|g(n)|f(n)| force neighbour flag| force neighbour 1 x | force
    %neighbour 1 y| force neighbour 2 x| force neighbour 2 y| parent_dir|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    force_flag = 0;
    force_x = 0;
    force_y = 0;
    parent_dir =1;
    OPEN(OPEN_COUNT,:)=insert_open_JPS(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance,force_flag,force_x,force_y,force_x,force_y,parent_dir);
    %OPEN(OPEN_COUNT,1)=1;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;
    node_idx = 0;

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while(1) %you have to dicide the Conditions for while loop exit 
        
     %
     %finish the while loop
     %
        node_idx = min_fn_JPS(OPEN,OPEN_COUNT,xTarget,yTarget);
        if(node_idx == -1)
            NoPath =1;
            break;
        end
        OPEN(node_idx,1) = 0;
        xNode = OPEN(node_idx,2);
        yNode = OPEN(node_idx,3);
        path_cost = OPEN(node_idx,7);
        % |x|y|force neighbor flag | force neigbor1 x | force neighbor1 y| force neighbour2 x| force neighbour2 y 
        % | parent_dir|
        node(1,1) = OPEN(node_idx,2);
        node(1,2) = OPEN(node_idx,3);
        node(1,3) = OPEN(node_idx,9);
        node(1,4) = OPEN(node_idx,10);
        node(1,5) = OPEN(node_idx,11);
        node(1,6) = OPEN(node_idx,12);
        node(1,7) = OPEN(node_idx,13);
        node(1,8) = OPEN(node_idx,14);
    
 
     
        CLOSED_COUNT = CLOSED_COUNT + 1;
        CLOSED(CLOSED_COUNT,1) = xNode;
        CLOSED(CLOSED_COUNT,2) = yNode;

        if(xNode == xTarget && yNode == yTarget)
            NoPath = 0;
            break;
        end
        exp_array=expand_array_JPS(node,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y,MAP);
        %要先看看是不是在开集里，如果在，更新gn和fn，如果不在，加入开集
        for i = 1:size(exp_array,1)
            neighbor_x = exp_array(i,1);
            neighbor_y = exp_array(i,2);
            neighbor_hn = exp_array(i,3);
            neighbor_gn = exp_array(i,4);
            neighbor_fn = exp_array(i,5);
            f_flag = exp_array(i,6);                    
            f_1_x = exp_array(i,7);
            f_1_y = exp_array(i,8);
            f_2_x = exp_array(i,9);
            f_2_y = exp_array(i,10);
            parent_dir = exp_array(i,11);
            if (node_index(OPEN,neighbor_x,neighbor_y)~= -1)
                open_idx = node_index(OPEN,neighbor_x,neighbor_y);
                if(OPEN(open_idx,7)>neighbor_gn)
                    OPEN(open_idx,4) = xNode;
                    OPEN(open_idx,5) = yNode;
                    OPEN(open_idx,7) = neighbor_gn;
                    OPEN(open_idx,8) = OPEN(open_idx,6) + OPEN(open_idx,7);
                    OPEN(open_idx,14) = parent_dir;
                end
            else
                OPEN_COUNT = OPEN_COUNT + 1;
                OPEN(OPEN_COUNT,:) = insert_open_JPS(neighbor_x ,neighbor_y,xNode,yNode,neighbor_hn,neighbor_gn,neighbor_fn,f_flag,f_1_x,f_1_y,f_2_x,f_2_y,parent_dir);
            end         
        end
     
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
    
   path = [];
   path_count = 0;
   if(~NoPath)
       while(1)
           path_count = path_count + 1;
           path(path_count,1)=xNode;
           path(path_count,2)=yNode;
           if(xNode == xStart && yNode == yStart)
             break;
           end
           last_idx = node_index(OPEN,xNode,yNode);
           xNode = OPEN(last_idx,4);
           yNode = OPEN(last_idx,5);
       end
   end
end