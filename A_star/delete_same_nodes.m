function nodes = delete_same_nodes(nodes)
same_idx = [];
same_count =0;
i=1;
while(1)
    if(size(nodes,1) == 0)
        break;
    end
    node = nodes(i,:);
    for j = i:size(nodes,1)
        if(node(1,1) == nodes(j,1) && node(1,2)==nodes(j,2) && i~=j)
            same_count = same_count + 1;
            same_idx(same_count,1) = j;
        end
    end
    if(i==size(nodes,1))
        break;
    end
    i= i+1;
end
same_idx = unique(same_idx);
same_count = size(same_idx,1);
if(same_count~=0)
    for k = 1:same_count
        
        nodes(same_idx(k,1),:)=[];
    end
end
end