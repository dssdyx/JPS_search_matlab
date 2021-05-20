function flag_search = search_nodes(x,y,nodes)
    i=1;
    flag = 0;
    if(size(nodes,1)==0)
        flag_search = -1
    else
        while(nodes(i,1) ~= x || nodes(i,2) ~= y )
            i=i+1;
            if(i>size(nodes,1))
                flag =0;
                break;
            end
            flag =1;
        end
    end
    if(flag)
        flag_search=1;
    else
        flag_search= -1;
    end
end