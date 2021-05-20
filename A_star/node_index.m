function n_index = node_index(OPEN,xval,yval)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    i=1;
    flag = 0;
    while(OPEN(i,2) ~= xval || OPEN(i,3) ~= yval )
        i=i+1;
        if(i>size(OPEN,1))
            flag =0;
            break;
        end
        flag =1;
    end
    if(flag)
        n_index=i;
    else
        n_index= -1;
    end
end