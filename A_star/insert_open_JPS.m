function new_row = insert_open_JPS(xval,yval,parent_xval,parent_yval,hn,gn,fn,force_neighbour_flag,force_neighbour_1_x,force_neighbour_1_y,force_neighbour_2_x,force_neighbour_2_y,parent_dir)
%Function to Populate the OPEN LIST
%OPEN LIST FORMAT
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|force neighbour flag| force neighbour 1 x | force
    %neighbour 1 y| force neighbour 2 x| force neighbour 2 y| parent_dir|
%-------------------------------------------------------------------------
%
%   Copyright 2009-2010 The MathWorks, Inc.
new_row=[1,8];
new_row(1,1)=1;
new_row(1,2)=xval;
new_row(1,3)=yval;
new_row(1,4)=parent_xval;
new_row(1,5)=parent_yval;
new_row(1,6)=hn;
new_row(1,7)=gn;
new_row(1,8)=fn;
new_row(1,9)= force_neighbour_flag;
new_row(1,10) = force_neighbour_1_x;
new_row(1,11) = force_neighbour_1_y;
new_row(1,12) = force_neighbour_2_x;
new_row(1,13) = force_neighbour_2_y;
new_row(1,14) = parent_dir;

end