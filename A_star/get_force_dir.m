function force_dir = get_force_dir(x0,y0,x1,y1)
if( x1 - x0 > 0 && y1 - y0 > 0) %右上角
    force_dir =1;
end
if(x1 - x0 > 0 && y1 - y0 < 0) %右下角
    force_dir =2;
end
if(x1 - x0 < 0 && y1 - y0 < 0) %左下角
    force_dir =3;
end
if(x1 - x0 < 0 && y1 - y0 > 0) %左上角
    force_dir =4;
end
end 