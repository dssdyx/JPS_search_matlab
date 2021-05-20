function state = obs_map(x,y,map,MAX_X,MAX_Y)
    if(x>MAX_X || y > MAX_Y || x<=0 || y<=0 )
        state = -1;
    else
        state = map(x,y);
    end
end