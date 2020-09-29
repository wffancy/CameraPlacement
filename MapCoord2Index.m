function Index = MapCoord2Index(problem,x)
    
    lbx = problem.sample_lbx;
    lby = problem.sample_lbx;
    step = problem.step;
    
    ix = floor((x(1)-lbx)/step) + 1;
    iy = floor((x(2)-lby)/step) + 1;
    
    Index = [ix,iy];
    
end