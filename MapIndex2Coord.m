function Index = MapIndex2Coord(problem,x)
    
    lbx = problem.sample_lbx;
    lby = problem.sample_lbx;
    step = problem.step;
    
    cx = (x(1)-1)*step + lbx;
    cy = (x(2)-1)*step + lby;
    
    Index = [cx,cy];
    
end