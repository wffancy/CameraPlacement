function PV = FindVisibilityPolygon(EL,x)

%     % Convert to polar coordinates
%      ELP = Cartesian2Polar(ELC,x);
    PV = [];
    [n,~]=size(EL);
    for i=1:n
        for j=1:2
            if j==0
                rdx = EL(i,1)-x(1);
                rdy = EL(i,2)-x(2);
            else
                rdx = EL(i,3)-x(1);
                rdy = EL(i,4)-x(2);
            end

            base_ang = atan2(rdy, rdx);
            % For each point, cast 3 rays, 1 directly at point
            % and 1 a little bit either side
            for k=1:3
                switch k
                   case 1
                       ang = base_ang - 0.0001;
                   case 2
                       ang = base_ang;
                   otherwise
                       ang = base_ang + 0.0001;
                end

                % Create ray along angle for required distance
                rdx = cos(ang);
                rdy = sin(ang);

                min_t1 = inf;
                min_px = 0;
                min_py = 0;
                min_ang = 0;
                bValid = false;
                
                % Check for ray intersection with all edges
                for l=1:n
                    
                    % Create line segment vector
                    sdx = EL(l,3) - EL(l,1);
                    sdy = EL(l,4) - EL(l,2);
                    
                    % Not Colinear
                    if norm([sdx sdy]-[rdx rdy])~=0

                        t2 = (rdx * (EL(l,2) - x(2)) + (rdy * (x(1) - EL(l,1)))) / (sdx * rdy - sdy * rdx);
                        t1 = (EL(l,1) + sdx * t2 - x(1)) / rdx;
                        if t1 > 0 && t2 >= 0 && t2 <= 1.0

                            if t1 < min_t1
                                min_t1 = t1;
                                min_px = x(1) + rdx * t1;
                                min_py = x(2) + rdy * t1;
                                min_ang = atan2(min_py - x(2), min_px - x(1));
                                bValid = true;
                            end

                        end

                    end

                end
                
                if bValid
                    PV = cat(1,PV,[min_ang min_px min_py]);
                end
                
            end

        end
         
    end
    
    PV = sortrows(PV,1);

end