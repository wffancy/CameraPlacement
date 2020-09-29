function PV = FindVisibilityPolygonWithFOV(EL,cameraParams)

%     % Convert to polar coordinates
%      ELP = Cartesian2Polar(ELC,x);

    % initialization
    PV = [];
    pos = cameraParams.pos(1:2);
    h = cameraParams.pos(3);     % height
    yaw = deg2rad(cameraParams.yaw);
    pitch = deg2rad(cameraParams.pitch);
    fovy = deg2rad(cameraParams.fovy);
    aspect_ratio = cameraParams.aspect_ratio;
%     fovx = atan(tan(fovy/2)*aspect_ratio)*2;
    f =(0.5)/tan(fovy/2);
    if pitch + fovy/2 >= pi/2
        far_radius = 1000;
    else
        far_radius = tan(pitch+fovy/2)*h;
    end
    near_radius = tan(pitch-fovy/2)*h;
    
% %     orientation is not within the polygon
%     dr = 1e-3;
%     pdr = [dr*cos(orient), dr*sin(orient)];
%     pdrIn = PointsInsidePolygon(EL,pdr);
%     if pdrIn==0
%         return;
%     end
    
    % trapezoid projection area
    far_halfAngle = abs(atan((0.5*1*aspect_ratio)/(f*sin(pitch)+0.5*cos(pitch))));
    near_halfAngle = abs(atan((0.5*1*aspect_ratio)/(f*sin(pitch)-0.5*cos(pitch))));
    right_radius = far_radius/cos(far_halfAngle); % used for edge drawing
    [x,y] = pol2cart(yaw+far_halfAngle,right_radius);
    right_top = [x y];
    [x,y] = pol2cart(yaw-far_halfAngle,right_radius);
    right_bottom = [x y];
    left_radius = near_radius/cos(near_halfAngle);
    [x,y] = pol2cart(yaw+near_halfAngle,left_radius);
    left_top = [x y];
    [x,y] = pol2cart(yaw-near_halfAngle,left_radius);
    left_bottom = [x y];
    if near_radius<0
        t = left_top;
        left_top = left_bottom;
        left_bottom = t;
    end
    
    % trapezoid right side edge
    alpha = yaw-far_halfAngle : pi/180 : yaw+far_halfAngle;
    iter = length(alpha);   % loop per degree
    for i=1:iter
        r = far_radius/cos(alpha(i)-yaw);
        P = FindNearestIntersection(EL,pos,alpha(i),r);
        PV = cat(1,PV,P);
    end
    PV = sortrows(PV,1);
    
    % trapezoid upper side edge
    PV_ = [];
    v_rt = right_top;
    v_lt = left_top;
    v_rb = right_bottom;
    v_lb = left_bottom;
    for i = 1:99
        v_t = (i/100)*v_lt + (1-i/100)*v_rt;
        [theta,rho] = cart2pol(v_t(1),v_t(2));
        P = FindNearestIntersection(EL,pos,theta,rho);
        PV_ = cat(1,PV_,P);
    end
    PV = cat(1,PV,PV_);
    
    % trapezoid left side edge
    PV_ = [];
    alpha = yaw-near_halfAngle : pi/180 : yaw+near_halfAngle;
    iter = length(alpha);
    for i=1:iter
        r = near_radius/cos(alpha(i)-yaw);
        P = FindNearestIntersection(EL,pos,alpha(i),r);
        PV_ = cat(1,PV_,P);
    end
    if pitch-fovy/2 < 0
        PV_ = sortrows(PV_,1);
    else
        PV_ = sortrows(PV_,1,'descend');
    end
    PV = cat(1,PV,PV_);
    
    % trapezoid lower side edge
    PV_ = [];
    for i = 1:99
        v_t = (i/100)*v_rb + (1-i/100)*v_lb;
        [theta,rho] = cart2pol(v_t(1),v_t(2));
        P = FindNearestIntersection(EL,pos,theta,rho);
        PV_ = cat(1,PV_,P);
    end
    PV = cat(1,PV,PV_);
    
end

function PV = FindNearestIntersection(EL,pos,ang,r)
    PV = [];
    [n,~] = size(EL);
    % For each point, cast 3 rays, 1 directly at point
    % and 1 a little bit either side
    for k=1:3
        switch k
           case 1
               ang = ang - 0.0001;
           case 2   % ang = ang
           otherwise
               ang = ang + 0.0001;
        end

        % Create ray along angle for required distance
        rdx = r * cos(ang);
        rdy = r * sin(ang);

        min_t1 = 1.0;
        min_px = pos(1) + rdx * min_t1;
        min_py = pos(2) + rdy * min_t1;
%                 bValid = false;

        % Check for ray intersection with all edges
        for l=1:n

            % Create line segment vector
            sdx = EL(l,3) - EL(l,1);
            sdy = EL(l,4) - EL(l,2);

            % Not Colinear
            if abs(sdx * rdy - sdy * rdx) > 1e-5

                if abs(rdx) > 1e-5
                    t2 = (rdx * (EL(l,2) - pos(2)) + (rdy * (pos(1) - EL(l,1)))) / (sdx * rdy - sdy * rdx);
                    t1 = (EL(l,1) + sdx * t2 - pos(1)) / rdx;
                else
                    t2 = (pos(1) - EL(l,1)) / sdx;
                    t1 = (EL(l,2) + sdy * t2 - pos(2)) / rdy; 
                end

                if t1 > 0 && t2 >= 0 && t2 <= 1.0

                    if t1 < min_t1
                        min_t1 = t1;
                        min_px = pos(1) + rdx * t1;
                        min_py = pos(2) + rdy * t1;
                    else

                    end

                end

            end

        end

        PV = cat(1,PV,[ang min_px min_py]);

    end

end