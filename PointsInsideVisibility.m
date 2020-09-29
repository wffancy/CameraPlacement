% @params poly is edge list
% @params sight is clear sight radius
% points here are 3 dimentional
function flags = PointsInsideVisibility(poly,camera,p,sight)

    pn = size(p,1);
    flags = zeros(pn,1);

    for i =1:pn
        % sight collide with obstacle
        if IntersectWithEL(poly,camera.pos,p(i,:))
            continue;
        end
        
        % stuff will be hard to distinguish
        if norm(camera.pos-p(i,:))>sight
            continue;
        end
        
        % the coord of points inside view is within the cube of [-1,1]
        t = CoordConvertion(camera,p(i,:));
        if t(4) ~= 0
            t = t/t(4);
        end
        if t(1)<=1 && t(1)>=-1 && t(2)<=1 && t(2)>=-1 && t(3)<=1 && t(3)>=-1
            flags(i) = 1;
        end
    end

end

function coord = CoordConvertion(cameraParams,p)
    p = [p 1];
    pos = cameraParams.pos;
    yaw = deg2rad(cameraParams.yaw);
    pitch = deg2rad(cameraParams.pitch);
    fovy = deg2rad(cameraParams.fovy);
    aspect_ratio = cameraParams.aspect_ratio;
    % Transformation
    T = eye(4);
    T(1,4) = -pos(1);
    T(2,4) = -pos(2);
    T(3,4) = -pos(3);
    
    g = zeros(1,3);
    g(1) = sin(pitch)*cos(yaw);
    g(2) = sin(pitch)*sin(yaw);
    g(3) = -cos(pitch);
    t = zeros(1,3);
    t(1) = cos(pitch)*cos(yaw);
    t(2) = cos(pitch)*sin(yaw);
    t(3) = sin(pitch);
    g_t = cross(g,t);
    % Rotation
    R = eye(4);
    R(1,1:3) = g_t;
    R(2,1:3) = t;
    R(3,1:3) = -g;
    
    top = tan(fovy/2);
    right = top*aspect_ratio;
    
    % Projection
    M1 = eye(4);
    M1(1,1) = 1/right;
    M1(2,2) = 1/top;
    M1(3,3) = 1/1000;
    M2 = eye(4);
    M2(3,4) = 501;
    M_ortho = M1 * M2;
    M_persp_ortho = zeros(4);
    M_persp_ortho(1,1) = -1;
    M_persp_ortho(2,2) = -1;
    M_persp_ortho(4,3) = 1;
    M_persp_ortho(3,3) = -1002;
    M_persp_ortho(3,4) = -1*1001;
    M_persp = M_ortho * M_persp_ortho;
    
    coord = M_persp * R * T * transpose(p);
end

% judge intersection
function flag = IntersectWithEL(poly,origin,p)
    polyn = size(poly,1);
    A = origin(1:2);
    B = p(1:2);
    flag = 0;
    
    for j=1:polyn
        C = poly(j,1:2);
        D = poly(j,3:4);
        
        if lineIntersectSide(A,B,C,D) && lineIntersectSide(C,D,A,B)
            flag = 1;
            return;
        end
    end
    
end

% judge line intersect with line segment
function flag = lineIntersectSide(A,B,C,D)
    fC = (C(2) - A(2)) * (A(1) - B(1)) - (C(1) - A(1)) * (A(2) - B(2));
    fD = (D(2) - A(2)) * (A(1) - B(1)) - (D(1) - A(1)) * (A(2) - B(2));
    if fC * fD >0
        flag = 0;
    else
        flag = 1;
    end
end