function S = SamplePointsNearEdges(EL,r,PointsZone)

S = [];
n = size(PointsZone,1);
m = size(EL,1);
for i=1:n
    p = PointsZone(i,:);
    for j=1:m
        x1 = EL(j,1:2);
        x2 = EL(j,3:4);
        min_x = min([x1(1),x2(1)]);
        max_x = max([x1(1),x2(1)]);
        min_y = min([x1(2),x2(2)]);
        max_y = max([x1(2),x2(2)]);
        % far from the line segment
        if p(1)>max_x+r || p(1)<min_x-r || p(2)>max_y+r || p(2)<min_y-r
            continue;
        end
        % the distance between the point and the edge(line)
        pos = p(1:2);
        d = abs(det([x2-x1;pos-x1]))/norm(x2-x1);
        if d<=r && PointsInsidePolygon(EL,pos)==1
            S = cat(1,S,p);
        end
    end
end

S = unique(S,'rows');