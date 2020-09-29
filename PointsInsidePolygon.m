% @params poly is edge list
function flags = PointsInsidePolygon(poly,p)
    
pn = size(p,1);
polyn = size(poly,1);
flags = zeros(pn,1);
if polyn==0
    return;
end

for i=1:pn

    x = p(i,1); y = p(i,2);
    % points lie on the vertices
    t = find(poly(:,1)==x & poly(:,2)==y, 1);
    if ~isempty(t)
        flags(i) = 0;
        continue;
    end

    for j=1:polyn

        x1 = poly(j,1); y1 = poly(j,2);
        x2 = poly(j,3); y2 = poly(j,4);
        min_y = min(y1,y2);
        max_y = max(y1,y2);
        % points lies on the edges
        if (x1-x2)*(y1-y) - (y1-y2)*(x1-x) == 0
            min_x = min(x1,x2);
            max_x = max(x1,x2);
            if y>=min_y && y<=max_y && x>=min_x && x<=max_x
                flags(i) = 0;
                break;
            end
        end

        % otherwise
        if y>min_y && y<=max_y
            if x1==x2
                if x1 < x
                    flags(i) = flags(i) + 1;
                end
            elseif x > (x2 - x1)*(y - y1) / (y2-y1) + x1
                flags(i) = flags(i) + 1;
            end
        end
    end
end

flags = mod(flags,2);
