function theta = ComputePolarAngle(coord)
    
    x = coord(1);
    y = coord(2);
    
%     if x==0
%        if y>0
%            theta = pi/2;
%        else
%            theta = pi*3/2;
%        end
%        return
%     end
%     
%     if x>0
%        if y>=0
%            theta = atan(y/x);
%        else
%            theta = atan(y/x)+2*pi;
%        end
%     else
%         theta = atan(y/x)+pi;
%     end
    
    theta = atan2(y/x);

end