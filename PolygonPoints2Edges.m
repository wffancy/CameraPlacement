function EL = PolygonPoints2Edges(P)

    [~,np] = size(P);
    if np==2
        SP = P;
        EP = P(2:end,:);
        EP = [EP;P(1,:)];
        EL = [SP EP];
    elseif np==3    % ???
        SP = P(:,2:3);
        EP = SP(2:end,:);
        EP = [EP;SP(1,:)];
        EL = [SP EP];
    else
        EL = [];
    end

end