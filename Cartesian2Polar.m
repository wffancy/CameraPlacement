function ELP = Cartesian2Polar(ELC,x)

    [n,~]=size(ELC);
    p = [repelem(x(1),n);repelem(x(2),n);repelem(x(1),n);repelem(x(2),n)];
    p = p';
    ELP = ELC - p;

    coord = ELP.*ELP;
    Rs = sqrt(coord(:,1)+coord(:,2));
    Re = sqrt(coord(:,3)+coord(:,4));
    Thetas = [];
    Thetae = [];

    for i = 1:n
        coords = ELP(i,1:2);
        coorde = ELP(i,3:4);
        ts = ComputePolarAngle(coords);
        te = ComputePolarAngle(coorde);
        Thetas = cat(1,Thetas,ts);
        Thetae = cat(1,Thetae,te);
    end
    
    ELP = [Rs Thetas Re Thetae];

end