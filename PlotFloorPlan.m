function PlotFloorPlan()

    figure;

    % Scene initialization
    axis off;
    axis equal;
    grid on;
    set(gca,'xtick',-10:2:120);
    set(gca,'ytick',-20:2:70);
    
    % Define the Floor Plan
    ELC = [[0 0,110 0];
        [110 0,110 10];
        [110 10,80 30];
        [80 30,50 30];
        [50 30,50 50];
        [50 50,0 50];
        [0 50,0 0]];
    [n,~]=size(ELC);
    for i=1:n
        line([ELC(i,1),ELC(i,3)],[ELC(i,2),ELC(i,4)] ...
            ,'LineWidth',2,'color','k');
    end
    hold on;

    % Define the Obstacle
    obstacle = [[20 20,20 30];
                [20 30,30 30];
                [30 30,30 20];
                [30 20,20 20];
                [50 10,50 30];
                [50 30,55 30];
                [55 30,55 10];
                [55 10,50 10]];
    [n,~]=size(obstacle);
    for i=1:n
        line([obstacle(i,1),obstacle(i,3)],[obstacle(i,2),obstacle(i,4)] ...
            ,'LineWidth',2,'color','k');
    end
    hold on;

    % Floor Plan & Camera Parameters
    EL = [ELC;obstacle];
    x = [10 15];
    cameraParams.pos = x;
    cameraParams.orient = 0;
    cameraParams.fov = 50;
    cameraParams.radius = 30;

    % Visibility Polygon consists of lots of Sampling Points
    PV_FOV = FindVisibilityPolygonWithFOV(EL,cameraParams);
    fill(PV_FOV(:,2),PV_FOV(:,3),[0.2 0.2 0.2],'FaceAlpha',0.5);
    hold on;

    % Plot the viewpoint afterwards for avoidance of covering
    plot(x(1),x(2),'.','Color','k','MarkerSize',30);
    hold on;

    % Plot all sampling points inside Camera'view
    sample_x = -10:2:120;
    sample_y = -10:2:60;
    [SX,SY] = meshgrid(sample_x,sample_y);
    SX = reshape(SX,[],1);
    SY = reshape(SY,[],1);
    M = [SX SY];
    EL_FOV = PolygonPoints2Edges(PV_FOV);
    vispnt = PointsInsidePolygon(EL_FOV,M);
    for i=1:numel(vispnt)
        if vispnt(i)==1
            plot(M(i,1),M(i,2),'.','Color','k','MarkerSize',1);
        end
    end
    hold on;

end