%% Plot the results
figure;

% Scene initialization
axis off;
axis equal;
grid on;
set(gca,'xtick',-10:2:120);
set(gca,'ytick',-20:2:70);

% Outline
[n,~]=size(ELC);
for i=1:n
    line([ELC(i,1),ELC(i,3)],[ELC(i,2),ELC(i,4)] ...
        ,'LineWidth',2,'color','k');
end
hold on;

% Floor Plan & Camera Parameters
for i=1:numel(x)
    if x(i)==1
        cameraParams.pos = CandidateCameraConfig(i,1:3);
        cameraParams.yaw = CandidateCameraConfig(i,4);
        cameraParams.pitch = CandidateCameraConfig(i,5);
        
        % Visibility Polygon consists of lots of Sampling Points
        PV_FOV = FindVisibilityPolygonWithFOV(EL,cameraParams);
        fill(PV_FOV(:,2),PV_FOV(:,3),[0.2 0.2 0.2],'FaceAlpha',0.5);
        hold on;
        
        % Plot the viewpoint afterwards to avoidance overlapping
        pos = cameraParams.pos;
        plot(pos(1),pos(2),'.','Color','k','MarkerSize',20);
        hold on;
        
        % convert to UE4 coord
        t = cameraParams.pos(1);
        cameraParams.pos(1) = cameraParams.pos(2);
        cameraParams.pos(2) = t;
        cameraParams.pos = cameraParams.pos*50;
        cameraParams.yaw = -mod(cameraParams.yaw-90,360);
        cameraParams.pitch = cameraParams.pitch - 90;
        disp(cameraParams);

    end
end