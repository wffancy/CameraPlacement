clc,clear,close all;

%% Scene Setup
% Edge List
ELC = [[0 0,110 0];
    [110 0,110 10];
    [110 10,80 30];
    [80 30,50 30];
    [50 30,50 50];
    [50 50,0 50];
    [0 50,0 0]];

% Define the Obstacle
obstacle = [[20 20,20 30];
            [20 30,30 30];
            [30 30,30 20];
            [30 20,20 20];
            [50 10,50 30];
            [50 30,55 30];
            [55 30,55 10];
            [55 10,50 10]];

%% Floor Plan & Camera Parameters
% EL = [ELC;obstacle];
EL = ELC;
x = [20 15 20];  % 3D Coord [x y z]
cameraParams.pos = x;
cameraParams.yaw = 45;   % direct right as 0 degree (in plane)
cameraParams.pitch = 80; % direct down as 0 degree
cameraParams.fovy = 50;
cameraParams.aspect_ratio = 1.0;
cameraParams.sight = 50; % sight distance

problem.EL = EL;
problem.step = 5;
problem.sample_lbx = 0;
problem.sample_ubx = 110;
problem.sample_lby = 0;
problem.sample_uby = 50;
problem.sample_lbz = 0;
problem.sample_ubz = cameraParams.pos(3);
problem.sample_yawN = 8;
problem.sample_pitchN = 5;
lengthScene = (problem.sample_ubx - problem.sample_lbx)/problem.step + 1;
widthScene = (problem.sample_uby - problem.sample_lby)/problem.step + 1;
heightScene = (problem.sample_ubz - problem.sample_lbz)/problem.step + 1;
problem.size = [lengthScene widthScene heightScene];
% Area = ComputeOccupancyGrid(problem,cameraParams);

%% Problem initialization
% TODO:change the solution domain to 3D,namely using the voxel
% TODO:consider the image quality,namely the distance between target and
% camera.
yawN = problem.sample_yawN;
pitchN = problem.sample_pitchN;
sample_x = problem.sample_lbx:problem.step:problem.sample_ubx;
sample_y = problem.sample_lby:problem.step:problem.sample_uby;
sample_z = cameraParams.pos(3);
step_yaw = 360/yawN;
sample_yaw = 0:step_yaw:360;
sample_yaw(end) = [];   % delete 360 element
step_pitch = 90/pitchN;
sample_pitch = 0:step_pitch:90;
sample_pitch(end) = []; % delete 90 degree
[SX,SY,SZ] = meshgrid(sample_x,sample_y,sample_z);
SX = reshape(SX,[],1);
SY = reshape(SY,[],1);
SZ = reshape(SZ,[],1);
m = [SX SY SZ];
M = [];
for i = 1:length(sample_yaw)
	tmp = sample_yaw(i) * ones(length(m),1);
    tmp = cat(2,m,tmp);
    M = cat(1,M,tmp);
end
m = M;
M = [];
for i = 1:length(sample_pitch)
	tmp = sample_pitch(i) * ones(length(m),1);
    tmp = cat(2,m,tmp);
    M = cat(1,M,tmp);
end

% % sample the whole polygon
% vispnt = PointsInsidePolygon(EL,M);
% CandidateCameraPos = [];
% for i=1:numel(vispnt)
%     if vispnt(i)==1
%         CandidateCameraPos = cat(1,CandidateCameraPos,M(i,:));
%     end
% end

CandidateCameraConfig = SamplePointsNearEdges(EL,5,M);

nVar = length(CandidateCameraConfig);
b = ComputeOccupancyGrid(problem);

% only choose one config in each position --> Aeq * x' <= beq
cameraPosNum = nVar/(yawN*pitchN);
degreeOfFreedom = yawN*pitchN;
Aueq = zeros(cameraPosNum,nVar);
for i=1:cameraPosNum
    for j=1:degreeOfFreedom
        Aueq(i,(i-1)*degreeOfFreedom+j)=1;
    end
end
bueq = ones(cameraPosNum,1);

%% Run GA
tic;
lb = zeros(1,nVar);
ub = ones(1,nVar);
intintices = (1:nVar);
initialx = zeros(1,nVar);
options = optimoptions(@ga, ...%                             'InitialPopulationMatrix',initialx, ...
                            'Display','iter', ...
                            'CrossoverFraction',0.75, ...
                            'MaxStallGenerations',50, ...
                            'PopulationSize',1000);
f = @(x)SetCoverageProblem(x,CandidateCameraConfig,problem,cameraParams,b);
% f(initialx);
[x,fval] = ga(f, nVar, Aueq, bueq, [], [], lb, ub, @AnsConstrains, intintices, options);
toc;
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

% % Obstacle
% [n,~]=size(obstacle);
% for i=1:n
%     line([obstacle(i,1),obstacle(i,3)],[obstacle(i,2),obstacle(i,4)] ...
%         ,'LineWidth',2,'color','k');
% end
% fill(obstacle(1:4,1),obstacle(1:4,2),[0.2 0.2 0.2]);
% fill(obstacle(5:8,1),obstacle(5:8,2),[0.2 0.2 0.2]);
% hold on;

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
