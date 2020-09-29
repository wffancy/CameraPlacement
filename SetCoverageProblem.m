% Fitness Function
function L = SetCoverageProblem(x,CandidateCameraPos,problem,cameraParams,b)

% N = sum(x);     % quantity of Cameras

% minimize coverage zone between cameras
A = [];
for i=1:length(x)
    if x(i)==1
        cameraParams.pos = CandidateCameraPos(i,1:3);
        cameraParams.yaw = CandidateCameraPos(i,4);
        cameraParams.pitch = CandidateCameraPos(i,5);

        Area = ComputeOccupancyGrid(problem,cameraParams);
        A = cat(2,A,Area & b);
    end
end

if numel(A)==0
    A = zeros(numel(b),1);
end
occu = sum(A,2);
L = norm(occu-b);