function [c,ceq] = AnsConstrains(x)

% M = [];
% L = 0;
% for i=1:numel(x)
%     if x(i)==1
%         ori = mod(i-1,8)*45;
%         index = floor((i-1)/8) + 1;
%         pos(1) = CandidateCameraPos(index,1);
%         pos(2) = CandidateCameraPos(index,2);
%         cameraParams.pos = pos;
%         cameraParams.orient = ori;
% 
%         Area = ComputeOccupancyGrid(problem,cameraParams);
%         M = cat(2,M,-sum(Area));
%         L = L+sum(Area);
%     end
% end

% limit camera numbers
c = sum(x)-5;
ceq =[];