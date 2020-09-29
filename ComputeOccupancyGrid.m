function Area = ComputeOccupancyGrid(varargin)
narginchk(1,2);

% resemble overloading
% scene occupied grid
if nargin == 1
    problem = varargin{1};
    
    sample_x = problem.sample_lbx:problem.step:problem.sample_ubx;
    sample_y = problem.sample_lby:problem.step:problem.sample_uby;
    sample_z = problem.sample_lbz:problem.step:problem.sample_ubz;
    [SX,SY,SZ] = meshgrid(sample_x,sample_y,sample_z);
    SX = reshape(SX,[],1);
    SY = reshape(SY,[],1);
    SZ = reshape(SZ,[],1);
    M = [SX SY SZ];
    Area = PointsInsidePolygon(problem.EL,M);
    
% camera view grid
elseif nargin == 2
    problem = varargin{1};
    cameraParams = varargin{2};
    sight = cameraParams.sight;
    
    sample_x = problem.sample_lbx:problem.step:problem.sample_ubx;
    sample_y = problem.sample_lby:problem.step:problem.sample_uby;
    sample_z = problem.sample_lbz:problem.step:problem.sample_ubz;
    [SX,SY,SZ] = meshgrid(sample_x,sample_y,sample_z);
    SX = reshape(SX,[],1);
    SY = reshape(SY,[],1);
    SZ = reshape(SZ,[],1);
    M = [SX SY SZ];
    Area = PointsInsideVisibility(problem.EL,cameraParams,M,sight);
end