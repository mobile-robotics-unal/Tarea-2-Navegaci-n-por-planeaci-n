clear,clc,close 'all'

load('tarea2.mat')
%% Create Occupancy Map Object
cellSize = 1/3
costmap = vehicleCostmap(double(sm4b),cellSize=cellSize)


% vehicle dimensions 731x614x720 mm
vehicleDims = vehicleDimensions(0.731 , 0.614, 0.720, "FrontOverhang",0.131,"RearOverhang", 0.131, "Wheelbase",0.458);    %[m]

ccConfig = inflationCollisionChecker(vehicleDims);
costmap.CollisionChecker = ccConfig;
%%


figure
plot(ccConfig)
title('Vehicle characteristics')

figure
plot(costmap)
title('Collision Checking with One Circle')




%% PRM

start = [1.2, 0.5];  
goal = [17,16]; 
map = binaryOccupancyMap(sm4b,Resolution=3); 
radius =sqrt((0.731/2)^2 + (0.614/2)^2)     %radios:= diagonal of the vehicle
inflatedMap = map
inflate(inflatedMap,0.6)

figure 
show(map)

show(inflatedMap)

PRM = mobileRobotPRM(map)
PRM.NumNodes = 200;
PRM.ConnectionDistance = 7;

PRM.findpath(start,goal)

figure 
PRM.show()
%% RRT

startPose = [1.2, 0.5, 90];   % [meters, meters, degrees]
goalPose = [17,16, 0]; 

planner = pathPlannerRRT(costmap)
planner.ConnectionDistance = 4;
planner.MinTurningRadius = 0.5;
planner.MinIterations = 100;
planner.MaxIterations= 80000;
[refPath,tree] = planner.plan(startPose,goalPose);

pathFound = ~isempty(refPath.PathSegments)

if (pathFound) 
   
        
    %,ConnectionDistance=0.5,MinTurningRadius=0.4
    
    plot(planner,'Tree','on')
    
end

%pos = refPath.PathSegments.MotionLengths
