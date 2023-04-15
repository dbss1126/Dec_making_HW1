%% Statespace
%ss = stateSpaceSE2;
ss = stateSpaceDubins;
sv = validatorOccupancyMap(ss);

%% set map
map = occupancyMap(LoadMap(),1);
sv.Map = map;

%% planner settings
sv.ValidationDistance = 0.01;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

planner = plannerRRT(ss,sv, ...
          MaxIterations=250000, ...
          MaxConnectionDistance=0.5);

start = [3 15 0];
goal = [23 1 0];

rng(100,'twister') % repeatable result
[pthObj,solnInfo] = plan(planner,start,goal);

%% visualize
map.show
hold on
% Tree expansion
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
% Draw path
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)