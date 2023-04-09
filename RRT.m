ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
map = LoadMap();

map = occupancyMap(map,1);
sv.Map = map;

sv.ValidationDistance = 0.1;

ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

planner = plannerRRTStar(ss,sv, ...
          ContinueAfterGoalReached=true, ...
          MaxIterations=5000, ...
          MaxConnectionDistance=0.3);

start = [3 15 0];
goal = [23 1 0];

rng(100,'twister') % repeatable result
[pthObj,solnInfo] = plan(planner,start,goal);

map.show
hold on
% Tree expansion
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
% Draw path
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)