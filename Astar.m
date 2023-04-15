%% Create map that will be updated with sensor readings

map = LoadMap();
map2 = zeros(size(map)*2);

for y = 1:size(map,1)
    for x=1:size(map,2)
        map2(y*2:y*2+1, x*2:x*2+1) = map(y,x);
    end
end

map2(1,:) = [];
map2(:,1) = [];
imshow(map2)


%%

map2 = occupancyMap(map2,1);
map2.show

estMap = occupancyMap(occupancyMatrix(map2));

%% Create a map that will inflate the estimate for planning
inflateMap = occupancyMap(estMap);

vMap = validatorOccupancyMap;
vMap.Map = inflateMap;
vMap.ValidationDistance = .1;
planner = plannerHybridAStar(vMap, 'MinTurningRadius',1.3);

entrance = [6 30 0];
packagePickupLocation = [45 2 0];
route = plan(planner, entrance, packagePickupLocation);
route = route.States;

%% Get poses from the route.
rsConn = dubinsConnection('MinTurningRadius', planner.MinTurningRadius);
startPoses = route(1:end-1,:);
endPoses = route(2:end,:);

rsPathSegs = connect(rsConn, startPoses, endPoses);
poses = [];
for i = 1:numel(rsPathSegs)
    lengths = 0:0.1:rsPathSegs{i}.Length;
    [pose, ~] = interpolate(rsPathSegs{i}, lengths);
    poses = [poses; pose];
end

figure
show(planner)
title('Initial Route to Package')