ban = ["LRL","LSR","LSL"];


dubConnObj = dubinsConnection('MinTurningRadius',1, "DisabledPathTypes",ban);
startPose = [0 0 0];
goalPose = [10 10 0];

[pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);
show(pathSegObj{1})