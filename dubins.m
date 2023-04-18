ban = ["RSL","RSR","RLR"];


dubConnObj = dubinsConnection('MinTurningRadius',1, "DisabledPathTypes",ban);
startPose = [rand()*10 rand()*10 rand()*10*pi()];
goalPose = [rand()*10 rand()*10 rand()*10*pi()];

[pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);
show(pathSegObj{1})