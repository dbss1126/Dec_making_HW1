%% initiate values
conf_init = [4 14 0];
conf_goal = [21 6 0];
k_att = 0.7;       % attractive field gain
k_rep = 9;       % repulsive field gain
DoI = 2.1;        % Maximum distance of infulence
is_end = false;
map = LoadMap();
map= flip(map ,1); 

%% find obstacle position
[row, col] = find(map);
obs = [col, row];

%% construct Attractive Field
goal_dist = zeros(size(map));
goal_dist(conf_goal(2), conf_goal(1)) = 1;
goal_dist(10, 21) = 1;
goal_dist = bwdist(goal_dist);
potential_att = k_att*(goal_dist);

%colormap prism;

%% construct Repulsive Field
obs_dist = bwdist(map);
potential_rep = (1./obs_dist-1/DoI).^2;
potential_rep(obs_dist > DoI) = 0;
potential_rep(potential_rep == Inf) = 100;
potential_rep = potential_rep*k_rep;

%colormap prism;

%% construct total field
potential_tot = potential_att + potential_rep;

%colormap prism

%% plot
figure;
m1 = mesh(potential_att);
m2 = mesh(potential_rep);
m3 = mesh(potential_tot);
subplot(3,3,1);
m1 = mesh(potential_att);
subplot(3,3,4);
m2 = mesh(potential_rep);
subplot(3,3,7);
m3 = mesh(potential_tot);


%% Follow lowest nearby potential, No backward
%    1  
%  2   4
%    3

conf_cur = [conf_init(1) conf_init(2) 1];



disp_map = cat(3, 255*~map,255*~map,255*~map);
disp_map(conf_goal(2),conf_goal(1),:) = [0,255,0];
disp_map(conf_init(2),conf_init(1),:) = [255,0,0];
disp_map(conf_cur(2),conf_cur(1),:) = [0,0,255];

subplot(3,3,[2,3,5,6,8,9]);

imshow(flip(disp_map),'InitialMagnification','fit')
pause(1)

path = [];

while(~is_end)

    heading = conf_cur(3);
    
    if heading == 1 %%FRONT HEADING
        neigh = [conf_cur(1) conf_cur(2)-1; conf_cur(1)-1 conf_cur(2); conf_cur(1)+1 conf_cur(2)];  % F L R
    elseif heading == 2 %%LEFT HEADING
        neigh = [conf_cur(1)-1 conf_cur(2); conf_cur(1) conf_cur(2)+1; conf_cur(1) conf_cur(2)-1];
    elseif heading == 3 %%BACK HEADING
        neigh = [conf_cur(1) conf_cur(2)+1; conf_cur(1)+1 conf_cur(2); conf_cur(1)-1 conf_cur(2)];
    elseif heading == 4 %%RIGHT HEADING
        neigh = [conf_cur(1)+1 conf_cur(2); conf_cur(1) conf_cur(2)-1; conf_cur(1) conf_cur(2)+1];
    end
    
    [m, i] = min([potential_tot(neigh(1,2),neigh(1,1)) potential_tot(neigh(2,2),neigh(2,1)) potential_tot(neigh(3,2),neigh(3,1))]);
    
    conf_cur = [neigh(i,:) 0];
    
    
    switch heading
        case 1
            switch i
                case 1
                    conf_cur(3) = 1;
                case 2
                    conf_cur(3) = 2;
                case 3
                    conf_cur(3) = 4;
            end
        case 2
            switch i
                case 1
                    conf_cur(3) = 2;
                case 2
                    conf_cur(3) = 3;
                case 3
                    conf_cur(3) = 1;
            end
        case 3
            switch i
                case 1
                    conf_cur(3) = 3;
                case 2
                    conf_cur(3) = 4;
                case 3
                    conf_cur(3) = 2;
            end
        case 4
            switch i
                case 1
                    conf_cur(3) = 4;
                case 2
                    conf_cur(3) = 1;
                case 3
                    conf_cur(3) = 3;
            end
    end

    disp_map = cat(3, 255*~map,255*~map,255*~map);
    disp_map(conf_goal(2),conf_goal(1),:) = [0,255,0];
    disp_map(conf_init(2),conf_init(1),:) = [255,0,0];
    disp_map(conf_cur(2),conf_cur(1),:) = [0,0,255];

    disp(conf_cur)

    switch conf_cur(3)
        case 1
            marker = 'gv';
        case 2
            marker = 'g<';
        case 3
            marker = 'g^';
        case 4
            marker = 'g>';
    end
    
    
    imshow(flip(disp_map),'InitialMagnification','fit')
    hold on
    plot(conf_cur(1),16-conf_cur(2),marker,'MarkerSize',10);
    hold off


    path = [path; conf_cur(1:2)];

    if conf_cur(1:2) == conf_goal(1:2)
        is_end = true;
    end

end


hold on
%plot(path(:,1),16-path(:,2),'ro','MarkerSize',10);


%% straight line
count = 0
for i=3:size(path,1)
    for j=(i+5):size(path,1)
        if path(i,1) == path(j,1)
            disp(i);
            path(i+1,:) = [];
            path(j-2,:) = [];
            path(i+1:j-3,1) = path(i+1:j-3,1) +1;
            count = count+1;
            break
        end
    end
    if count>0
        break
    end
end
count = 0;
for i=7:size(path,1)
    for j=i+1:size(path,1)
        if path(i,2) == path(j,2)
            disp(i);
            path(i,:) = [];
            path(j-1,:) = [];
            path(i:j-2,2) = path(i:j-2,2) +1;
            count = count+1;
            break
        end
    end
    if count>0
        break
    end
end

count = 0;
for i=34:size(path,1)
    for j=i+1:size(path,1)
        if path(i,2) == path(j,2)
            disp(i);
            path(i,:) = [];
            path(j-1,:) = [];
            path(i:j-2,2) = path(i:j-2,2) +1;
            count = count+1;
            break
        end
    end
    if count>0
        break
    end
end

path(20,1) = path(20,1)-1;
path(20,2) = path(20,2)+1;
path(39,1) = path(39,1)-1;
path(39,2) = path(39,2)+1;

hold on
plot(path(:,1),16-path(:,2),'bo','MarkerSize',10);

%% init end path
init_path = [3 15; 4 15; 4 14];
end_path = [22 6; 22 5; 22 4; 22 3; 22 2; 22 1; 23 1;]
path = [init_path; path; end_path];

hold on
plot(path(:,1),16-path(:,2),'bo','MarkerSize',10);
%%


%%