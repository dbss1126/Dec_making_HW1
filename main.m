%% initiate values
conf_init = [3 15 0];
conf_goal = [23 1 0];
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
goal_dist(10, 23) = 1;
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

    pause(0.1)

    if conf_cur(1:2) == conf_goal(1:2)
        is_end = true;
    end

end

