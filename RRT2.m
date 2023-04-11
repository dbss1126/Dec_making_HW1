%% initiate values
tic
clear
map = LoadMap();
map= flip(map ,1); 
conf_init = [3 15];
conf_goal = [23 1];
V = [];
E = [];
is_end = false;
goal_torl = 0.5;
max_iter = 1000;

dist = 1;
step = 0.2;
rot_angle = pi()-2*atan2(1,step/2);

%% Place Random Node

V = [V; conf_init];
disp_map = cat(3, 255*~map,255*~map,255*~map);
disp_map(conf_goal(2),conf_goal(1),:) = [0,255,0];
disp_map(conf_init(2),conf_init(1),:) = [255,0,0];
imshow(disp_map,'InitialMagnification','fit')
hold on

V = [V; [conf_init(1)+step, conf_init(2)]];
E = [E; [1, 2]];  

while(~is_end)
    q_rand = [rand_btw(-0.2*size(map,2), 1.2*size(map,2)) rand_btw(-0.2*size(map,1), 1.2*size(map,1))]; % New Random Node
    q_near = V(find_nearest(q_rand,V),:);
    [q_near_i, col] = find(V==q_near,1);

    [connected_i, col2] = find(E==q_near_i,1,'last');
    connected_i = E(connected_i,1);
    connected = V(connected_i,:);

    vec_cur = q_near-connected;
    
    
    if rand()>0.5
        dir = 1;
    else
        dir = -1;
    end
    
    rot = [cos(dir*rot_angle) -1*sin(dir*rot_angle); sin(dir*rot_angle) cos(dir*rot_angle)];

    vec_next = rot*vec_cur';
    vec_next = vec_next';
    q_new = q_near + vec_next;

    is_in_V = V==q_new;
    if sum(is_in_V,'all') == 0

        %q_new = points_on_line(q_near, q_rand, dist);
        if ((1 < q_new(2)) & (q_new(2)< size(map,1))) & ((1 < q_new(1))& (q_new < size(map,2)))
            if map(round(q_new(2)),round(q_new(1))) == 0
                V = [V; [q_new(1), q_new(2)]];
                E = [E; [find(V==q_near,1) size(V,1)]];
            end
            p1 = plot(q_near(1),q_near(2),'r.','MarkerSize',25);
            plot(V(:,1),V(:,2),'g.','MarkerSize',15);
            p2 = plot(q_new(1),q_new(2),'b.','MarkerSize',25);
    
            plot([V(E(end,1),1),V(E(end,2),1)],[V(E(end,1),2),V(E(end,2),2)],'g','MarkerSize',15);
    
            pause(0.001)
            delete(p1)
            delete(p2)
            if norm(conf_goal - q_new)< goal_torl
                is_end = true;
            end
        end
    end
end


toc

%% test


%%
while_end = false;
old_E = E;
while (~while_end)
    new_E = remove_endpoint(old_E,V);
    disp(size(old_E));
    disp(size(new_E));
    if size(old_E,1) == size(new_E,1)
        while_end = true;
    else
        old_E = new_E;
    end
    
end


%%
figure;
imshow(disp_map,'InitialMagnification','fit')
hold on
plot(V(:,1),V(:,2),'g.','MarkerSize',15);
for i = 1:size(new_E,1)
    j = new_E(i,:)
    plot([V(j(1),1),V(j(2),1)],[V(j(1),2),V(j(2),2)],'r','MarkerSize',15);
end




%%


%function choose_next()


function nearest_index = find_nearest(q, V)
    distance = zeros(size(V,1),1);
    for i=1:size(V,1)
        distance(i)=norm(q-V(i,:));
    end
    [m, nearest_index] = min(distance);
end

function q_new = points_on_line(q_start, q_end, dist)
    syms x y
    f = (x-q_start(1))^2 + (y-q_start(2))^2 - dist^2;
    g = (y-q_start(2)) -(x-q_start(1))*(q_start(2)-q_end(2))/(q_start(1)-q_end(1));
    [solu, solv] = solve(f,g);
    solu = eval(solu);
    solv = eval(solv);
    if ((q_start(1) < solu(1)) && (q_end(1) > solu(1))) || ((q_end(1) < solu(1)) && (q_start(1) > solu(1)))
        q_new = [solu(1), solv(1)];
    else
        q_new = [solu(2), solv(2)];
    end
end

function rand_num = rand_btw(a,b)
    rand_num = a + (b-a)*rand();
end

function new_E= remove_endpoint(E,V)
    new_E = E;
    V_index = 1:size(V,1);
    end_E_index = [];
    for i=V_index(2:size(V,1)-1)
        [row, col] = find(E==i);
        size(row,1)
        if size(row,1) == 1
            end_E_index = [end_E_index; row];
        end
    end
    
    new_E(end_E_index,:) = [];
end



%{

function path = dubins(conf_i, conf_f);
    mid_point = (conf_i(1:2)+conf_f(1:2))/2;
    vector_m = [-1*mid_point(2) mid_point(1)] %% vector passes mid_point and Center
    vector_i = [-1*conf_i(2) conf_i(1)] %% vector passes inital_point and Center
    syms t1 t2;
    eq1 = conf_i(1)+t1*
    
end
%}

