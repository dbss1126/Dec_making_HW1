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

dist = 0.5;

%% Place Random Node

V = [V; conf_init];
disp_map = cat(3, 255*~map,255*~map,255*~map);
disp_map(conf_goal(2),conf_goal(1),:) = [0,255,0];
disp_map(conf_init(2),conf_init(1),:) = [255,0,0];
imshow(disp_map,'InitialMagnification','fit')
hold on


while(~is_end)
    q_rand = [rand_btw(-0.2*size(map,2), 1.2*size(map,2)) rand_btw(-0.2*size(map,1), 1.2*size(map,1))]; % New Random Node
    q_near = V(find_nearest(q_rand,V),:)
    q_new = points_on_line(q_near, q_rand, dist)
    if ((1 < q_new(2)) & (q_new(2)< size(map,1))) & ((1 < q_new(1))& (q_new < size(map,2)))
        if map(round(q_new(2)),round(q_new(1))) == 0
            V = [V; [q_new(1), q_new(2)]];
            E = [E; [find(V==q_near,1) size(V,1)]];
        end
    end
    p1 = plot(q_rand(1),q_rand(2),'r.','MarkerSize',15);
    plot(V(:,1),V(:,2),'g.','MarkerSize',15);
    p2 = plot(q_new(1),q_new(2),'b.','MarkerSize',15);

    plot([V(E(end,1),1),V(E(end,2),1)],[V(E(end,1),2),V(E(end,2),2)],'g','MarkerSize',15);

    pause(0.0001)
    delete(p1)
    delete(p2)
    if norm(conf_goal - q_new)< goal_torl
        is_end = true;
    end
end

toc



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
    [solu, solv] = solve(f,g)
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
