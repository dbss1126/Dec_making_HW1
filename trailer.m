clear
L = 1;
d1 = 1;
step_t = 0.001;
t = [0:step_t:400];
u1 = zeros(size(t));
u2 = zeros(size(t));
x = zeros(size(t));
y = zeros(size(t));
theta0 = zeros(size(t));
theta1 = zeros(size(t));
phi = zeros(size(t));

roh1 = 8;
roh2 = 1;


%% 1. steering
is_end = false;
i = 2;
while 1
    u2(i) = 1;
    phi(i) = phi(i-1) - step_t*u2(i);
    if phi(i) < atan2d(-1,8)
        disp(1);
        phi(i) = phi(i-1);
        break
    else
        i = i+1;
    end
end
phi(i:end) = phi(i);



%% 2. 1st dubins path
is_end = false;
i = i+1;
while(~is_end)
    u1(i) = 2;
    theta0(i) = theta0(i-1) + u1(i)*tand(phi(i))*step_t/L;
    x(i) = x(i-1) + u1(i)*cos(theta0(i))*step_t;
    y(i) = y(i-1) + u1(i)*sin(theta0(i))*step_t;
    if x(i) > 7.999
        is_end = true;
    else
        i=i+1;
    end
end
theta0(i:end) = theta0(i);
x(i:end) = x(i);
y(i:end) = y(i);



%% 3. steering
is_end = false;
i = i+1;
while 1
    u2(i) = 1;
    phi(i) = phi(i-1) + step_t*u2(i);
    if phi(i) >= atan2d(1,1)
        disp(1);
        break
    else
        i = i+1;
    end
end
phi(i:end) = phi(i);

%% 4. 2nd dubins path
is_end = false;
while(~is_end)
    u1(i) = 1;
    theta0(i) = theta0(i-1) + u1(i)*tand(phi(i))*step_t/L;
    x(i) = x(i-1) + u1(i)*cos(theta0(i))*step_t;
    y(i) = y(i-1) + u1(i)*sin(theta0(i))*step_t;
    if x(i) > 9.9989
        is_end = true;
    else
        i=i+1;
    end
end

x(i:end) = x(i);
y(i:end) = y(i);
theta0(i:end) = theta0(i);

%% 5. steering
is_end = false;
i = i+1;
while 1
    u2(i) = 1;
    phi(i) = phi(i-1) - step_t*u2(i);
    if phi(i) <= 0
        disp(1);
        break
    else
        i = i+1;
    end
end
phi(i:end) = phi(i);
disp(phi(i))


%% 6. move forward
is_end = false;
while(~is_end)
    u1(i) = 10;
    theta0(i) = theta0(i-1) + u1(i)*tand(phi(i))*step_t/L;
    x(i) = x(i-1) + u1(i)*cos(theta0(i))*step_t;
    y(i) = y(i-1) + u1(i)*sin(theta0(i))*step_t;
    if y(i) > 0
        is_end = true;
    else
        i=i+1;
    end
end

plot(x,y);
