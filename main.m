% Run the robot with a variety of functions.

%% Setup
max_speed = 0.3;  % max speed of the robot
d = 0.24;  % distance between wheels
c = 0.125;  % speed up/slow down
t_step = 0.1125;
u = [0:t_step:2*pi]';  % values to sweep through

% recommended ellipse
r = [0.5*cos(u), 0.75*sin(u)];

% % Challenge 1
% r = [0.3960*cos(2.65*(u + 1.4)), 0.99*sin(u + 1.4)];
% 
% % Challenge 2
% a = 0.4;
% l = 0.4;
% r = [-2*a*((l-cos(u)*cos(u)+(1-l)), 2*a*(l-cos(u))*sin(u)];

%% Calculate wheel velocities
T = diff(r) ./ diff(u);  % velocity vector
T_hat = T ./ sqrt(sum(T.^2, 2));  % velocity unit vector
N = diff(T_hat) ./ diff(u(1:end - 1));
T_hat3 = [T_hat, zeros(size(T_hat(:, 1)))];  % add a third dim to T_hat
N3 = [N, zeros(size(N(:, 1)))];  % add a third dim to N
Omega = cross(T_hat3(1:end - 1, :), N3);  % rotational velocities
V = sqrt(sum(T.^2, 2));  % linear velocities


Vr = V(1:end-1,:) + d * sum(Omega, 2);
Vl = V(1:end-1,:) - d * sum(Omega, 2);
V_comb = [Vr, Vl];
Vr = Vr * (0.3 / max(max(V_comb)));
Vl = Vl *(0.3 / max(max(V_comb)));
display(max(Vr))
display(max(Vl))

%% Calculate each time step:
distances = sqrt(sum(diff(r).^2, 2));
times = distances ./ V;

%% Run the robot
disp(max(Vr))
disp(max(Vl))
if max(Vr) > max_speed || max(Vl) > max_speed
    disp("WARNING: Velocities greater than max of "+string(max_speed)+".")
end

runCourse(ones(size(Vr))*t_step,Vr,Vl)
