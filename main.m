% Run the robot with a variety of functions.

%% Setup
max_speed = 0.3;  % max speed of the robot
d = 0.24;  % distance between wheels
c = 0.125;  % speed up/slow down
u_step = 0.125;
u = [0:u_step:5/c]';  % values to sweep through

% % circle with radius R
% R = 0.9144;  % 3 feet in meters
% r = [R*cos(u*c),R*sin(u*c)];

% % recommended ellipse
% r = [0.5*cos(u*c), 0.75*sin(u*c)];

% % Challenge 1
% r = [0.3960*cos(2.65*(c*u + 1.4)), 0.99*sin(c*u + 1.4)];

% Challenge 2
a = 0.4;
l = 0.4;
r = fliplr([-2*a*((l-cos(c*u)).*cos(c*u)+(1-l)), 2*a*(l-cos(c*u)).*sin(c*u)]);

%% Calculate wheel velocities
T = diff(r) ./ diff(u);  % velocity vector
T_hat = T ./ sqrt(sum(T.^2, 2));  % velocity unit vector
N = diff(T_hat) ./ diff(u(1:end - 1));
T_hat3 = [T_hat, zeros(size(T_hat(:, 1)))];  % add a third dim to T_hat
N3 = [N, zeros(size(N(:, 1)))];  % add a third dim to N
Omega = cross(T_hat3(1:end - 1, :), N3);  % rotational velocities
V = sqrt(sum(T.^2, 2));  % linear velocities

Vr = V(1:end-1,:) + d / 2 * sum(Omega, 2);
Vl = V(1:end-1,:) - d / 2 * sum(Omega, 2);

%% Calculate each time step:
distances = sqrt(sum(diff(r).^2, 2));
times = distances ./ V;

%% Plot the predicted course
clf;
figure;
hold on
plot(r(:,1),r(:,2))
pos = zeros(length(Pl), 3); %x, y, theta
dt = times;
lin_vel = (Vl+ Vr)/2;
ang_vel = (Vr - Vl)/d;

for i = 2:1:length(times) - 2
    T_hat = [cos(pos(i - 1, 3)), sin(pos(i - 1, 3))];
    pos(i, 1) = (lin_vel(i) * dt(i) * T_hat(1)) + pos(i - 1, 1);
    pos(i, 2) = (lin_vel(i) * dt(i) * T_hat(2)) + pos(i - 1, 2);
    pos(i, 3) = (ang_vel(i) * dt(i)) + pos(i - 1, 3);
end
plot(pos(:, 1), pos(:, 2), 'ro')

figure; hold on
plot(r(:,1),r(:,2))
pos = r(1,:);  % position of robot
head = pi/12;  % heading of robot
plot(pos(1),pos(2),'b*')
for i = 1:length(u) - 2
    dt = times(i);
    That = [cos(head), sin(head)];
    lin_vel = (Vl(i) + Vr(i))/2;
    ang_vel = (Vr(i) - Vl(i))/d;
    drdpos = lin_vel*That;
    drdhead = ang_vel;
    plot(pos(1),pos(2),'r*')
%     quiver(pos(1),pos(2), That(1),That(2))
    new_pos = pos + drdpos*dt;
    new_head = head + drdhead*dt;
    pos = new_pos;
    head = new_head;
end
plot(pos(1),pos(2),'g*')
axis equal

%% Run the robot
disp(max(Vr))
disp(max(Vl))
if max(Vr) > max_speed || max(Vl) > max_speed
    disp("WARNING: Velocities greater than max of "+string(max_speed)+".")
end

runCourse(times(1:end-1,:),Vr,Vl)
