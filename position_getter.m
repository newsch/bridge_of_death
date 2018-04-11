clear
clf
load 'not_an_s (1).mat'
Pl = dataset(:, 2); 
Pr = dataset(:, 3);
times = dataset(:, 1);
d = 0.24;

dt = diff(times);
Vr = diff(Pr) ./ dt;
Vl = diff(Pl) ./ dt;
V = (Vr + Vl) / 2;
omega = (Vr - Vl) / d;

pos = zeros(length(Pl), 3); %x, y, theta
for i = 2:1:length(times) - 2
    T_hat = [cos(pos(i - 1, 3)), sin(pos(i - 1, 3))];
    pos(i, 1) = (V(i) * dt(i) * T_hat(1)) + pos(i - 1, 1);
    pos(i, 2) = (V(i) * dt(i) * T_hat(2)) + pos(i - 1, 2);
    pos(i, 3) = (omega(i) * dt(i)) + pos(i - 1, 3);
end
pos = pos(48:end, :);
hold on

plot(pos(1, 1), pos(1, 2), 'ro');
plot(pos(:, 1), pos(:, 2))
plot(pos(end, 1), pos(end, 2), 'go')
axis('equal');
