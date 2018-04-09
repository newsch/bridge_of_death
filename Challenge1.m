% This node drives forward until it smashes into an object (e.g., John's
% Leg), and then drives in reverse

cleanupObj = onCleanup(@cleanMeUp);

sub = rossubscriber('/bump');
pub = rospublisher('/raw_vel');
message = rosmessage(pub);


t = [0:0.125:pi]';
r = [0.396*cos(2.65*(t + 1.4)), 0.99*sin(t + 1.4)];
d = 0.24;

T = diff(r) ./ diff(t);
T_hat = T ./ sqrt(sum(T.^2, 2));
N = diff(T_hat) ./ diff(t(1:end - 1));
T_hat3 = [T_hat, zeros(size(T_hat(:, 1)))];
N3 = [N, zeros(size(N(:, 1)))];
Omega = cross(T_hat3(1:end - 1, :), N3);
V = sqrt(sum(T.^2, 2));
Vr = V(1:end-1,:) + d * sum(Omega, 2);
Vl = V(1:end-1,:) - d * sum(Omega, 2);

disp("running course!");
for i = 1:length(t)-2
    message.Data = [Vr(i), Vl(i)];
    send(pub, message);
    pause(0.125);
end
message = rosmessage(pub);
message.Data = [0, 0];
send(pub, message);