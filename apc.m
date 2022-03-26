% ----------------------------------
% system variables:
% alpha - angle of attack
% q - pitch rate
% theta - pitch angle
% delta - elevator deflection angle

% input : delta
% output: theta
% ----------------------------------

% ----------------------------------
% system eqns:
% alpha_dot = - 0.313 * alpha + 56.7 * q + 0.232 * delta
% q_dot = - 0.0135 * alpha - 0.426 * q + 0.0203 * delta
% theta_dot = 56.7 * q
% ----------------------------------

% ----------------------------------
% design requirements:
% overshoot < 10%
% rise time < 2 secs
% settling time < 10 secs
% ss error < 2%
% ----------------------------------

% using pid controller - easiest way
s = tf('s');
p_pitch = (1.151 * s + 0.1774) / (s ^ 3 + 0.739 * s ^ 2 + 0.921 * s);
controlSystemDesigner(p_pitch); % Kp = 5.1852, Ki = 1.7, Kd = 2.98

% using lqr algorithm
% x = [alpha q theta]
% u = [delta]
A = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0];
C = [0 0 1];
D = [0];
ss_pitch = ss(A, B, C, D);
co = rank(ctrb(A, B))

p = 50;
Q = p * C' * C;
R = 1;
[K] = lqr(A, B, Q, R);
ss_cl = ss(A - B * K, B, C, D);
Nbar = 1 / dcgain(ss_cl);
ss_cl = ss(A - B * K, Nbar * B, C, D);
step(ss_cl);
ylabel('pitch angle(rad)');
