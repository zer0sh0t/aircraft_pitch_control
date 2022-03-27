% ----------------------------------
% system variables:
% alpha - angle of attack
% q - pitch rate
% theta - pitch angle
% delta - elevator deflection angle

% input: delta
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

% transfer function
s = tf('s');
p_pitch = (1.151 * s + 0.1774) / (s ^ 3 + 0.739 * s ^ 2 + 0.921 * s);

% state space model
% x = [alpha q theta]
% u = [delta]
% y = [theta]
A = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0];
C = [0 0 1];
D = [0];
ss_pitch = ss(A, B, C, D);

% using pid controller: the easiest way
controlSystemDesigner(p_pitch); % Kp = 5.1852, Ki = 1.7, Kd = 2.98

% using compensator-rootlocus method
controlSystemDesigner(p_pitch); % lead compensator: 200 * (s + 0.9) / (s + 30)

% using compensator-bode method
controlSystemDesigner(p_pitch); % lead compensator: 10 * (0.55 * s + 1) / (0.022 * s + 1) with K = 10, alpha = 0.04, T = 0.55

% using lqr algorithm
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

% digitization
Ts = 1 / 30;
Kp = 5.1852;
Ki = 1.7;
Kd = 2.98;
c = pid(Kp, Ki, Kd); % this can also be a lead compensator
dc = c2d(c, Ts, 'tustin');
dp = c2d(p_pitch, Ts, 'zoh');
sys = feedback(c * p_pitch, 1);
d_sys = feedback(dc * dp, 1);
step(sys, d_sys);
legend('continuous', 'digitized');
