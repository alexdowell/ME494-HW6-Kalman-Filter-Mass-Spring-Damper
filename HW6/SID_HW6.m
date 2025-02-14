% #2
load('msd_data_hw6')
% import data %
time = msd.t;
y = msd.y;
yd = deriv(y,.01);
ydd = msd.ydd;
%% OLS %%
x = [ y, yd];
T_hat = (x'*x)\x'*ydd;
Y_hat = x*T_hat;
%% Setting up the model matrices %%
dt = 0.01;
k = 1;
c1 = T_hat(1,:);
c2 = T_hat(2,:);
c3 = .95;
N = 3;
Xk = [y(k); yd(k); ydd(k)];
A = [1 dt dt^2/2; 0 1 dt; c1 c2 c3];
B = [0 0; 0 0; 0 0];
H = [1 0 0;0 0 1];
P = eye(N)*1.0;
Q = [.001 0 0;
 0 .001 0;
 0 0 .001];
R = [0.001 0; 0 10];
P_diags(1,:) = diag(P);
%% Kalman %%
for k = 2:length(time)
 X_pred = A*Xk + B*0;
 P_pred = A*P*A' + Q;
 Z = [y(k); ydd(k)];
 yk = Z - H*X_pred;
 Sk = H*P_pred*H' + R;
 Kk = P_pred*H'*Sk^-1;
 Xk = X_pred + Kk*yk;
 P = (eye(N) - Kk*H)*P_pred;
 P_diags(k,:) = diag(P);
 angle_kal(k) = Xk(1);
 rate_kal(k) = Xk(2);
 accel_kal(k) = Xk(3);
end
figure(1)
plot(time, y, time, angle_kal)
title('Kalman Filter Position vs Measured Position')
xlabel('Time [sec]')
ylabel('Angle [rad]')
legend('Meas','Kalman')
figure(2)
plot(time, yd, time, rate_kal)
title('Kalman Filter Ang Velocity vs Measured Ang Velocity')
xlabel('Time [sec]')
ylabel('Ang. Velocity [rad/s]')
legend('Meas','Kalman')
figure(3) % Comparing derivative of rate data to KF accel output
plot(time, ydd, time, accel_kal)
title('Kalman Filter Ang Acceleration vs Measured Ang Acceleration')
xlabel('Time [sec]')
ylabel('Ang. Acceleration [rad/s^2]')
legend('Data','Kalman')
%#3
load('msd_data_hw6')
% import data %
time = msd.t;
N = length(time);
bias = ones(N,1);
y = msd.y;
yd = deriv(y,.01);
ydd = msd.ydd;
%% OLS %%
x = [ones(N,1), y, yd];
T_hat = (x'*x)\x'*ydd;
Y_hat = x*T_hat;
%% Setting up the model matrices %%
dt = 0.01;
k = 1;
c1 = T_hat(1,:);
c2 = T_hat(2,:);
c3 = T_hat(3,:);
N = 3;
Xk = [y(k) ; yd(k); ydd(k)];
A = [1 dt dt^2/2; 0 1 dt; c2 c3 0];
B = [0 0; 0 0; 0 0];
C = [ 0; 0; c1];
H = [1 0 0;0 0 1];
P = eye(N)*1.0;
Q = [.001 0 0;
 0 .001 0;
 0 0 .001];
R = [0.001 0; 0 10];
P_diags(1,:) = diag(P);
%% Kalman %%
for k = 2:length(time)
 X_pred = A*Xk + B*0 + C;
 P_pred = A*P*A' + Q;
 Z = [y(k); ydd(k)];
 yk = Z - H*X_pred;
 Sk = H*P_pred*H' + R;
 Kk = P_pred*H'*Sk^-1;
 Xk = X_pred + Kk*yk;
 P = (eye(N) - Kk*H)*P_pred;
 P_diags(k,:) = diag(P);
 angle_kal(k) = Xk(1);
 rate_kal(k) = Xk(2);
 accel_kal(k) = Xk(3);
end
figure(1)
plot(time, y, time, angle_kal)
title('Kalman Filter Position vs Measured Position')
xlabel('Time [sec]')
ylabel('Angle [rad]')
legend('Meas','Kalman')
figure(2)
plot(time, yd, time, rate_kal)
title('Kalman Filter Ang Velocity vs Measured Ang Velocity')
xlabel('Time [sec]')
ylabel('Ang. Velocity [rad/s]')
legend('Meas','Kalman')
figure(3) % Comparing derivative of rate data to KF accel output
plot(time, ydd, time, accel_kal)
title('Kalman Filter Ang Acceleration vs Measured Ang Acceleration')
xlabel('Time [sec]')
ylabel('Ang. Acceleration [rad/s^2]')
legend('Data','Kalman')