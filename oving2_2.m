h = 0.01;               %Time step
h_imu = 0.01;           %IMU Sampling time
h_gps = 0.1;            %GPS Sampling time
h_ratio = h_gps/h_imu;  %GPS reads every tenth sample

t_end = 100;            %run time
n = t_end/h;            %Samples
n_imu = t_end/h_imu;    %IMU samples
n_gps = t_end/h_gps;    %GPS samples


T_1 = 0.5; T_2 = 0.2;   %Bias variables

%Memory allocation
a = zeros(n, 1);
v = zeros(n, 1);
x = zeros(n, 1);
w = zeros(n, 1);
theta = zeros(n, 1);
b_1 = zeros(n, 1);
b_2 = zeros(n, 1);
a_imu = zeros(n_imu, 1);
w_imu = zeros(n_imu, 1);
x_gps = zeros(n_gps, 1);
theta_gps = zeros(n_gps, 1);

i = 0;
for t = 0:h:t_end
    i = i + 1;     %Indexing variable
    
    %%Gaussian white noise
    w_i = wgn(1, 6, 0.1);

    %%True values
    u = true_mes(t);
    a(i) = u(1);
    w(i) = u(2);
    
    v(i+1) = v(i) + a(i)*h;
    x(i+1) = x(i) + v(i)*h;
    theta(i+1) = theta(i) + w(i)*h;

%{
    %%Noisy measurements
    a_imu(i) = a(i) + w_i(1) + b_1(i);
    w_imu(i) = w(i) + w_i(2) + b_2(i);
 
    %Read every tenth measurement from gps
    if mod(i, h_ratio) == 0
        x_gps(i/h_ratio) = x(i) + w_i(3);
        theta_gps(i/h_ratio) = theta(i) + w_i(4);
    end

    %%Update IMU bias
    b_1(i+1) = -h*b_1(i)/T_1 + w_i(5);
    b_2(i+1) = -h*b_2(i)/T_2 + w_i(6);
%}
    
end
v(end) = []; x(end) = []; theta(end) = [];
t = 0:h:t_end;

%%Translational
subplot(3, 1, 1); plot(t, a); title('a'); xlabel('t');
subplot(3, 1, 2); plot(t, v); title('v'); xlabel('t');
subplot(3, 1, 3); plot(t, x); title('x'); xlabel('t');

%%Rotational
figure;
subplot(2, 1, 1); plot(t, w); title('omega'); xlabel('t');
subplot(2, 1, 2); plot(t, theta); title('theta'); xlabel('t')


%% KALMAN %%
%{
A = [0  1   0   0   0;
     0  0   0   0   0;
     0  0   0 -1/T_1 0;
     0  0   0   0   0;
     0  0   0   0 -1/T_2];

B = [1 0;
     0 0;
     0 1;
     0 0;
     0 0];
     
%x = [v; x; theta; b1; b2];

%Time update
x = A*x + B*u;
P = A*P*A' + Q;

%Measurement update
K = P*H' / (H*P*H'+R);
x = x + K*(z - H*x);
P = (eye(n) - K*H)*P;
%}