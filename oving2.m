h = 0.01;
h_imu = 0.01;
h_gps = 0.1;
n = 3;

w_a = wgn(1, 3);
w_w = wgn(1, 3);
w_x = wgn(1, 3);
w_theta = wgn(1, 3);

for t = 0:h:t_end
    [a, w] = true_mes(t);
    
    %True values
    v = v + a*h;
    x = x + v*h;
    theta = theta + w*h;
    
    %Noisy measurements
    a_imu = a + w_a;
    w_imu = w + w_w;
    
    if mod(i, h_gps/h_imu) == 0 %Read every tenth measurement from gps
        x_gps = x + w_x;
        theta_gps = theta + w_theta;
    end
    
    %b1 = -b1/T_1*h;
    %b2 = -b2/T_1*h;
    
end
A = [0 1 0 0;]
     


%Time update
x = A*x + B*u;
P = A*P*A' + Q;

%Measurement update
K = P*H' / (H*P*H'+R);
x = x + K*(z - H*x);
P = (eye(n) - K*H)*P;