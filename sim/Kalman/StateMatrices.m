global A B C R Q

% Constants
dt = 0.1;
xsize = 20;
ysize = 20;

% State Matrices
A = [ 1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1 ];
B = [ (dt.^2)/2 0; 0 (dt.^2)/2; dt 0; 0 dt];
C = [ 1 0 0 0; 0 1 0 0 ];

% Covariance Matrices
R = eye(4)*(0.1.^2);
Q = eye(2)*(0.1.^2);

% We start off with an exact estimate of position and velocity
mu_0 = [ 10 10 0 0 ]';
sigma_0 = [ 0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0 ];
