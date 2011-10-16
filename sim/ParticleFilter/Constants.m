global M X_SIZE Y_SIZE V_MEAN V_DEV UP_DEV UA_DEV DELTA_T D_DEV A_DEV L_DEV

% The number of particles
M = 1000;

% We constrain the dimensions of the room to 20 by 20.
X_SIZE = 20;
Y_SIZE = 20;

% Update Noise
UP_DEV = 0.03;
UA_DEV = 0.02;

% Assume that the initial speed is normally distributed around zero.
V_MEAN = 0;
V_DEV = 0.1;

% Time change
DELTA_T = 0.1;

% Measurement Uncertainties
D_DEV = 0.1;
A_DEV = 0.1;
L_DEV = 0.1;

% We represent the particles as state vectors containing: 
% [x; y; vx; vy; dx; dy]
