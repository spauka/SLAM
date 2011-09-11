%x is a 1x4 row vector mu_x mu_y sigma_xx sigmaxy sigma_yx sigma_yy
%px is in the same format as x
%u is the column vector [dx ; dy]
function x_bar = prediction(px,u)
    A = eye(2); % x_t = x_{t-1} + dx
    B = eye(2); %therefore A=I, B=I
    global d_dev
    R = d_dev*d_dev*eye(2); %movement covariance

    pmu = px(1:2)';
    psigma = [px(3:4) ; px(5:6)];
    
    mu_bar = A*pmu + B*u;
    sigma_bar = A*psigma*(A') + R;
    
    x_bar = [mu_bar' sigma_bar(1,:) sigma_bar(2,:)];
end