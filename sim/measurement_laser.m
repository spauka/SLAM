
%z is the column vector [left ; right ; down ; up]
function x = measurement_laser(x_bar,z)
    function z = h(x)
        z = [x(1) ; 20 - x(1); x(2); 20- x(2)];
    end
    global l_dev
    Q = l_dev*l_dev*eye(4); %covariance of z
    H = [ 1 0 ; -1 0 ; 0 1 ; 0 -1]; %jacobian of h

    mu_bar = x_bar(1:2)';
    sigma_bar = [x_bar(3:4); x_bar(5:6)];
    
    K =  sigma_bar * (H') * inv(H * sigma_bar * (H') + Q);
    mu = mu_bar + K * (z - h(mu_bar));

    sigma = (eye(2) - K*H)*sigma_bar;
    x = [mu' sigma(1,:) sigma(2,:)];
end
