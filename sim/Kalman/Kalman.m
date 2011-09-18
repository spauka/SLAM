function [ mu_t, sigma_t ] = Kalman( mu_t1, sigma_t1, ut, zt )
%KALMAN Perform an iteration of the kalman filter
	
	global A B C Q R;	

	mu_t_bar = A*mu_t1 + B*ut;
	sigma_t_bar = A*sigma_t1*A' + R;
	
	K = sigma_t_bar*C'*inv(C*sigma_t_bar*C' + Q);
	mu_t = mu_t_bar + K*(zt - C*mu_t_bar);
	sigma_t = (eye(size(sigma_t_bar)) - K*C)*sigma_t_bar;

end

