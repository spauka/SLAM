function [ prob ] = MeasurementProbability( x_t_bar, z_t )
%MEASUREMENTPROBABILITY Calculate a measurement probability for x_t_bar, 
% given the measurement z_t

    global D_DEV L_DEV X_SIZE Y_SIZE

    % We consider the probability to be the linear combination of the
    % individual measurements.
    
    % Calculate PDF of Laser measurements
    l_meas(1:2) = [X_SIZE; Y_SIZE] - x_t_bar(1:2);
    l_meas(3:4) = x_t_bar(1:2);
    l_meas = l_meas';
    prob = prod(normpdf(l_meas, z_t(1:4), ones(4,1).*L_DEV));

end

