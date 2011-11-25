function [ x_m_t ] = StateChange( x_m_t_1, u_t )
%STATECHANGE Do a time update of each of the states.
%   This function is situation dependant

    global DELTA_T UP_DEV UA_DEV
    
    x_m_t(3:4) = x_m_t_1(3:4) + u_t(3:4).*DELTA_T + normrnd(0, UP_DEV,2,1);
    x_m_t(1:2) = x_m_t_1(1:2) + (x_m_t_1(3:4).*DELTA_T) + normrnd(0, UA_DEV,2,1);
    
    x_m_t = x_m_t';

end

