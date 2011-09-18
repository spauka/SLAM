function [ chi_t, chi_t_bar ] = ParticleFilter( chi_t_1, u_t, z_t )
%PARTICLE Perform a particle filter update
%   Perform the update step of the particle filter

    global M
    
    chi_t = zeros(size(chi_t_1));
    omega_t = zeros(M,1);
    chi_t_bar = zeros(size(chi_t_1));
    dist = zeros(M,1);
    
    for m = 1:M
        chi_t_bar(:,m) = StateChange(chi_t_1(:,m), u_t);
        omega_t(m) = MeasurementProbability(chi_t_bar(:,m), z_t);
        dist(m) = sqrt((chi_t_bar(1,m)-10).^2 + (chi_t_bar(2,m)-10).^2);
    end
    
    omega_t = (1./sum(omega_t)).*omega_t;
    omega_t = cumsum(omega_t);
    for m = 1:M
        x = rand(1);
        ind = 1;
        while x > omega_t(ind)
            ind = ind+1;
        end
        chi_t(:,m) = chi_t_bar(:,ind);
    end
end

