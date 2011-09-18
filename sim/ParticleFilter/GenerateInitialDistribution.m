function [ chi ] = GenerateInitialDistribution(  )
%GENERATEINITIALDISTRIBUTION Generate uniform distribution of paritcles
       
    global M X_SIZE Y_SIZE V_MEAN V_DEV 
    
    chi(1,:) = rand(1, M).*X_SIZE;
    chi(2,:) = rand(1, M).*Y_SIZE;
    chi(3:4,:) = normrnd(V_MEAN, V_DEV, 2, M);
    chi(5:6,:) = zeros(2, M);
    
end

