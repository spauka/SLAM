function [ chi ] = GenerateInitialDistribution(  )
%GENERATEINITIALDISTRIBUTION Generate uniform distribution of paritcles
       
    global M X_SIZE Y_SIZE V_MEAN V_DEV 
    
    chi(1,:) = normrnd(10,5,1, M);
    chi(2,:) = normrnd(10,5,1, M);
    chi(3:4,:) = normrnd(V_MEAN, V_DEV, 2, M);
    
end

