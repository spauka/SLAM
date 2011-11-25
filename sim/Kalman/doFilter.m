data = getData('sim4');
StateMatrices

mu = mu_0;
sigma = sigma_0;
result = [];
result_unc = [];

dataSize = size(data);

for X = 1:dataSize(1)
	u = [data(X,4);data(X,5)];
	z = [data(X,8);data(X,9)];
	[mu, sigma] = Kalman(mu, sigma, u, z)
	result(X,:) = mu';
	result_unc(X,:) = [sigma(1,1) sigma(2,2) sigma(3,3) sigma(4,4)];
end
