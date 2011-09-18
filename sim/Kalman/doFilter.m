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
	result_unc = sigma';
end
