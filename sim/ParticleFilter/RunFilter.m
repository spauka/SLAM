% Read data from file
data = getData('sim5');

% Load Constants
Constants

% Get Initial Distribution

dataSize = size(data);

err = zeros(2,dataSize(1));
RMSE = zeros(1000,dataSize(1));

for count = 1:6
	chi = GenerateInitialDistribution();
	result = zeros(4,dataSize(1));
	for X = 1:dataSize(1)
		 u = [data(X,2);data(X,3);data(X,4);data(X,5)];
		 z = [data(X,6);data(X,7);data(X,8);data(X,9)];
		 chi = ParticleFilter(chi,u,z);
		 result(:,X) = mean(chi,2);
		 err(:,X) = [sqrt(sum((chi(1,:)-data(X,10)).^2)/dataSize(1)); sqrt(sum((chi(2,:)-data(X,11)).^2)/dataSize(1))];
		 RMSE(count,X) = sqrt(err(1,X).^2 + err(2,X).^2);
	end
	count
	subplot(3,2,count)
	scatter(result(1,:), result(2,:))
	hold on
	scatter(data(:,10), data(:,11))
	hold off
	drawnow
end

