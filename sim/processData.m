filename = 'robotdata_1'

d = getData(filename);

result = kalman(d);
time = result(:,1);
x_bar = result(:,2);
y_bar = result(:,3);


figure(1)
title('x bar and actual x')
hold on
plot(time,x_bar);
plot(time(2:end), d(:,10));
figure(2)
title('ybar and actual y')
hold on
plot(time,y_bar);
plot(time(2:end), d(:,11));

