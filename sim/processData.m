filename = 'robotdata_1'

d = getData(filename);

result = kalman(d);
time = result(:,1);
x_bar = result(:,2);
y_bar = result(:,3);
x = result(:,8);
y = result(:,9);

figure(1)
title('x bar, smoothed x and actual x')
hold on
plot(time,x_bar, 'g');
plot(time(2:end), d(:,10),'m');
plot(time,x, 'b');
figure(2)
title('ybar, smoothed y and actual y')
hold on
plot(time,y_bar, 'g');
plot(time(2:end), d(:,11),'m');
plot(time,y, 'b');
