filename = 'sim4';

d = getData(filename);

result = kalman(d);
time = result(:,1);
x_bar = result(:,2);
y_bar = result(:,3);
x_bar_err = result(:,4).^0.5;
y_bar_err = result(:,7).^0.5;
x = result(:,8);
y = result(:,9);
x_err = result(:,10).^0.5;
y_err = result(:,13).^0.5;




figure(1)
clf
title('x bar, smoothed x and actual x')
hold on
errorbar(time,x_bar,x_bar_err, 'g');
plot(time(2:end), d(:,10),'m');
errorbar(time,x,x_err, 'b');

figure(2)
clf
title('ybar, smoothed y and actual y')
hold on
errorbar(time,y_bar,y_bar_err, 'g');
plot(time(2:end), d(:,11),'m');
errorbar(time,y,y_err, 'b');

figure(3)
clf
title('filtered x and y vs actual position')
errorbarxy(x,y,x_err,x_err,y_err,y_err,'Color','b','LineWidth', 2);
hold on
plot(d(:,10),d(:,11),'g');


