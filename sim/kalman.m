% x = [time x(unfiltered) y(unfiltered) x(filtered) y(filtered)] 
function result = kalman(data)
    time = data(:,1);
    dx = data(:,2);
    dy = data(:,3);
    right = data(:,6);
    up = data(:,7);
    left = data(:,8);
    down = data(:,9);
    
    global d_dev l_dev
    d_dev = data(1,16);
    l_dev = data(1,18);
    
    x_bar = zeros(length(time)+1,6); %without filtering
    x = zeros(length(time)+1,6);
    x(1,:) = [10 10 0 0 0 0];
    x_bar(1,:) = x(1,:);
    %not have zero-indices is REALLY bloody annoying!
    for i = 1:length(time)
        x_bar(i+1,:) = prediction(x_bar(i,:),[dx(i) ; dy(i)]);
        x(i+1,:) = measurement(x_bar(i+1,:), [left(i) ; right(i) ;down(i); up(i)]);
    end
    result = [[0;time] x_bar x];
end