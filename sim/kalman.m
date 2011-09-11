% x = [time x(unfiltered) y(unfiltered) x(filtered) y(filtered)] 
function result = kalman(data)
    time = data(:,1);
    dx = data(:,2);
    dy = data(:,3);
    ax = data(:,4);
    ay = data(:,5);
    right = data(:,6);
    up = data(:,7);
    left = data(:,8);
    down = data(:,9);
    
    global d_dev a_dev l_dev
    d_dev = data(1,16);
    a_dev = data(1,17);
    l_dev = data(1,18);
    
    x_bar = zeros(length(time)+1,6); %without filtering
    x_bar(1,:) = [10 10 0 0 0 0];

    x = x_bar;
    
    %not have zero-indices is REALLY bloody annoying!
    for i = 1:length(time)
        x_bar(i+1,:) = prediction(x_bar(i,:),[dx(i) ; dy(i)]);
        x(i+1,:) = measurement_laser(x_bar(i+1,:), [left(i) ; right(i) ;down(i); up(i)]);
    end
    result = [[0;time] x_bar x];
end
