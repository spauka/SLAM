function data = getData(filename)
    global t dx dy ax ay right up left down
    fid = fopen(filename);
    C = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n', 'delimiter', ',');
    mC = cell2mat(C);
    data = mC(:,1:17);
    t = data(:,1);
    dx = data(:,2);
    dy = data(:,3);
    ax = data(:,4);
    ay = data(:,5);
    right = data(:,6);
    up = data(:,7);
    left = data(:,8);
    down = data(:,9);
    fclose(fid);
end

