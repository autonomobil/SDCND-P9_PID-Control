clear all
close all

addpath(genpath('C:\Users\morit\Documents\MATLAB\Matlab_vka\LIB'))

data_folder = 'C:\Users\morit\Downloads\SDCND-P9_PID-Control\build';

first_data_point = 200;
time_limit = 60;

fileList = dir(fullfile(data_folder, '*dataOut-*.dat'));
fileList = {fileList.name}';


for i=1:length(fileList)
    
    M = csvread([data_folder '\' fileList{i}]);
    
    parameters = strsplit(fileList{i}, '-');
    parameters = parameters{2};
    parameters = strsplit(parameters, {'_', '.dat'});
    
    legend_str{i} = ['Kp: ' parameters{1}(1:4), '; Ki: ' parameters{2}(1:7), '; Kd: ' parameters{3}(1:3)];
    
    % out_data << timestep << ',';
    % out_data << cte << ',';
    % out_data << speed << ',';
    % out_data << steer_value << ',';
    % out_data << angle << endl;    
    time =          M(:, 1);
    min(time(time>0))
    time = (time - min(time(time>0)))/1000;
    min(time(time>0))
    [~, endpoint] = min(abs(time - time_limit));
    
    time =          time(first_data_point:endpoint, 1);
    cte =           M(first_data_point:endpoint, 2);
    speed =         M(first_data_point:endpoint, 3);
    steer_value=    M(first_data_point:endpoint, 4);
    angle =         M(first_data_point:endpoint, 5);
    
    temp_inte = cumtrapz(time, abs(cte));
    score(i) = temp_inte(end) * max(time) /1000
    
    if i==1
        f1 = figure;
        set_standardplot
        
        ax1 = subplot(3,1,1);
        h1 = plot(ax1, time, cte);
        hold on
        ylabel(ax1, 'CTE')
        
        y = 7;
        line([0, 80],[y, y], 'Color','m','LineWidth',0.5)
        line([0, 80],[-y, -y], 'Color','m','LineWidth',0.5)
        

        ax2 = subplot(3,1,2);
        h2 = plot(ax2, time, speed);
        hold on
        ylabel(ax2, 'Speed')
        
        ax3 = subplot(3,1,3);
        h3 = plot(ax3, time, steer_value);
        hold on
        ylabel(ax3, 'Steer value')
        
    else
        h1 = plot(ax1, time, cte);
        
        h2 = plot(ax2, time, speed);
        
        h3 = plot(ax3, time, steer_value);
    end
    
end

xlim(ax1, [0 time_limit])
xlim(ax2, [0 time_limit])
xlim(ax3, [0 time_limit])

legend(legend_str)