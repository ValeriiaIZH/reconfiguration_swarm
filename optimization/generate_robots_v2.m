function [ robot_coordinates ] = generate_robots_v2(location_mode, num, max_peak)
global min_dist;
    shiftIn = 3;    % смещение роботов
    shiftOut = 6;   
    bool = true;
    if (location_mode == 1)
        % роботы "вокруг" 
        randmin = (-1)*shiftIn*max_peak;
        randmax = shiftIn*max_peak;
        rand = false(num, 2);
    else 
        % роботы "вне"
        randmin = (-1)*shiftOut*max_peak;
        randmax = shiftIn*max_peak;
        rand = logical(randi([0, 1], num, 2));
    end 
    i = 0; 
    while bool
         % координаты роботов
         robot_coordinates = randi([randmin, randmax], num, 2); 
         robot_coordinates(rand) = robot_coordinates(rand)*(-1);
         % для бага с нулевой координатой: 
         a = false(size(robot_coordinates));
         a(robot_coordinates == 0)= true;
         robot_coordinates(a) = 0.0001;

         % проверка на исходное расстояние
        dist = pdist(robot_coordinates); % исходное расстояние между роботами
        if  min(dist) < min_dist 
            disp('Wrong distribution. Change configuration...');            
        else 
            bool = false;
        end
        i = i + 1;
        if i == 25
            disp('Не удалось установить корректное начальное расположение!');
            break;
        end
    end
end

