function [ robots ] = generateRobots( location_mode, ROBOTNUM, maxPeak )
% location_mode = 1 - роботы вокруг центра
% location_mode ~= 1 - роботы вне поверхности

    global MinDist;    
    shiftIn = 3;
    shiftOut = 6;

    bool = true;

    if (location_mode == 1)
% %     роботы "вокруг" 
        randmin = (-1)*shiftIn*maxPeak;
        randmax = shiftIn*maxPeak;
        rand = false(ROBOTNUM, 3);
    else 
% %     роботы "вне"
        randmin = (-1)*shiftIn*maxPeak;
        randmax = shiftOut*maxPeak;
        rand = logical(randi([0, 1], ROBOTNUM, 3));
    end  
    i = 0; 
    while bool
            % координаты роботов
         robots = randi([randmin, randmax], ROBOTNUM, 3); 
         robots(rand) = robots(rand)*(-1);
         % для бага с нулевой координатой: 
         a = false(size(robots));
         a(robots == 0)= true;
         robots(a) = 0.0001;

         % проверка на исходное расстояние
        DefDist = pdist(robots); % исходное расстояние между роботами
        if  min(DefDist) < MinDist 
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