% алгоритм в трёхмерном пространстве
clc;
clearvars;
close all;

% location_mode = input('Размещение роботов: 1-снаружи, иначе внутри = ');
% ROBOTNUM = input('Введите количество роботов: ROBOTNUM = ');
% maxPeak = input('Введите максимальную координату вершины: maxPeak = ');

location_mode = 2;
ROBOTNUM = 100; 
maxPeak = 100;

global MinDist Density R DELTAMIN FIN_EDGE MASHTAR MASHROB;
R = 1;              % радиус робота
MASHTAR = 5;        % размер таргентой точки на графике
MASHROB = 5;        % размер робота на графике
MinDist = 3*R;      % минимальное допустимое расстояние
DELTAMIN = 4.3*R;   % минимальный шаг решетки
FIN_EDGE = DELTAMIN*fix(sqrt((ROBOTNUM-2)/6)); % размер ребра поверхности
Density = (fix(FIN_EDGE/DELTAMIN)+1)^2;     % плотность размещения таргентных точек
% Density = 4;

%% поверхность КУБ
S = [0,        0,        0;        ...  % S(1,:)
     0,        0,        FIN_EDGE; ...  % S(2,:)
     0,        FIN_EDGE, FIN_EDGE; ...  % S(3,:)
     0,        FIN_EDGE, 0;        ...  % S(4,:)      
     FIN_EDGE, FIN_EDGE, 0;        ...  % S(5,:)
     FIN_EDGE, 0,        0;        ...  % S(6,:)
     FIN_EDGE, 0,        FIN_EDGE; ...  % S(7,:)
     FIN_EDGE, FIN_EDGE, FIN_EDGE];     % S(8,:)
 
Robots = generateRobots(location_mode, ROBOTNUM, maxPeak);

% % проверка корректности
% if R > 1 
%     disp('Радиус некорректен, R = 1');
%     R = 1;
% end
% if Density > 4/(R^2)
%     disp('Плотность некорректна, Density = 4');
%     Density = 4;
% end
Delta = DELTAMIN;
M = fix(FIN_EDGE/Delta)+1;      % число точек в ряду
TargetNum = 6*M^2-12*M+8;       % общее число таргетных точек
n = 1;                          % счетчик таргетных точек
% заполнение таргентрых точек
TargetCor = SettingTargetCube(Delta, TargetNum);
[NewRobotCorCenter, NewRobotCor] = math_function(RobotCor, TargetCor);
%% расчёт траектории для параллельного переноса
tic
% определение числа активных роботов
if ROBOTNUM >= TargetNum        
    ActiveRobotsUnsort = zeros(TargetNum, 3);
else
    ActiveRobotsUnsort = zeros(ROBOTNUM, 3);
end
% создание массива активных роботов
RobotCor2 = RobotCor;
ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1);                                                                                                        
for i = 1:size(TargetCor,1)                         
    TargetRast = zeros(1, size(RobotCor2,1));
    distdist = zeros(1, size(RobotCor2,1));
    for j = 1:size(RobotCor2,1)
            distdist(i,j)= pdist2(TargetCor(i,:), RobotCor2(j,:));
%             TargetRast(1, j)= pdist2(TargetCor(i,:), RobotCor2(j,:));                                                   % оставшихся роботов
    if i == j
            TargetRast(1, j) = distdist(i,j);
    end 
    end
    [DistValue, Ind] = max(TargetRast);
    ActiveDist(i,1) =  DistValue;                  % значение расстояния 
    ActiveRobotsUnsort(i, :) = RobotCor2(Ind, :);  
%     RobotCor2(Ind, :) = [];
    if isempty(RobotCor2)
        break;
    end
end                                                
ActiveRobots = ActiveRobotsUnsort;
ActiveNewRobCor = NewRobotCor;
clear ActiveRobotsUnsort RobotCor2;
[PathCor1, PathTime1] = animbycoordinates(ActiveDist, ActiveRobots, ActiveNewRobCor);
%% оценка параметров движения
% сумма длин перемещений активных роботов
SumDist = sum(ActiveDist);
msg = fprintf('Сумма длин перемещений активных роботов, SumDist = %.3f', SumDist);
disp(msg);
% время реконфигурации
ReconfigurationTime = max(PathTime1);
msg = fprintf('Время реконфигурации, ReconfigurationTime = %.1f', ReconfigurationTime);
disp(msg);
clear msg;
time1 = toc;




%% отрисовка всего
% отображение поверхности
 PlottingSurface;
% отображение таргетных точек
 for i = 1:TargetNum
     plot3(ActiveNewRobCor(i,1), ...
           ActiveNewRobCor(i,2), ...
           ActiveNewRobCor(i,3), ...
           'or','MarkerSize', MASHTAR);
 end
% отображение исходного расположения роботов
for i = 1:size(RobotCor,1)
    plot3(RobotCor(i,1),RobotCor(i,2),RobotCor(i,3),...
            'Marker','o', ...
            'MarkerSize', MASHROB, ...
            'MarkerEdgeColor','k');
end
% % расчетная траектория движения
%     for i = 1:size(ActiveRobots,1)
%         plot3([ActiveRobots(i,1), ActiveNewRobCor(i,1)],...
%               [ActiveRobots(i,2), ActiveNewRobCor(i,2)],...
%               [ActiveRobots(i,3), ActiveNewRobCor(i,3)],...
%                 'g');
%     end

% анимация движения роботов
AnimatedMovement3(PathCor1);


% 
% % [CollidedRobotsNum, CollidedRobotsCor] = ...
% %                                   testingSmallCollision_v2(PathCor, PathTime);
                        