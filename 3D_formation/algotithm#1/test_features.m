% алгоритм в трёхмерном пространстве
% для тестирования всяких фич

clc;
clearvars;
close all;

location_mode = 2;
ROBOTNUM = 100; 
maxPeak = 10;

global MinDist Density R DELTAMIN S FIN_EDGE MASHTAR MASHROB VEL TIMESTEP MINR;
R = 1;              % радиус робота
MINR = R;
MASHTAR = 5;        % размер таргентой точки на графике
MASHROB = 5;        % размер робота на графике
MinDist = 3*R;      % минимальное допустимое расстояние
DELTAMIN = 4.3*R;   % минимальный шаг решетки
FIN_EDGE = DELTAMIN*fix(sqrt((ROBOTNUM-2)/6)); % размер ребра поверхности
Density = (fix(FIN_EDGE/DELTAMIN)+1)^2;     % плотность размещения таргентных точек
% Density = 4;
VEL = 1;          % скорость робота
TIMESTEP = 2*R/VEL; % шаг дискретизации по времени 
                  % по умолчанию полагаем равным времени, 
                  % за которое робот проходит
                  % расстояние, равное своему диаметру
%% поверхность КУБ
S = [0,        0,        0;        ...  % S(1,:)
     0,        0,        FIN_EDGE; ...  % S(2,:)
     0,        FIN_EDGE, FIN_EDGE; ...  % S(3,:)
     0,        FIN_EDGE, 0;        ...  % S(4,:)      
     FIN_EDGE, FIN_EDGE, 0;        ...  % S(5,:)
     FIN_EDGE, 0,        0;        ...  % S(6,:)
     FIN_EDGE, 0,        FIN_EDGE; ...  % S(7,:)
     FIN_EDGE, FIN_EDGE, FIN_EDGE];     % S(8,:)
 
RobotCor = generateRobots(location_mode, ROBOTNUM, maxPeak);

Delta = DELTAMIN;
M = fix(FIN_EDGE/Delta)+1;      % число точек в ряду
TargetNum = 6*M^2-12*M+8;       % общее число таргетных точек
% n = 1;                          % счетчик таргетных точек

% заполнение таргентрых точек
TargetCor = SettingTargetCube(Delta, TargetNum);
tic
% расчёт центров и новых координат
[NewRobotCorCenter, NewRobotCor, RobotCorCenter] = math_function(RobotCor, TargetCor);
% паралллеьный перенос
[PathKor0, ~, RobotCor0] = parallelMove(RobotCor, NewRobotCor, TargetNum);
% гомомтетия
distantionAll = pdist2(NewRobotCor,RobotCor0);
%расчёт лямбда для гомотетии
lambda = calculate_lambda(distantionAll);
% lambda = 2;
[homRobot, homTarget] = homotheticTransformation(lambda, NewRobotCor,...
                                   TargetCor, NewRobotCorCenter, ROBOTNUM);
                               % homRobot - координаты роботов после
                               % гомотетии с коэф. лямбда
                               % homTarget - координаты поверхности после
                               % гомотетии с коэф. лямбда
[PathKor1, PathTime1, ActiveDist1] = homotheticMove(homRobot,...
                                                 NewRobotCor, homTarget);
                                             % движение роботов от
                                             % смещенных коордит до
                                             % кооддинат гомотетии
[PathKor2, PathTime2] = homRobMovehomTar(homRobot,homTarget);
                                            % движение роботов после
                                            % гомотетии к таргентным точкам
                                            % после гомотетии
[CollidedRobotsNum, CollidedRobotsCor] = ...
                                  testingSmallCollision_v2(PathKor2, PathTime2);
                                             % тест на коллизию. движение
                                             % роботов от homRobot до
                                             % homTarget
[PathKor3, PathTime3, ActiveRobots3, ActiveDist3] = homTarMoveTar(homTarget, TargetCor);
                                             % движение роботов до целевых
                                             % точек
% оценка параметров движения
% сумма длин перемещений активных роботов
time1 = toc;
SumDist = sum(ActiveDist3);
msg = sprintf('Сумма длин перемещений активных роботов, SumDist = %.3f', SumDist);
disp(msg);
% время реконфигурации
ReconfigurationTime = max(PathTime1);
msg = sprintf('Время реконфигурации, ReconfigurationTime = %.1f', ReconfigurationTime);
disp(msg);
msg = sprintf('Роботы с коллизией : %.3f', CollidedRobotsNum);
disp(msg);
msg = sprintf('Время работы алгоритма = %.3f', time1);
disp(msg);
clear msg;
%% общий путь
% allPath = zeros(?,3,size(ActiveRobots3,1));
for i = 1:size(ActiveRobots3,1)
allPath(:,:,i) = [PathKor0(:,:,i); PathKor1(:,:,i);...
                  PathKor2(:,:,i); PathKor3(:,:,i)];
end
%% отрисовка всего
% отображение поверхности
 PlottingSurface;
 axis([-80 80 -80 80 -80 80])
 
% отображение таргетных точек
 for i = 1:TargetNum
     plot3(TargetCor(i,1), TargetCor(i,2), TargetCor(i,3), ...
            'Marker','o', ...
            'MarkerSize', MASHTAR, ...
            'MarkerEdgeColor','m');
 end
 
% отображение исходного расположения роботов
for i = 1:size(RobotCor,1)
    plot3(RobotCor(i,1),RobotCor(i,2),RobotCor(i,3),...
            'Marker','o', ...
            'MarkerSize', MASHROB, ...
            'MarkerEdgeColor','k');
end

% отображение коорднат роботов после параллельного переноса
for i = 1:size(NewRobotCor,1)
     plot3(NewRobotCor(i,1), NewRobotCor(i,2),NewRobotCor(i,3), ...
           'Marker','.', ...
           'MarkerSize', MASHROB, ...
           'MarkerEdgeColor','b');
end
 
% отображение коорднат роботов после гомотетии
for i = 1:size(homRobot,1)
     plot3(homRobot(i,1), homRobot(i,2),homRobot(i,3), ...
           'Marker','.', ...
           'MarkerSize', MASHROB, ...
           'MarkerEdgeColor','c');
end
 
 % отображение коорднат поверхности после гомотетии
for i = 1:size(homTarget,1)
     plot3(homTarget(i,1), homTarget(i,2),homTarget(i,3), ...
           'Marker','.', ...
           'MarkerSize', MASHTAR, ...
           'MarkerEdgeColor','g');
end
% анимация движения роботов
AnimatedMovement3(allPath);

%% отрисовка сфер
radSfery = 2*R;
for i = 1:size(allPath,3)
    [X, Y, Z] = sphere(10);
    surf((allPath(end, 1, i)+ radSfery*X), ...
         (allPath(end, 2, i)+ radSfery*Y), ...
         (allPath(end, 3, i)+ radSfery*Z));
    shading interp;
    alpha .5;
end




