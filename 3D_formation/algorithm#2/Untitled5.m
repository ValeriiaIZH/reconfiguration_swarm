% алгоритм № 2 в трёхмерном пространстве
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
VEL = 1;            % скорость робота
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
tic
% заполнение таргентрых точек
TargetCor = SettingTargetCube(Delta, TargetNum);
% расчёт центров и новых координат
[NewRobotCorCenter, NewRobotCor, RobotCorCenter] = math_function(RobotCor, TargetCor);
% паралллеьный перенос
[PathKor0, PathTime0, RobotCor0, ActiveRobots0] = parallelMove(RobotCor, NewRobotCor, TargetNum);
% if num of algorithm = 2
% перемещение роботов к цели
[PathKor1, PathTime1, ActiveRobots1, ActiveDist1] = homTarMoveTar(NewRobotCor, TargetCor);
% проверка на коллизии
[CollidedRobotsNum, CollidedRobotsCor] = ...
                                  testingSmallCollision_v2(PathKor1, PathTime1);
% построение графа коллизий
plotGraf(PathKor1, CollidedRobotsNum);
% проверка графа на наличие циклов
E = CollidedRobotsNum;
Cycles = grCycleBasis(E);
if isempty(Cycles)
    disp('В графе коллизий нет циклов.');
end

addDelays(CollidedRobotsNum, TargetCor, ActiveRobots1);

% оценка параметров движения
% сумма длин перемещений активных роботов
time1 = toc;
SumDist = sum(ActiveDist1);
msg = sprintf('Сумма длин перемещений активных роботов = %.3f', SumDist);
disp(msg);
% время реконфигурации
ReconfigurationTime = max(PathTime1);
msg = sprintf('Время реконфигурации = %.1f', ReconfigurationTime);
disp(msg);
disp('Роботы с коллизией :'); disp(CollidedRobotsNum);
msg = sprintf('Время работы алгоритма = %.3f', time1);
disp(msg);
clear msg;
% %% общий путь
% allPath = zeros(?,3,size(ActiveRobots3,1));
for i = 1:size(ActiveRobots1,1)
    allPath(:,:,i) = [PathKor0(:,:,i); PathKor1(:,:,i)];
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

% анимация движения роботов
AnimatedMovement3(allPath);

plotTraj(ActiveRobots0,PathKor0);
plotTraj(ActiveRobots1,PathKor1);

% %% отрисовка сфер
% radSfery = 2*R;
% for i = 1:size(allPath,3)
%     [X, Y, Z] = sphere(10);
%     surf((allPath(end, 1, i)+ radSfery*X), ...
%          (allPath(end, 2, i)+ radSfery*Y), ...
%          (allPath(end, 3, i)+ radSfery*Z));
%     shading interp;
%     alpha .5;
% end