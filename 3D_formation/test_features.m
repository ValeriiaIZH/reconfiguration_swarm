% алгоритм в трёхмерном пространстве
% для тестирования всяких фич

clc;
clearvars;
close all;

location_mode = 2;
ROBOTNUM = 100; 
maxPeak = 10;

global MinDist Density R DELTAMIN S FIN_EDGE MASHTAR MASHROB VEL TIMESTEP;
R = 1;              % радиус робота
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
n = 1;                          % счетчик таргетных точек
% заполнение таргентрых точек
TargetCor = SettingTargetCube(Delta, TargetNum);

[NewRobotCorCenter, NewRobotCor, RobotCorCenter] = math_function(RobotCor, TargetCor);
tic
% определение числа активных роботов
if ROBOTNUM >= TargetNum        
    ActiveRobotsUnsort = zeros(TargetNum, 3);
else
    ActiveRobotsUnsort = zeros(ROBOTNUM, 3);
end
% создание массива активных роботов
RobotCor2 = RobotCor;

ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1); % Массив расстояний 
                                                   % от активного робота 
                                                   % до его таргетной точки
for i = 1:size(TargetCor,1)                        % Массив расстояний от 
    TargetRast = zeros(1, size(RobotCor2,1));       % точки до каждого из
    for j = 1:size(RobotCor2,1)
            distdist(i,j)=pdist2(TargetCor(i,:), RobotCor2(j,:));
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

% оценка параметров движения
% сумма длин перемещений активных роботов
SumDist = sum(ActiveDist);
msg = sprintf('Сумма длин перемещений активных роботов, SumDist = %.3f', SumDist);
disp(msg);
% время реконфигурации
ReconfigurationTime = max(PathTime1);
msg = sprintf('Время реконфигурации, ReconfigurationTime = %.1f', ReconfigurationTime);
disp(msg);
clear msg;
% 
% % [CollidedRobotsNum, CollidedRobotsCor] = ...
% %                                   testingSmallCollision_v2(PathCor, PathTime);
% time1 = toc;
% disp(time1);
% 
% отрисовка всего
% отображение поверхности
% figure 
% plot3(TargetCor(:,1),TargetCor(:,2),TargetCor(:,3),...
%                            'or',...
%                            'MarkerSize',MASHTAR); 
% grid on;
% hold on;

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
% 
% % расчетная траектория движения
% %     for i = 1:size(ActiveRobots,1)
% %         plot3([ActiveRobots(i,1), ActiveTargets(i,1)],...
% %               [ActiveRobots(i,2), ActiveTargets(i,2)],...
% %               [ActiveRobots(i,3), ActiveTargets(i,3)],...
% %                 'g');
% %     end;
% 
% % % траектория движения на графике
% %     for i = 1:size(ActiveRobots,1)
% %         for j = 1:size(PathCor,1)
% %              plot3(PathCor(:, 1, i), ...
% %                    PathCor(:, 2, i), ... 
% %                    PathCor(:, 3, i), 'g');
% % %                    'x','MarkerSize', 2.5);
% %         end
% %     end
% 
% 
% анимация движения роботов
  AnimatedMovement3(PathCor1);
%   
% % отрисовка сфер
% % mashtabSfery = 1.5*R;
% % for i = 1:size(PathCor,3)
% %     [X, Y, Z] = sphere(8);
% % %     surf((PathKor(end, 1, i)+R*X), (PathKor(end, 2, i)+R*Y), (PathKor(end, 3, i)+R*Z));
% %     surf((PathCor(end, 1, i)+ mashtabSfery*X), ...
% %             (PathCor(end, 2, i)+mashtabSfery*Y), ...
% %             (PathCor(end, 3, i)+mashtabSfery*Z));
% %     shading interp;
% %     alpha .8;
% % end
% 















%% отображение
figure 
plot3(RobotCor(:,1), RobotCor(:,2), RobotCor(:,3),...
                           'ob',...
                           'MarkerSize',MASHROB);
grid on;
hold on;
plot3(TargetCor(:,1),TargetCor(:,2),TargetCor(:,3),...
                           'or',...
                           'MarkerSize',MASHTAR);                        
plot3(NewRobotCor(:,1),NewRobotCor(:,2),NewRobotCor(:,3),...
                           'og',...
                           'MarkerSize',MASHTAR);

plot3(RobotCorCenter(:,1),RobotCorCenter(:,2),RobotCorCenter(:,3),...
                           'or',...
                           'MarkerSize',MASHTAR);  
grid on;
hold on;
plot3(NewRobotCorCenter(:,1),NewRobotCorCenter(:,2),NewRobotCorCenter(:,3),...
                           'og',...
                           'MarkerSize',MASHTAR);  