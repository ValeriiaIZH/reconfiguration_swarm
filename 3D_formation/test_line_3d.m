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

% определение числа активных роботов
if ROBOTNUM >= TargetNum        
    ActiveRobotsUnsort = zeros(TargetNum, 3);
else
    ActiveRobotsUnsort = zeros(ROBOTNUM, 3);
end
% создание массива активных роботов
RobotCor = Robots;

[NewRobotCorCenter, NewRobotCor] = math_function(RobotCor, TargetCor);





% ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1); % массив расстояний 
%                                                    % от активного робота 
%                                                    % до его таргетной точки
% for i = 1:size(TargetKor,1)
%     TargetRast = zeros(1, size(RobotKor2,1));      % массив расстояний от 
%                                                    % точки до каждого из 
%                                                    % оставшихся роботов 
%%
% figure 
% plot3(Robots(:,1), Robots(:,2), Robots(:,3),...
%                            'ob',...
%                            'MarkerSize',MASHROB);
% grid on;
% hold on;
% plot3(TargetCor(:,1),TargetCor(:,2),TargetCor(:,3),...
%                            'or',...
%                            'MarkerSize',MASHTAR);
%                         
                        