% первая сборка всех частей алгоритма


clc; 
close all;
clearvars;
% ввод количества роботов с консоли
% disp('Введенное количество роботов = активному числу роботов!');
num_of_robots = 5;
% koef = input('Ввод коэффициента увеличения времени задержки  koef = ');
koef = 0.5;
% определяем границы области
% t0 = input('Введите первое граничное значение t0 = ');
% t1 = input('Введите второе граничное значение t1 = ');
max_peak = 5;
location = 1;
if location == 1
    location_mode = 1;
else
    location_mode = location;
    disp('Роботы вне поверхности');
end
% константы 
global min_dist R Density Vel TIMESTEP MASHROB MINR DELAY;
min_dist = 2.5;   % минимальное допустимое расстояние
R = 1;            % радиус робота
Density = 1;
Vel = 1;
MINR = R;
TIMESTEP = 2*min_dist/Vel;
MASHROB = 5;
DELAY = koef*TIMESTEP;      % задержка
num_of_algorithm = 0;
% назначение первоначальных координат
robot_coordinates = generate_robots_v2(location_mode, num_of_robots, max_peak);
% запись в словарь
id = cell(num_of_robots,1);
for i = 1:num_of_robots
    for j =1:num_of_robots
    id{i,j} = sprintf('%d-%d:',i,j);
    end
end
distance=pdist2(robot_coordinates, robot_coordinates);
M = containers.Map(id,distance);
% генерируем поверхность
% координаты поверхности с целевыми точками 
num_of_peak = 4;
Peak = [max_peak*eye(num_of_peak/2) (-1)*max_peak*eye(num_of_peak/2)];
Peak = (reshape(Peak', 2, []))';      
Edge = [1 2;
        1 3;
        1 4;
        2 3;
        2 4;
        3 4];
num_of_edge = length(Edge);
AdjacentPeaks = zeros(num_of_peak, 2);
for i = 1 : num_of_peak
    [row, ~] = find(Edge == i, 2);
    AdjacentPeaks(i, :) = row';
end
clear row;
[ TargetAll, TargetPeak , TargetEdge ] = ...
                                         generate_targets_flat(Peak, Edge);                                                                       
% графики                                     
plot(robot_coordinates(:,1), robot_coordinates(:,2), 'om');
hold on;
grid on;
plotSurface( Peak, Edge, TargetAll )
% проверка корректности радиуса и плотности размещения
if R > 1 
    disp('Радиус некорректен, R = 1');
    R = 1;
end
if Density > 4/(R^2)
    disp('Плотность некорректна, Density = 4/(R^2)');
    Density = 4/(R^2);
end
%%
[x_1_sd, y_1_sd, CC, center] = math_function(robot_coordinates(:,1),...
                                            robot_coordinates(:,2),...
                                            TargetAll(:,1),...
                                            TargetAll(:,2), Peak);
                                        new_rob_cor = [x_1_sd, y_1_sd];
parallel_move(new_rob_cor, num_of_robots, TargetAll, robot_coordinates, center);