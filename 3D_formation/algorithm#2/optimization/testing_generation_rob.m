% VERSION 1
% тестирование функции генерации роботов, запись результатов в словарь (map)
% ввод количества роботов с консоли
%
clc; 
close all;

% ввод в консоль
num_of_robots = input('Введите количество роботов num = ');
% роботы генерируются в области "квадрат" 
% определяем границы области
t0 = input('Введите первое граничное значение t0 = ');
t1 = input('Введите второе граничное значение t1 = ');
location = input('Введите расположение роботов: 1-роботы вокруг центра, иначе роботы вне поверхности loc = ');


% константы 
global min_dist R;
min_dist = 1;   % минимальное допустимое расстояние
R = 1;          % радиус робота

% назначение первоначальных координат
robot_coordinates = generate_robots(num_of_robots, t0, t1);

% расстояния между координатами
id = cell(num_of_robots,1);
for i = 1:num_of_robots
    for j =1:num_of_robots
    id{i,j} = sprintf('%d-%d:',i,j);
    end
end
distance=pdist2(robot_coordinates,robot_coordinates);
% проверка корректности координат
for i=1:num_of_robots
    for j=1:num_of_robots
       answer = true;
       while answer
            if (distance(i,j) < min_dist) && (distance(i,j)~=0)
                x_1 = randi([t0 t1],1,num);
                y_1 = randi([t0 t1],1,num);
                new_coordinates = [x_1; y_1]';
                new_distance = pdist2(coordinates, coordinates);
            else
                answer = false;
            break
            end
       end
    end
end
M = containers.Map(id,distance);