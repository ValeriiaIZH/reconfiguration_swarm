% VERSION 2
% ������������ ������� ��������� �������, ������ ����������� � ������� (map)
% ���� ���������� ������� � �������
clc; 
close all;
clearvars;
% ���� � �������
num_of_robots = input('������� ���������� ������� num = ');
% ���������� ������� �������
% t0 = input('������� ������ ��������� �������� t0 = ');
% t1 = input('������� ������ ��������� �������� t1 = ');
max_peak = input('max_peak = ');
location = input('������� ������������ �������: "1"-������ ������ ������, ����� ������ ��� ����������� loc = ');
if location == 1
    location_mode = 1;
else
    location_mode = location;
    disp('������ ��� �����������');
end
% ��������� 
global min_dist R Density;
min_dist = 1;   % ����������� ���������� ����������
R = 1;          % ������ ������
Density = 1;
% ���������� �������������� ���������
robot_coordinates = generate_robots_v2(location_mode, num_of_robots, max_peak);
% ������ � �������
id = cell(num_of_robots,1);
for i = 1:num_of_robots
    for j =1:num_of_robots
    id{i,j} = sprintf('%d-%d:',i,j);
    end
end
distance=pdist2(robot_coordinates, robot_coordinates);
M = containers.Map(id,distance);
% ���������� �����������
% ���������� ����������� � �������� ������� 
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
% �������                                     
plot(robot_coordinates(:,1), robot_coordinates(:,2), 'om');
hold on;
grid on;
plotSurface( Peak, Edge, TargetAll )