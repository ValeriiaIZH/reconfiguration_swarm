% VERSION 1
% ������������ ������� ��������� �������, ������ ����������� � ������� (map)
% ���� ���������� ������� � �������
%
clc; 
close all;

% ���� � �������
num_of_robots = input('������� ���������� ������� num = ');
% ������ ������������ � ������� "�������" 
% ���������� ������� �������
t0 = input('������� ������ ��������� �������� t0 = ');
t1 = input('������� ������ ��������� �������� t1 = ');
location = input('������� ������������ �������: 1-������ ������ ������, ����� ������ ��� ����������� loc = ');


% ��������� 
global min_dist R;
min_dist = 1;   % ����������� ���������� ����������
R = 1;          % ������ ������

% ���������� �������������� ���������
robot_coordinates = generate_robots(num_of_robots, t0, t1);

% ���������� ����� ������������
id = cell(num_of_robots,1);
for i = 1:num_of_robots
    for j =1:num_of_robots
    id{i,j} = sprintf('%d-%d:',i,j);
    end
end
distance=pdist2(robot_coordinates,robot_coordinates);
% �������� ������������ ���������
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