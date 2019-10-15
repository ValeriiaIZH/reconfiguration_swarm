% ������ ������ ���� ������ ���������

clc; 
close all;
clearvars;
% ���� ���������� ������� � �������
num_of_robots = input('������� ���������� ������� num = ');
% koef = input('���� ������������ ���������� ������� ��������  koef = ');
koef = 0.5;
max_peak = input('������������ ���������� ����� max_peak = ');
all_calculated_time = 0;
location_mode = 0;
tic
% ��������� 
global min_dist R Density Vel TIMESTEP MASHROB MINR DELAY;
min_dist = 1;             % ����������� ���������� ����������
R = 1;                      % ������ ������
Density = 1;                % ���������
Vel = 1;                    % ��������
MINR = R;                   % ����������� ������
TIMESTEP = 2*min_dist/Vel;  % ��� �������������
MASHROB = 5;                % ������ ������� �� �������
DELAY = koef*TIMESTEP;      % ��������
num_of_algorithm = 0;
% ���������� �������������� ���������
robot_coordinates = generate_robots_v2(location_mode, num_of_robots, max_peak);
% % ������ � �������
% id = cell(num_of_robots,1);
% for i = 1:num_of_robots
%     for j =1:num_of_robots
%     id{i,j} = sprintf('%d-%d:',i,j);
%     end
% end
% distance=pdist2(robot_coordinates, robot_coordinates);
% M = containers.Map(id,distance);
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
%% �������                                     
% plot(robot_coordinates(:,1), robot_coordinates(:,2), 'om');
% hold on;
% grid on;
% plotSurface( Peak, Edge, TargetAll )
%% �������� ������������ ������� � ��������� ����������
if R > 1 
    disp('������ �����������, R = 1');
    R = 1;
end
if Density > 4/(R^2)
    disp('��������� �����������, Density = 4/(R^2)');
    Density = 4/(R^2);
end
%%
[x_1_sd, y_1_sd, CC, center] = math_function(robot_coordinates(:,1),...
                                            robot_coordinates(:,2),...
                                            TargetAll(:,1),...
                                            TargetAll(:,2), Peak);
new_rob_cor = [x_1_sd, y_1_sd];
ActiveDist = parallel_move(new_rob_cor, num_of_robots, TargetAll, robot_coordinates, center);
[PathKor0, PathTime0, ActiveDist0] = calculate_rob_to_tar(num_of_robots, TargetAll, new_rob_cor);
% �������� �� ��������
[CollidedRobotsNum0,...
  CollidedRobotsKor0, lambda] = my_collisiontest(PathKor0, PathTime0);
% disp(lambda);
% disp(max(lambda(:)));
if isempty(CollidedRobotsNum0)
        disp('�������������� ��� ��������');
        num_of_algorithm = 0;
else
%     disp('�������� ����� ��������(��������������� ���� �������)');
%     disp(CollidedRobotsNum0);
%     ������ ����. lambda
%     lambda = calculate_lambda(dist0);
    disp('lambda:');
    my_lambda = max(lambda(:));
    disp(my_lambda);
    num_of_algorithm = 1;
end
if my_lambda > 20
    disp('����� ������� �������� lambda!');
    num_of_algorithm = 2;
end

if num_of_algorithm == 0
    disp(' ');
end
% if num_of_algorithm == 1
%         [new_x_1_sd,  new_y_1_sd, ...
%         new_x_2, new_y_2] = Homothetic_transformation(lambda,...
%                       robot_coordinates, TargetAll, center, num_of_robots);
%         ActiveDist1 = homothetic_move(num_of_robots, new_x_1_sd, new_y_1_sd, ...
%                         robot_coordinates, new_x_2, new_y_2);
%         [PathKor1, PathTime1] = after_homothetic(new_x_2, new_y_2,... 
%                                 num_of_robots, new_x_1_sd, new_y_1_sd);
%         % �������� �� ��������
%         [CollidedRobotsNum1,...
%             CollidedRobotsKor1, dist1] = my_collisiontest(PathKor1, PathTime1);
%         if isempty(CollidedRobotsNum1)
%             disp('�������� �1 ��������!');
%             [PathKor2, PathTime2, ActiveRobots, ActiveDist2] = last_path(num_of_robots,...
%                                               TargetAll, new_x_2, new_y_2);
%              ActiveDist = [ActiveDist, ActiveDist0, ActiveDist1, ActiveDist2];                            
%         else
%             disp('�������� ����� �������� ����� ���������:');
%             disp(CollidedRobotsNum1);
%             disp('������� ��������� �������� �2!');
%             [ActiveDist] = algorithm_2(Peak, Edge,num_of_robots, new_rob_cor, TargetAll, PathKor0,...
%                      CollidedRobotsNum0);
%         end
% end
% if num_of_algorithm == 2 
%      disp('������� ��������� �������� �2!');
%     [ActiveDist] = algorithm_2(Peak, Edge,num_of_robots, new_rob_cor, TargetAll, PathKor0,...
%                   CollidedRobotsNum0);
% end
% time = toc;
% all_calculated_time = all_calculated_time + time;
% msg = sprintf('����� ����� ���������� t = %6.4g', all_calculated_time); 
% disp(msg);
% % ����� ���� ����������� �������� �������
% sum_dist = sum(ActiveDist);
% msg = sprintf('����� ���� ����������� �������� ������� L = %.3f', sum_dist);
% disp(msg);