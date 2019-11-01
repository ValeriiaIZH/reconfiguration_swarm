% �������� � ��������� ������������
% ��� ������������ ������ ���

clc;
clearvars;
close all;

location_mode = 2;
ROBOTNUM = 100; 
maxPeak = 10;

global MinDist Density R DELTAMIN S FIN_EDGE MASHTAR MASHROB VEL TIMESTEP;
R = 1;              % ������ ������
MASHTAR = 5;        % ������ ��������� ����� �� �������
MASHROB = 5;        % ������ ������ �� �������
MinDist = 3*R;      % ����������� ���������� ����������
DELTAMIN = 4.3*R;   % ����������� ��� �������
FIN_EDGE = DELTAMIN*fix(sqrt((ROBOTNUM-2)/6)); % ������ ����� �����������
Density = (fix(FIN_EDGE/DELTAMIN)+1)^2;     % ��������� ���������� ���������� �����
% Density = 4;
VEL = 1;          % �������� ������
TIMESTEP = 2*R/VEL; % ��� ������������� �� ������� 
                  % �� ��������� �������� ������ �������, 
                  % �� ������� ����� ��������
                  % ����������, ������ ������ ��������
%% ����������� ���
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
M = fix(FIN_EDGE/Delta)+1;      % ����� ����� � ����
TargetNum = 6*M^2-12*M+8;       % ����� ����� ��������� �����
n = 1;                          % ������� ��������� �����
% ���������� ���������� �����
TargetCor = SettingTargetCube(Delta, TargetNum);

[NewRobotCorCenter, NewRobotCor, RobotCorCenter] = math_function(RobotCor, TargetCor);
tic
% ����������� ����� �������� �������
if ROBOTNUM >= TargetNum        
    ActiveRobotsUnsort = zeros(TargetNum, 3);
else
    ActiveRobotsUnsort = zeros(ROBOTNUM, 3);
end
% �������� ������� �������� �������
RobotCor2 = RobotCor;

ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1); % ������ ���������� 
                                                   % �� ��������� ������ 
                                                   % �� ��� ��������� �����
for i = 1:size(TargetCor,1)                        % ������ ���������� �� 
    TargetRast = zeros(1, size(RobotCor2,1));       % ����� �� ������� ��
    for j = 1:size(RobotCor2,1)
            distdist(i,j)=pdist2(TargetCor(i,:), RobotCor2(j,:));
%             TargetRast(1, j)= pdist2(TargetCor(i,:), RobotCor2(j,:));                                                   % ���������� �������
    if i == j
            TargetRast(1, j) = distdist(i,j);
    end 
    end
    [DistValue, Ind] = max(TargetRast);
    ActiveDist(i,1) =  DistValue;                  % �������� ���������� 
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

% ������ ���������� ��������
% ����� ���� ����������� �������� �������
SumDist = sum(ActiveDist);
msg = sprintf('����� ���� ����������� �������� �������, SumDist = %.3f', SumDist);
disp(msg);
% ����� ��������������
ReconfigurationTime = max(PathTime1);
msg = sprintf('����� ��������������, ReconfigurationTime = %.1f', ReconfigurationTime);
disp(msg);
clear msg;
% 
% % [CollidedRobotsNum, CollidedRobotsCor] = ...
% %                                   testingSmallCollision_v2(PathCor, PathTime);
% time1 = toc;
% disp(time1);
% 
% ��������� �����
% ����������� �����������
% figure 
% plot3(TargetCor(:,1),TargetCor(:,2),TargetCor(:,3),...
%                            'or',...
%                            'MarkerSize',MASHTAR); 
% grid on;
% hold on;

 PlottingSurface;
 
% ����������� ��������� �����
 for i = 1:TargetNum
     plot3(ActiveNewRobCor(i,1), ...
           ActiveNewRobCor(i,2), ...
           ActiveNewRobCor(i,3), ...
           'or','MarkerSize', MASHTAR);
 end
 
% ����������� ��������� ������������ �������
for i = 1:size(RobotCor,1)
    plot3(RobotCor(i,1),RobotCor(i,2),RobotCor(i,3),...
            'Marker','o', ...
            'MarkerSize', MASHROB, ...
            'MarkerEdgeColor','k');
end
% 
% % ��������� ���������� ��������
% %     for i = 1:size(ActiveRobots,1)
% %         plot3([ActiveRobots(i,1), ActiveTargets(i,1)],...
% %               [ActiveRobots(i,2), ActiveTargets(i,2)],...
% %               [ActiveRobots(i,3), ActiveTargets(i,3)],...
% %                 'g');
% %     end;
% 
% % % ���������� �������� �� �������
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
% �������� �������� �������
  AnimatedMovement3(PathCor1);
%   
% % ��������� ����
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















%% �����������
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