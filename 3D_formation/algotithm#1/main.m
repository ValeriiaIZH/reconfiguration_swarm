% �������� � ��������� ������������
clc;
clearvars;
close all;

% location_mode = input('���������� �������: 1-�������, ����� ������ = ');
% ROBOTNUM = input('������� ���������� �������: ROBOTNUM = ');
% maxPeak = input('������� ������������ ���������� �������: maxPeak = ');

location_mode = 2;
ROBOTNUM = 100; 
maxPeak = 100;

global MinDist Density R DELTAMIN FIN_EDGE MASHTAR MASHROB;
R = 1;              % ������ ������
MASHTAR = 5;        % ������ ��������� ����� �� �������
MASHROB = 5;        % ������ ������ �� �������
MinDist = 3*R;      % ����������� ���������� ����������
DELTAMIN = 4.3*R;   % ����������� ��� �������
FIN_EDGE = DELTAMIN*fix(sqrt((ROBOTNUM-2)/6)); % ������ ����� �����������
Density = (fix(FIN_EDGE/DELTAMIN)+1)^2;     % ��������� ���������� ���������� �����
% Density = 4;

%% ����������� ���
S = [0,        0,        0;        ...  % S(1,:)
     0,        0,        FIN_EDGE; ...  % S(2,:)
     0,        FIN_EDGE, FIN_EDGE; ...  % S(3,:)
     0,        FIN_EDGE, 0;        ...  % S(4,:)      
     FIN_EDGE, FIN_EDGE, 0;        ...  % S(5,:)
     FIN_EDGE, 0,        0;        ...  % S(6,:)
     FIN_EDGE, 0,        FIN_EDGE; ...  % S(7,:)
     FIN_EDGE, FIN_EDGE, FIN_EDGE];     % S(8,:)
 
Robots = generateRobots(location_mode, ROBOTNUM, maxPeak);

% % �������� ������������
% if R > 1 
%     disp('������ �����������, R = 1');
%     R = 1;
% end
% if Density > 4/(R^2)
%     disp('��������� �����������, Density = 4');
%     Density = 4;
% end
Delta = DELTAMIN;
M = fix(FIN_EDGE/Delta)+1;      % ����� ����� � ����
TargetNum = 6*M^2-12*M+8;       % ����� ����� ��������� �����
n = 1;                          % ������� ��������� �����
% ���������� ���������� �����
TargetCor = SettingTargetCube(Delta, TargetNum);
[NewRobotCorCenter, NewRobotCor] = math_function(RobotCor, TargetCor);
%% ������ ���������� ��� ������������� ��������
tic
% ����������� ����� �������� �������
if ROBOTNUM >= TargetNum        
    ActiveRobotsUnsort = zeros(TargetNum, 3);
else
    ActiveRobotsUnsort = zeros(ROBOTNUM, 3);
end
% �������� ������� �������� �������
RobotCor2 = RobotCor;
ActiveDist = zeros(size(ActiveRobotsUnsort,1), 1);                                                                                                        
for i = 1:size(TargetCor,1)                         
    TargetRast = zeros(1, size(RobotCor2,1));
    distdist = zeros(1, size(RobotCor2,1));
    for j = 1:size(RobotCor2,1)
            distdist(i,j)= pdist2(TargetCor(i,:), RobotCor2(j,:));
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
%% ������ ���������� ��������
% ����� ���� ����������� �������� �������
SumDist = sum(ActiveDist);
msg = fprintf('����� ���� ����������� �������� �������, SumDist = %.3f', SumDist);
disp(msg);
% ����� ��������������
ReconfigurationTime = max(PathTime1);
msg = fprintf('����� ��������������, ReconfigurationTime = %.1f', ReconfigurationTime);
disp(msg);
clear msg;
time1 = toc;




%% ��������� �����
% ����������� �����������
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
% % ��������� ���������� ��������
%     for i = 1:size(ActiveRobots,1)
%         plot3([ActiveRobots(i,1), ActiveNewRobCor(i,1)],...
%               [ActiveRobots(i,2), ActiveNewRobCor(i,2)],...
%               [ActiveRobots(i,3), ActiveNewRobCor(i,3)],...
%                 'g');
%     end

% �������� �������� �������
AnimatedMovement3(PathCor1);


% 
% % [CollidedRobotsNum, CollidedRobotsCor] = ...
% %                                   testingSmallCollision_v2(PathCor, PathTime);
                        