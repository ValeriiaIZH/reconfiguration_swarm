% �������� � 2 � ��������� ������������
% ��� ������������ ������ ���

clc;
clearvars;
close all;

location_mode = 2;
ROBOTNUM = 100; 
maxPeak = 10;

global MinDist Density R DELTAMIN S FIN_EDGE MASHTAR MASHROB VEL TIMESTEP MINR;
R = 1;              % ������ ������
MINR = R;
MASHTAR = 5;        % ������ ��������� ����� �� �������
MASHROB = 5;        % ������ ������ �� �������
MinDist = 3*R;      % ����������� ���������� ����������
DELTAMIN = 4.3*R;   % ����������� ��� �������
FIN_EDGE = DELTAMIN*fix(sqrt((ROBOTNUM-2)/6)); % ������ ����� �����������
Density = (fix(FIN_EDGE/DELTAMIN)+1)^2;     % ��������� ���������� ���������� �����
% Density = 4;
VEL = 1;            % �������� ������
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
% n = 1;                          % ������� ��������� �����
tic
% ���������� ���������� �����
TargetCor = SettingTargetCube(Delta, TargetNum);
% ������ ������� � ����� ���������
[NewRobotCorCenter, NewRobotCor, RobotCorCenter] = math_function(RobotCor, TargetCor);
% ������������ �������
[PathKor0, PathTime0, RobotCor0, ActiveRobots0] = parallelMove(RobotCor, NewRobotCor, TargetNum);
% if num of algorithm = 2
% ����������� ������� � ����
[PathKor1, PathTime1, ActiveRobots1, ActiveDist1] = homTarMoveTar(NewRobotCor, TargetCor);
% �������� �� ��������
[CollidedRobotsNum, CollidedRobotsCor] = ...
                                  testingSmallCollision_v2(PathKor1, PathTime1);
% ���������� ����� ��������
plotGraf(PathKor1, CollidedRobotsNum);
% �������� ����� �� ������� ������
E = CollidedRobotsNum;
Cycles = grCycleBasis(E);
if isempty(Cycles)
    disp('� ����� �������� ��� ������.');
end

addDelays(CollidedRobotsNum, TargetCor, ActiveRobots1);

% ������ ���������� ��������
% ����� ���� ����������� �������� �������
time1 = toc;
SumDist = sum(ActiveDist1);
msg = sprintf('����� ���� ����������� �������� ������� = %.3f', SumDist);
disp(msg);
% ����� ��������������
ReconfigurationTime = max(PathTime1);
msg = sprintf('����� �������������� = %.1f', ReconfigurationTime);
disp(msg);
disp('������ � ��������� :'); disp(CollidedRobotsNum);
msg = sprintf('����� ������ ��������� = %.3f', time1);
disp(msg);
clear msg;
% %% ����� ����
% allPath = zeros(?,3,size(ActiveRobots3,1));
for i = 1:size(ActiveRobots1,1)
    allPath(:,:,i) = [PathKor0(:,:,i); PathKor1(:,:,i)];
end
%% ��������� �����
% ����������� �����������
 PlottingSurface;
 axis([-80 80 -80 80 -80 80])
 
% ����������� ��������� �����
 for i = 1:TargetNum
     plot3(TargetCor(i,1), TargetCor(i,2), TargetCor(i,3), ...
            'Marker','o', ...
            'MarkerSize', MASHTAR, ...
            'MarkerEdgeColor','m');
 end
 
% ����������� ��������� ������������ �������
for i = 1:size(RobotCor,1)
    plot3(RobotCor(i,1),RobotCor(i,2),RobotCor(i,3),...
            'Marker','o', ...
            'MarkerSize', MASHROB, ...
            'MarkerEdgeColor','k');
end

% ����������� �������� ������� ����� ������������� ��������
for i = 1:size(NewRobotCor,1)
     plot3(NewRobotCor(i,1), NewRobotCor(i,2),NewRobotCor(i,3), ...
           'Marker','.', ...
           'MarkerSize', MASHROB, ...
           'MarkerEdgeColor','b');
end

% �������� �������� �������
AnimatedMovement3(allPath);

plotTraj(ActiveRobots0,PathKor0);
plotTraj(ActiveRobots1,PathKor1);

% %% ��������� ����
% radSfery = 2*R;
% for i = 1:size(allPath,3)
%     [X, Y, Z] = sphere(10);
%     surf((allPath(end, 1, i)+ radSfery*X), ...
%          (allPath(end, 2, i)+ radSfery*Y), ...
%          (allPath(end, 3, i)+ radSfery*Z));
%     shading interp;
%     alpha .5;
% end