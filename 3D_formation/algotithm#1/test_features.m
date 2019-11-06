% �������� � ��������� ������������
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
% n = 1;                          % ������� ��������� �����

% ���������� ���������� �����
TargetCor = SettingTargetCube(Delta, TargetNum);
tic
% ������ ������� � ����� ���������
[NewRobotCorCenter, NewRobotCor, RobotCorCenter] = math_function(RobotCor, TargetCor);
% ������������ �������
[PathKor0, ~, RobotCor0] = parallelMove(RobotCor, NewRobotCor, TargetNum);
% ����������
distantionAll = pdist2(NewRobotCor,RobotCor0);
%������ ������ ��� ���������
lambda = calculate_lambda(distantionAll);
% lambda = 2;
[homRobot, homTarget] = homotheticTransformation(lambda, NewRobotCor,...
                                   TargetCor, NewRobotCorCenter, ROBOTNUM);
                               % homRobot - ���������� ������� �����
                               % ��������� � ����. ������
                               % homTarget - ���������� ����������� �����
                               % ��������� � ����. ������
[PathKor1, PathTime1, ActiveDist1] = homotheticMove(homRobot,...
                                                 NewRobotCor, homTarget);
                                             % �������� ������� ��
                                             % ��������� ������� ��
                                             % ��������� ���������
[PathKor2, PathTime2] = homRobMovehomTar(homRobot,homTarget);
                                            % �������� ������� �����
                                            % ��������� � ���������� ������
                                            % ����� ���������
[CollidedRobotsNum, CollidedRobotsCor] = ...
                                  testingSmallCollision_v2(PathKor2, PathTime2);
                                             % ���� �� ��������. ��������
                                             % ������� �� homRobot ��
                                             % homTarget
[PathKor3, PathTime3, ActiveRobots3, ActiveDist3] = homTarMoveTar(homTarget, TargetCor);
                                             % �������� ������� �� �������
                                             % �����
% ������ ���������� ��������
% ����� ���� ����������� �������� �������
time1 = toc;
SumDist = sum(ActiveDist3);
msg = sprintf('����� ���� ����������� �������� �������, SumDist = %.3f', SumDist);
disp(msg);
% ����� ��������������
ReconfigurationTime = max(PathTime1);
msg = sprintf('����� ��������������, ReconfigurationTime = %.1f', ReconfigurationTime);
disp(msg);
msg = sprintf('������ � ��������� : %.3f', CollidedRobotsNum);
disp(msg);
msg = sprintf('����� ������ ��������� = %.3f', time1);
disp(msg);
clear msg;
%% ����� ����
% allPath = zeros(?,3,size(ActiveRobots3,1));
for i = 1:size(ActiveRobots3,1)
allPath(:,:,i) = [PathKor0(:,:,i); PathKor1(:,:,i);...
                  PathKor2(:,:,i); PathKor3(:,:,i)];
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
 
% ����������� �������� ������� ����� ���������
for i = 1:size(homRobot,1)
     plot3(homRobot(i,1), homRobot(i,2),homRobot(i,3), ...
           'Marker','.', ...
           'MarkerSize', MASHROB, ...
           'MarkerEdgeColor','c');
end
 
 % ����������� �������� ����������� ����� ���������
for i = 1:size(homTarget,1)
     plot3(homTarget(i,1), homTarget(i,2),homTarget(i,3), ...
           'Marker','.', ...
           'MarkerSize', MASHTAR, ...
           'MarkerEdgeColor','g');
end
% �������� �������� �������
AnimatedMovement3(allPath);

%% ��������� ����
radSfery = 2*R;
for i = 1:size(allPath,3)
    [X, Y, Z] = sphere(10);
    surf((allPath(end, 1, i)+ radSfery*X), ...
         (allPath(end, 2, i)+ radSfery*Y), ...
         (allPath(end, 3, i)+ radSfery*Z));
    shading interp;
    alpha .5;
end




