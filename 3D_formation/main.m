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
MASHTAR = 5;
MASHROB = 5;
MinDist = 3*R;
DELTAMIN = 4.3*R;   % ����������� ��� �������
FIN_EDGE = DELTAMIN*fix(sqrt((ROBOTNUM-2)/6)); % ������ ����� �����������
Density = (fix(FIN_EDGE/DELTAMIN)+1)^2;
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
TargetKor = SettingTargetCube(Delta, TargetNum);
%%
figure 
plot3(Robots(:,1), Robots(:,2), Robots(:,3),...
                           'ob',...
                           'MarkerSize',MASHROB);
grid on;
hold on;
plot3(TargetKor(:,1),TargetKor(:,2),TargetKor(:,3),...
                            'or','MarkerSize',MASHTAR);