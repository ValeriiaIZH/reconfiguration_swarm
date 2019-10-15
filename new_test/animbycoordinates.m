function [PathKor, PathTime] = animbycoordinates(ActiveDist, ActiveRobots, ActiveTargets)
% �������������� �������� �������
global Vel TIMESTEP

PathTime = zeros(size(ActiveDist));            % ����� ������ � ����
PathKor = zeros((ceil((max(ActiveDist)/Vel)/TIMESTEP))+1, ... 
                3,size(ActiveRobots,1));      % ���������� ������� � ����
                    % PathKor(i,:,:) - i-� ���������� ������� �� �������
                    % PathKor(:,j,:) - ���������� ��������� x, y, z
                    % PathKor(:,:,k) - k-� �����

for i = 1:size(ActiveRobots,1)
    TimeCount = 0;                            % ���������� ������ ������� t
    PathTime(i) = ActiveDist(i)/Vel;            % ��������� ����� ���� ������
    T = 0;                                    % ����� �������� ������
    F = Vel/ActiveDist(i);
    u = (ActiveTargets(i,1) - ActiveRobots(i,1))*F;
    v = (ActiveTargets(i,2) - ActiveRobots(i,2))*F;        
    A = ActiveRobots(i,1) - u*T;
    B = ActiveRobots(i,2) - v*T;

    for j = 1:(ceil(PathTime(i)/TIMESTEP))
        PathKor(j, 1, i) = u*TimeCount + A;
        PathKor(j, 2, i) = v*TimeCount + B;
        TimeCount = TimeCount + TIMESTEP;
    end
    PathKor((j+1):end, 1, i) = ActiveTargets(i,1);
    PathKor((j+1):end, 2, i) = ActiveTargets(i,2);
end
end

