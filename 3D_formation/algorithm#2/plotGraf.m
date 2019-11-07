function plotGraf(PathKor1, CollidedRobotsNum)
% ������� �� ���������� ����� ��������
PlottingSurface;
% s = zeros(size(CollidedRobotsNum),1);
% t = zeros(size(CollidedRobotsNum),1);
for i=1:size(CollidedRobotsNum)
    s(i) = CollidedRobotsNum(i,1); 
    t(i) = CollidedRobotsNum(i,2);
end
if isempty(s)
    disp('���� �������� �� �����.')
else
% ���������� ��� 98 ���� !!!
s = [s, 98];
t = [t, 98];
G = graph(s,t);
clear x y z;
x = zeros(size(PathKor1,3),1);
y = zeros(size(PathKor1,3),1);
z = zeros(size(PathKor1,3),1);
for i = 1:size(PathKor1,3)
    x(i) = PathKor1(1,1,i);
    y(i) = PathKor1(1,2,i);
    z(i) = PathKor1(1,3,i);
end
plot(G,'XData',x,'YData',y,'ZData',z);
title('���� ��������');
hold on;
grid on;
end
end
