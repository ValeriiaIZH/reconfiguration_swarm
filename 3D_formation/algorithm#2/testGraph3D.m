% ��c��������� ����� � ��������� ������������
clearvars;
clc;

s = [6 4 6];
t = [3 1 5];
G = graph(s,t);

x = [0 0.5 -0.5 -0.5 0.5 0 ];
y = [0 0.5 0.5 -0.5 -0.5 0 ];
z = [5 3 3 3 3 0 ];
plot(G,'XData',x,'YData',y,'ZData',z)