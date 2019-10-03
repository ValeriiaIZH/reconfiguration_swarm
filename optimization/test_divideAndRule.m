function [x_c, y_c,sol, F, M] = test_divideAndRule(XY1, XY2)
x_1 = XY1(:,1)';
y_1 = XY1(:,2)';
x_2 = XY2(:,1)';
y_2 = XY2(:,2)';
M = [ x_1 x_2 ; y_1 y_2]';
polyin_1 = polyshape(M(:,1), M(:,2));
[X_C, Y_C] = centroid(polyin_1);
% ����� ������� �������� ����� ����� ���������
x0 = X_C;
y0 = Y_C;
x_c = X_C;
y_c = Y_C;
% �� ��������� ������, ���������� ����� 2 ����� (x0,y0) � (x1,y1), �����
F = @(x,x1,y1)y0+(y1-y0)*(x-x0)/(x1-x0);
% ���� ������� ������ �������� ���������� �����, �� ���������� �����
% ����� ������ � ���� ��������� ���������, ������ ��� ������ � ����� ������
% ������ ��������� ����� ���� �� �����
oldopts = gaoptimset(@ga);
options = gaoptimset(oldopts,'InitialPopulation',M,'FitnessLimit',0);
[sol,fval] = ga(@DevideSetByLine,2,[],[],[],[],[],[],[],[],options);
% if fval == 0
%   disp('���� �������!')
% end
function Obj = DevideSetByLine(genes)
% �������� ���� �������� �������� ����� ����������� ����� �� ������
% ������� ������
  x1 = genes(1);
  y1 = genes(2);
  Obj = abs(sum(M(:,2)<F(M(:,1),x1,y1))-sum(M(:,2)>F(M(:,1),x1,y1)));
end
end