function lambda = calculate_lambda(len)
%% ������ ������������ lambda
% A - ���������� �������
% � - ���������� ����
global min_dist;
lambda = max(1, min_dist./len);
disp('����������� lambda');
disp(max(lambda(:)));
end


