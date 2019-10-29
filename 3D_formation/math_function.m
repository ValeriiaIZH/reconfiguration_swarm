function [NewRobotCorCenter, NewRobotCor, RobotCorCenter] = math_function(RobotCor, TargetCor)
% вычисление центра множеств и коэф. пр€мой
x_1 = RobotCor(:,1);
y_1 = RobotCor(:,2);
z_1 = RobotCor(:,3);
x_2 = TargetCor(:,1);
y_2 = TargetCor(:,2);
z_2 = TargetCor(:,3);
clear s1 s2 s3 s4 s5 s6;
s1 = 0;
s2 = 0;
s3 = 0;
s4 = 0;
s5 = 0;
s6 = 0;
for i=1:size(x_2,1)
    s1 = x_1(i)+s1;
    s2 = x_2(i)+s2;
    s3 = y_1(i)+s3;
    s4 = y_2(i)+s4;
    s5 = z_1(i)+s5;
    s6 = z_2(i)+s6;
end
x_c1 = s1/size(x_1,1);
x_c2 = s2/size(x_2,1);
y_c1 = s3/size(y_1,1);
y_c2 = s4/size(y_2,1);
z_c1 = s5/size(z_1,1);
z_c2 = s6/size(z_2,1);

RobotCorCenter = [x_c1, y_c1, z_c1];
TargetCorCenter = [x_c2, y_c2, z_c2];
%% параллельный перенос
% парметры направл€ющего вектора
param_a1  = TargetCorCenter(:,1)-RobotCorCenter(:,1);
param_a2 = TargetCorCenter(:,2)- RobotCorCenter(:,2);
param_a3  = TargetCorCenter(:,3)- RobotCorCenter(:,3);

NewRobotCorCenter = TargetCorCenter;
NewRobotCor = [param_a1+RobotCor(:,1) param_a2+RobotCor(:,2) param_a3+RobotCor(:,3)];
end
% A = [x_1_sd; y_1_sd]';
% B = [x_1; y_1]';
% dist = pdist2(A, B);
% %% определение соотношени€ роботов и таргентных точек
% N = size(A,1);
% matchAtoB=NaN(N,1);
% for i=1:N
%     dist(:,matchAtoB(1:i-1))=Inf;           % выбранные точки ¬ не могут быть новыми ближайшими точками
%     [~,matchAtoB(i)] = min(dist(i,:));
% end
% matchBtoA = NaN(size(B,1),1);
% matchBtoA(matchAtoB)=1:N;
% remaining_indices = find(isnan(matchBtoA));
% C = arrayfun(@(i) [A(matchBtoA(i),:); B(i,:)], 1:N, 'uni', false);
% for i=1:N
%     CC(i) = pdist2([C{i}(1); C{i}(3)]',[C{i}(2); C{i}(4)]');
% end
% % disp('–ассто€ни€ между точками ROB+TARG');
% % disp(CC')
% clear A B;
% 
% varargout{1} = CC;
% varargout{2} = [x_c2, y_c2];
% end

