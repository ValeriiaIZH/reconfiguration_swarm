function lambda = calculate_lambda(len)
%% расчёт коэффициента lambda
% A - координаты роботов
% В - координаты цели
global min_dist
% len = pdist2(A, B);
min_len = min(len(:));
if min_len==0
    min_len = 0.01;
end
lambda = max(1, min_dist./min_len);
% lambda = min_dist./len;
% if lambda==1
%     disp('Расчёт коэффициента lambda для гомотетии не нужен. '); 
% else
    disp('Коэффициент lambda');
    disp(lambda);
% end
end


