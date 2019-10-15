function lambda = calculate_lambda(len)
%% расчёт коэффициента lambda
% A - координаты роботов
% В - координаты цели
global min_dist;
lambda = max(1, min_dist./len);
disp('коэффициент lambda');
disp(max(lambda(:)));
end


