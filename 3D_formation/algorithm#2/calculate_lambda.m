function lambda = calculate_lambda(len)
%% расчёт коэффициента lambda
global MinDist
min_len = min(len(:));
if min_len==0
    min_len = 0.01;
end
lambda = max(1, MinDist./min_len);
if lambda < 1
    msg=sprintf('Реконфигурация без коллизий');
    disp(msg);
else
    msg=sprintf('Коэффициент lambda : %.1f',lambda);
    disp(msg);
end
