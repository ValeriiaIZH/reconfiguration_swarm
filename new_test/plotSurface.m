function [] = plotSurface( Peak, Edge, TargetAll ) 

% функция отрисовки поверхности с таргентными точками
% close all;
% figure();  
% figure('Units', 'normalized');
axis normal;

% for i = 1:size(Face)
%     line ([Peak(Face(i,1),1), Peak(Face(i,2),1), Peak(Face(i,3),1), Peak(Face(i,1),1)], ...
%           [Peak(Face(i,1),2), Peak(Face(i,2),2), Peak(Face(i,3),2), Peak(Face(i,1),2)], ...
%           [Peak(Face(i,1),3), Peak(Face(i,2),3), Peak(Face(i,3),3), Peak(Face(i,1),3)]  ...
%           );
%     if i ~= 1 
%         continue;
%     else
%      hold on;
%     end;
% end;
for i = 1:size(TargetAll)
    plot (TargetAll(i,1), TargetAll(i,2),'Marker','o','MarkerFaceColor','g');
    if i ~= 1 
        continue;
    else
     hold on;
    end
end
clear i;
for i = 1:size(Edge)
         line ([Peak(Edge(i,1),1), Peak(Edge(i,2),1)], ...
               [Peak(Edge(i,1),2), Peak(Edge(i,2),2)])
         hold on;
end
grid on;

xlabel('Ox');
ylabel('Oy');
zlabel('Oz');