function PlottingSurface

global S FIN_EDGE

% figure;
% size = get(0, 'ScreenSize');
figure('Units', 'normalized'); % 

% axis normal;

plot3(S(:,1),S(:,2),S(:,3));

xlim([-FIN_EDGE 1.5*FIN_EDGE]);
ylim([-FIN_EDGE 1.5*FIN_EDGE]);
zlim([-FIN_EDGE 1.5*FIN_EDGE]);
hold on; 
grid on;

xlabel('Ox');
ylabel('Oy');
zlabel('Oz');
line([S(4,1) S(1,1) S(6,1)],   ...
     [S(4,2) S(1,2) S(6,2)],   ...
     [S(4,3) S(1,3) S(6,3)]);
line([S(2,1) S(7,1)],          ...
     [S(2,2) S(7,2)],          ...
     [S(2,3) S(7,3)]);
line([S(5,1) S(8,1) S(3,1)],   ...
     [S(5,2), S(8,2), S(3,2)], ...
     [S(5,3), S(8,3), S(3,3)]);