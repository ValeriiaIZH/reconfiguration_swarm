function [PathKor, PathTime] = animbycoordinates(ActiveDist, ActiveRobots, ActiveTargets)
% покоординатное движение роботов
global VEL TIMESTEP

PathTime = zeros(size(ActiveDist));            % время робота в пути
PathKor = zeros((ceil((max(ActiveDist)/VEL)/TIMESTEP))+1, ... 
                3,size(ActiveRobots,1));      % координаты роботов в пути
                    % PathKor(i,:,:) - i-я координата роботов по времени
                    % PathKor(:,j,:) - координаты положения x, y, z
                    % PathKor(:,:,k) - k-й робот

for i = 1:size(ActiveRobots,1)
    TimeCount = 0;                            % дискретный момент времени t
    PathTime(i) = ActiveDist(i)/VEL;            % расчетное время пути робота
    T = 0;                                    % время задержки робота
    F = VEL/ActiveDist(i);
    u = (ActiveTargets(i,1) - ActiveRobots(i,1))*F;
    v = (ActiveTargets(i,2) - ActiveRobots(i,2))*F; 
    w = (ActiveTargets(i,3) - ActiveRobots(i,3))*F;
    A = ActiveRobots(i,1) - u*T;
    B = ActiveRobots(i,2) - v*T;
    C = ActiveRobots(i,3) - w*T;

    for j = 1:(ceil(PathTime(i)/TIMESTEP))
        PathKor(j, 1, i) = u*TimeCount + A;
        PathKor(j, 2, i) = v*TimeCount + B;
        PathKor(j, 3, i) = w*TimeCount + C;
        TimeCount = TimeCount + TIMESTEP;
    end
    PathKor((j+1):end, 1, i) = ActiveTargets(i,1);
    PathKor((j+1):end, 2, i) = ActiveTargets(i,2);
    PathKor((j+1):end, 3, i) = ActiveTargets(i,3);
end
end

