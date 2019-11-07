function [ TargetAll, varargout ] = generate_targets_flat( Peak, Edge )

global  Density;

NumOfPeak = length(Peak);
NumOfEdge = length(Edge);

% создание ТТ и точек портала
DistBtwTarget = 10/sqrt(Density);
PeakDist = pdist2(Peak(Edge(1, 1),:), Peak(Edge(1, 2),:));    
NumOfEdgeTarget = floor(PeakDist/DistBtwTarget);   

        % первого типа
TargetPeak = Peak;
TargetTypeOne = ones(NumOfPeak,1); % тип точки 1

%         % второго типа      
TargetEdge = zeros(NumOfEdgeTarget, 2);
n = 1;
for i = 1: NumOfEdge
    for j = 1: (NumOfEdgeTarget-1)
        TargetEdge(n,:) =  Peak(Edge(i, 1),:) + ...
            j*((Peak(Edge(i,2),:)-Peak(Edge(i,1),:))/NumOfEdgeTarget);
        n = n + 1;       
    end
end
TargetTypeTwo = 2 * ones(length(TargetEdge),1); % тип точки 2

TargetPeak = roundn(TargetPeak, -2);
TargetEdge = roundn(TargetEdge, -2);

% общий массив таргетных точек
TargetAll  = cat(1, TargetPeak, TargetEdge);
TargetType = cat(1, TargetTypeOne, TargetTypeTwo);
clear i j n z row TargetTypeOne TargetTypeTwo ;

varargout{1} = TargetPeak;
varargout{2} = TargetEdge;
varargout{3} = TargetType;