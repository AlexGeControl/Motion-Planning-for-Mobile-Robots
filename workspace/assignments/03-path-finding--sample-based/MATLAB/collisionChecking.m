function feasible=collisionChecking(startPose,goalPose,map)
feasible=true;
dir=atan2(goalPose(1)-startPose(1),goalPose(2)-startPose(2));
for r=0:0.5:sqrt(sum((startPose-goalPose).^2))
    posCheck = startPose + r.*[sin(dir) cos(dir)];
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ...
         feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;
    end
end
if ~feasiblePoint([floor(goalPose(1)),ceil(goalPose(2))],map), feasible=false; end

function feasible=feasiblePoint(point,map)
feasible=true;
if ~(point(1)>=1 && point(1)<=size(map,2) && ... % x in map
     point(2)>=1 && point(2)<=size(map,1) && ... % y in map
     map(point(2),point(1))==255) % x,y is Free
    feasible=false;
end