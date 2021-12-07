function sanjiao(cx,cy,theta,clr)
x = [1 -0.5 -0.5]*2.5;
y = [0 0.5 -0.5]*2.5;
R=[cos(theta) -sin(theta);
   sin(theta)  cos(theta)];
pts = R*[x;y]+[cx;cy];
patch(pts(1,:),pts(2,:),clr);
end