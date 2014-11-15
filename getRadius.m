%this function uses Sagitta's formula to calculate the radius of curvature
%for 3 points
function r=getRadius(x,y)

L=length(x);
r=zeros(L-2,1);


for i=1:L-2
    
    %calculates 0.5 * straightline distance between first and last point
    dist = 0.5* sqrt((x(i+2)-x(i))^2+(y(i+2)-y(i))^2);

    %calculate midpoint between first and last point
    xMid = (x(i)+x(i+2))/2;
    yMid = (y(i)+y(i+2))/2;

    %calculate Sagitta (distance between the second point and the midpoint of
    %the first and last
    S = 0.5* sqrt((x(i+1)-xMid)^2+(y(i+1)-yMid)^2);

    %apply Sagittas formula
    r(i)=(S^2+dist^2)/(2*S);
       
end


%change any "inf" to a large number to avoid errors later on
yesIfInf = r==inf;
r(yesIfInf)=999999999999999;







    
end
