%this function calculates the angle between successive best fit lines, each
%with each line including a set amount of points (pointsPerCurvature).  The
%loop progresses through the entire track, calculating the angles of each
%line relative to the previous.  It returns a logical vector denoting a 1
%(motorcycle is turning) or a 0 (motorcycle isnt turning).  This is defined
%by the tolerance (tol).

function [turnflag]= getCurvature(xCourse, yCourse)

tol=45*pi()/180; %angle tolerence in radians

%xCourse=course_interp_X;
%yCourse=course_interp_Y;



pointsPerCurvature=10; %number of points that make up each successive arc
L=length(xCourse);
angleTotal=zeros(L-pointsPerCurvature+1,1);


for i=1:L-(pointsPerCurvature-1)
    %extract desired number of points
    x=xCourse(i:i+pointsPerCurvature-1);
    y=yCourse(i:i+pointsPerCurvature-1);
    
    f=polyfit(x,y,1); %slope is stored in f(1)
    angleTotal(i) = atan(f(1));
   
    
    
end

deltaAngleTotal=diff(angleTotal);
turnflag=deltaAngleTotal>tol;


end
