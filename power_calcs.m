%--------------------------------------------------------------------
% Resistance racing simulation
% Connor Archard 10/23/14
%
% References: 
% Motorcycle Dynamics by Vittore Cossalter
%
% Notes:
%
%
%--------------------------------------------------------------------
clc;
%clear all; %commented out because this clears all debug break points
close all;

% Plotting Parameters
% -------------------------------------------------------------------
plot_mode = 3;

% Bike Parameters
% -------------------------------------------------------------------
mass = 226.8;     % 500lbs in kg
hub_rad = 0.4572;   % inner diameter of the tires
rolling_rad = 0.53;   % radius of the wheel
cross_section = 0.75;   % cross sectional area in m^2
%torque_curve = [];  % torque curve from a motor would go here indexed by RPM
%gear_ratio = 0; % a factor to convert engine RPM to velocity -need wheel rad
c_drag = .56; % from bike forum, speculated range is .55 to .65
area = .75; %1.05; % 2.5ft wide, 4.5ft tall rhombus
rho = 1.184; % mass density of air at 25C
g = 9.81; % acceleration do to gravity in m/s^2
timestep = .1; % seconds
max_lateral_g = 1; % starting with 1 as a rough estimate of no camber turn
% The course
% -------------------------------------------------------------------

%SQUARE COURSE
%course_x=[1000,-1000,-1000,+1000,1000]; 
%course_y=[1000,1000,-1000,-1000,0];

%SEMI CIRCLE COURSE
%course_x=-1000:1000;
%course_y=sqrt(1000^2-course_x.^2);

%STRIAGHT LINE COURSE
course_x=-1000:1000;
course_y=-1000:1000;

%AIRFOIL COURSE
%course_x=xlsread('mh114','A1:A68');
%course_y=xlsread('mh114','B1:B68');

%SINE WAVE COURSE
%course_x=-1000:1000;
%course_y=100*sin(course_x/50);



%interpolation of the course data. For explantation:
    %http://blogs.mathworks.com/steve/2012/07/06/walking-along-a-path/)
    %this code will interpolate a specificed (num_points) number of points
    %to interpolate over.  The points will be equally spaced in terms of
    %PATH distance
xy=[course_x' course_y'];
d = diff(xy,1);
dist_from_vertex_to_vertex = hypot(d(:,1), d(:,2));
cumulative_dist_along_path = [0;cumsum(dist_from_vertex_to_vertex,1)];
num_points = 500; %set the number of interpolation points
dist_steps = linspace(0, cumulative_dist_along_path(end), num_points);
points = interp1(cumulative_dist_along_path, xy, dist_steps);
course_interp_X=points(:,1);
course_interp_Y=points(:,2);

%calculate course radiuses
radiusTotal=getRadius(course_interp_X, course_interp_Y);
radiusTotal(end+1)=inf; radiusTotal(end+1)=inf; %small error/bug - need to fix later


%vectorLength=num_points;
vectorLength=4000;

%calculates the path distance between each point
dist_between_steps=dist_steps(2)-dist_steps(1);

%calculates total path length
Total_path_length=sum(diff(dist_steps));

% Current Values
% -------------------------------------------------------------------
%variables below store history of all values
t=zeros(vectorLength,1);
x_pos = zeros(vectorLength,1);
y_pos = zeros(vectorLength,1);
x_velocity = zeros(vectorLength,1);
y_velocity = zeros(vectorLength,1);
v_goal = zeros(vectorLength,1);
x_accel = zeros(vectorLength,1);
y_accel = zeros(vectorLength,1);


power_consumed = 0;
Force_total_X = 0;    
Force_total_Y = 0;
Total_dist_traveled=0;

%initialize starting position
x_pos(1)=course_interp_X(1);
y_pos(1)=course_interp_Y(1);


% The main loop
% -------------------------------------------------------------------
%i is the main counter for everything but below:
%j is the counter for the interpolated course vector
j=1; %initialize j

%determine INITIAL theta (this is angle of current position to desired position)
theta=atan2d((course_interp_Y(2)-y_pos(1)),(course_interp_X(2)-x_pos(1)));
theta_m3 = 0;
theta_m2 = 0;
theta_m1 = 0;

%initialize radius
radius=radiusTotal(1);

debugFlag1=0;
debugFlag2=0;
for i = 2:vectorLength
    
    %update time vector
    t(i)=t(i-1)+timestep; 
    
    %determine theta (this is angle of current position to desired position)
   if Total_dist_traveled>=dist_between_steps*j
    while Total_dist_traveled>=dist_between_steps*j
        j=j+1;
    end
     Velocity_Mag = sqrt(x_velocity(i)^2 + y_velocity(i)^2);
     look_ahead = round(1.5^(Velocity_Mag/5));

     %update radius
     radius=radiusTotal(j);
     
     
     if(j+look_ahead > num_points) %if true, end of course reached
        break;
     end
     %theta=atan2d((course_interp_Y(j+look_ahead)-y_pos(i-1)),(course_interp_X(j+look_ahead)-x_pos(i-1)));
   end
   
%    theta_m3 = theta_m2;
%    theta_m2 = theta_m1;
%    theta_m1 = theta;
    theta=atan2d((course_interp_Y(j)-y_pos(i-1)),(course_interp_X(j)-x_pos(i-1)));
    

    
    
    
    
   
   % estimates the radius by treating the last four points as a wedge of
   % a circle. By checking the portion of the circumference we have
   % travelled, we can get a radius. From the radius we calculate, we get a
   % maximum allowable velocity.
%    %delta_theta = abs(theta_m3-theta);
%    if delta_theta>0 % Prevents dividing by 0
%        radius = ((dist_between_steps * 4)*(360/delta_theta))/(2*pi());
%    else
%        radius = 10000000000;
%    end
   Velocity_Mag = sqrt(max_lateral_g*radius);
   v_goal_x = Velocity_Mag*cosd(theta);
   v_goal_y = Velocity_Mag*sind(theta);
   v_goal(i)=sqrt( v_goal_x^2+v_goal_y^2);
   
    %determine whether bike has reached its goal velocity
    if (abs(v_goal_x)>=abs(x_velocity(i)) && abs(v_goal_y)>=abs(y_velocity(i)))
        % engine is on code here
        F_motor = 500; %random value for code testing
       
        F_motor_X = F_motor * cosd(theta);
        F_motor_Y = F_motor * sind(theta);
        F_brake_X = 0;
        F_brake_Y = 0;
        
        debugFlag1=debugFlag1+1;
        
    else
       % Braking code here
       F_motor=0; %random value for code testing
       F_motor_X =0;
       F_motor_Y =0;
       F_brake_X = 500; %positive value
       F_brake_Y = 500; %positive value
       
       debugFlag2=debugFlag2+1;
    end
    
    %theta = arctan(course_y(i)/course_x(i));
    %r_velocity(i) = sqrt(x_velocity(i).^2 + y_velocity(i).^2); %calculates magnitude of velocity vector ("true" velocity)
    
    %Calculate rolling friction force
    c_roll_X = (x_velocity(i)>0)*0.2+0.2*(x_velocity(i)/60.67);   % rolling fric
    c_roll_Y = (y_velocity(i)>0)*0.2+0.2*(y_velocity(i)/60.67);   % rolling fric
   
    F_roll_X = c_roll_X * mass * g;
    F_roll_Y = c_roll_Y * mass * g;  
    
    %change sign of rolling friction force (needs to be opposite velocity)
    F_roll_X = -F_roll_X;
    F_roll_Y = -F_roll_Y;
    
    %Calculate aero drag force
    F_drag_X = .5*rho*x_velocity(i-1)^2*c_drag*area;
    F_drag_Y = .5*rho*y_velocity(i-1)^2*c_drag*area;
    
    %determine whether aero drag forces will need to be added or subtracted
    %(always opposite velocity)
    if x_velocity(i-1)>0
        F_drag_X=-F_drag_X;
        F_brake_X=-F_brake_X;
        
    end
    if y_velocity(i-1)>0
        F_drag_Y=-F_drag_Y;
        F_brake_Y=-F_brake_Y;
    end
    
    %Calculate total force
    Force_total_X = F_motor_X + F_drag_X + F_roll_X + F_brake_X;
    Force_total_Y = F_motor_Y + F_drag_Y + F_roll_Y + F_brake_Y;
    
    x_accel(i) = Force_total_X / mass;
    y_accel(i) = Force_total_Y / mass;
    
    x_velocity(i) = x_velocity(i-1) + x_accel(i) * timestep;
    y_velocity(i) = y_velocity(i-1) + y_accel(i) * timestep;
    
    x_pos(i) = x_pos(i-1) + x_velocity(i) * timestep;
    y_pos(i) = y_pos(i-1) + y_velocity(i) * timestep;
    
    Total_dist_traveled = Total_dist_traveled + sqrt((x_pos(i)-x_pos(i-1))^2+(y_pos(i)-y_pos(i-1))^2);
end

%trim excess unused zeros off of vectors
t=t(1:i-1);
x_pos = x_pos(1:i-1);
y_pos = y_pos(1:i-1);
x_velocity = x_velocity(1:i-1);
y_velocity = y_velocity(1:i-1);
v_goal = v_goal(1:i-1);
x_accel = x_accel(1:i-1);
y_accel = y_accel(1:i-1);




r_pos = sqrt(x_pos.^2 + y_pos.^2);
r_velocity = sqrt(x_velocity.^2 + y_velocity.^2);
r_accel = sqrt(x_accel.^2 + y_accel.^2);
%velocity = linspace(0,99);     % speed in m/s
%c_roll = 0.2+0.2.*(velocity./60.67);   % rolling fric
%F_roll = c_roll * mass * g;



%F_drag = .5.*rho.*velocity.^2.*c_drag.*area;

%F_total = F_roll + F_drag;

%Power = (F_total .* velocity)./1000;

% if(plot_mode==1)
%     set(0,'DefaultAxesFontName','Iskoola Pota')
% 
%     hold on
%     plot(velocity,Power,'color',([.2 .2 .6]),'linewidth',2)
%     plot(26.82*ones(1,100),linspace(0,500),'color',([.2 .6 .2]),'linewidth',2)
%     plot(53.6*ones(1,100),linspace(0,500),'color',([.6 .2 .2]),'linewidth',2)
%     hold off
%     grid on
%     legend('Power Required to Maintain Speed','60mph','120mph','Location','SouthOutside')
%     xlim([0 75]);
%     ylim([0 75]);
%     title('Power Dissipated versus Velocity')
%     ylabel('Power - KW')
%     xlabel('Velocity - m/s')
% end

if(plot_mode==2)
    figure
    set(0,'DefaultAxesFontName','Iskoola Pota')

    hold on
    plot(r_pos,r_velocity,'color',([.2 .2 .6]),'linewidth',2)
    plot(r_pos,r_velocity*2.23694,'color','r','linewidth',2)
    
   % axis([0,100,-inf,inf]); 
    hold off
    grid on
    ylabel('Velocity')
    xlabel('Position - m')
    legend('m/s','mph')
    title('r-velocity vs r-position')
    %----------------------
    figure
    set(0,'DefaultAxesFontName','Iskoola Pota1')

    hold on
    plot(t,r_velocity,'color',([.2 .2 .6]),'linewidth',2)
    plot(t,r_velocity*2.23694,'color','r','linewidth',2)
   % plot(t,v_goal,'linewidth',2)
    

    hold off
    grid on
    ylabel('Velocity')
    xlabel('Time - s')
   % legend('m/s','mph','v-goal(m/s)')
    title('r-velocity vs time')
    %----------------------
    figure
    set(0,'DefaultAxesFontName','Iskoola Pota1')

    hold on
    plot(t,r_accel,'color',([.2 .2 .6]),'linewidth',2)
   
    

    hold off
    grid on
    ylabel('Accel - m/s^2')
    xlabel('Time - s')
    title('r-accel vs time')
   %-----------------------------------
   
    figure
    set(0,'DefaultAxesFontName','Iskoola Pota')

    hold on
    plot(course_x,course_y,'color',([.2 .2 .6]),'linewidth',2)
    plot(points(:,1), points(:,2), 'g.')
    plot(x_pos, y_pos, 'r.')
    
    turnflag=0;
    [turnflag]= getCurvature(course_interp_X, course_interp_Y);
    plot(course_interp_X(turnflag),course_interp_Y(turnflag),'k*')
    
    
    
    %axis([-2000,2000,-2000,2000]); 
    axis equal
    hold off
    grid on
    ylabel('y')
    xlabel('x')
    title('bikes actual path overlayed on course')
    
    
    %-----------------------------------
   
   
   
end

if(plot_mode==3)
   
   
    figure
    set(0,'DefaultAxesFontName','Iskoola Pota')

    hold on
    
    z=zeros(length(course_x))';
    plot3(course_x,course_y,z,'color',([.2 .2 .6]),'linewidth',2)
    z=zeros(length(x_pos))';
    plot3(x_pos, y_pos,z, 'r.')
    plot3(x_pos, y_pos, r_velocity,'c.')
    hold off
    grid on
    zlabel('Velocity - m/s')
    ylabel('y - m')
    xlabel('x - m')
    title('Bike Velocity on Track')
    
    %---------------------------------
    
    
    figure
    set(0,'DefaultAxesFontName','Iskoola Pota')

    hold on
    
    z=zeros(length(course_x))';
    plot3(course_x,course_y,z,'color',([.2 .2 .6]),'linewidth',2)
    z=zeros(length(x_pos))';
    %plot3(x_pos, y_pos,z, 'r.')
    plot3(x_pos, y_pos, v_goal,'c.')
    hold off
    grid on
    zlabel('Velocity - m/s')
    ylabel('y - m')
    xlabel('x - m')
    title('Goal Velocity on Track')
    
end


