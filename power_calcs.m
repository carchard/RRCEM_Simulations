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
clear all;
close all;

% Plotting Parameters
% -------------------------------------------------------------------
plot_mode = 2;

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

% The course
% -------------------------------------------------------------------

%SQUARE COURSE
course_x=[1000,-1000,-1000,+1000,1000]; 
course_y=[1000,1000,-1000,-1000,0];
%course_x=-1000:1000;
%course_y=sqrt(1000^2-course_x.^2);


%interpolation of the course data. For explantation:
    %http://blogs.mathworks.com/steve/2012/07/06/walking-along-a-path/)
    %this code will interpolate a specificed (num_points) number of points
    %to interpolate over.  The points will be equally spaced in terms of
    %PATH distance
xy=[course_x' course_y'];
d = diff(xy,1);
dist_from_vertex_to_vertex = hypot(d(:,1), d(:,2));
cumulative_dist_along_path = [0;cumsum(dist_from_vertex_to_vertex,1)];
num_points = 100; %set the number of interpolation points
dist_steps = linspace(0, cumulative_dist_along_path(end), num_points);
points = interp1(cumulative_dist_along_path, xy, dist_steps);
course_interp_X=points(:,1);
course_interp_Y=points(:,2);
%course_x = linspace(0,1006.97,1113)';
%course_y=5+sqrt(16-4*(course_x+6).^2);


%vectorLength=num_points;
vectorLength=2000;

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

for i = 2:vectorLength
    
    %update time vector
    t(i)=t(i-1)+timestep; 
    
    %determine theta (this is angle of current position to desired position)
   if Total_dist_traveled>=dist_between_steps*j
    while Total_dist_traveled>=dist_between_steps*j
        j=j+1;
    end
     theta=atan2d((course_interp_Y(j+1)-y_pos(i-1)),(course_interp_X(j+1)-x_pos(i-1)));
   end
   
    %[v_goal_x, v_goal_y] = getGoalVelocity(); 
    v_goal_x=100000; v_goal_y=100000;
    
    %determine whether bike has reached its goal velocity
    if (v_goal_x>x_velocity(i) && v_goal_y>y_velocity(i))
        % engine is on code here
        F_motor = 500; %random value for code testing
       
        F_motor_X = F_motor * cosd(theta);
        F_motor_Y = F_motor * sind(theta);
        
        %need to break into components and apply F_motor_X and F_motor Y_
        %to keep direction on path
    else
       % Braking code here
       F_motor=0; %random value for code testing
    end
    
    %theta = arctan(course_y(i)/course_x(i));
    %r_velocity(i) = sqrt(x_velocity(i).^2 + y_velocity(i).^2); %calculates magnitude of velocity vector ("true" velocity)
    
    %Calculate rolling friction force
    %c_roll_X = 0.2+0.2*(x_velocity(i)/60.67);   % rolling fric
    %c_roll_Y = 0.2+0.2*(y_velocity(i)/60.67);   % rolling fric
    c_roll_X = 0.2*(x_velocity(i-1)/60.67);   % rolling fric
    c_roll_Y = 0.2*(y_velocity(i-1)/60.67);   % rolling fric
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
    end
    if y_velocity(i-1)>0
        F_drag_Y=-F_drag_Y;
    end
    
    %Calculate total force
    Force_total_X = F_motor_X + F_drag_X + F_roll_X;
    Force_total_Y = F_motor_Y + F_drag_Y + F_roll_Y;
    
    x_accel(i) = Force_total_X / mass;
    y_accel(i) = Force_total_Y / mass;
    
    x_velocity(i) = x_velocity(i-1) + x_accel(i) * timestep;
    y_velocity(i) = y_velocity(i-1) + y_accel(i) * timestep;
    
    x_pos(i) = x_pos(i-1) + x_velocity(i) * timestep;
    y_pos(i) = y_pos(i-1) + y_velocity(i) * timestep;
    
    Total_dist_traveled = Total_dist_traveled + sqrt((x_pos(i)-x_pos(i-1))^2+(y_pos(i)-y_pos(i-1))^2);
end

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
    %----------------------
    figure
    set(0,'DefaultAxesFontName','Iskoola Pota1')

    hold on
    plot(t,r_velocity,'color',([.2 .2 .6]),'linewidth',2)
    plot(t,r_velocity*2.23694,'color','r','linewidth',2)
    

    hold off
    grid on
    ylabel('Velocity')
    xlabel('Time - s')
     legend('m/s','mph')
     
     figure
    set(0,'DefaultAxesFontName','Iskoola Pota1')

    hold on
    plot(t,r_accel,'color',([.2 .2 .6]),'linewidth',2)
   
    

    hold off
    grid on
    ylabel('Accel - m/s^2')
    xlabel('Time - s')
   %-----------------------------------
   
    figure
    set(0,'DefaultAxesFontName','Iskoola Pota')

    hold on
    plot(course_x,course_y,'color',([.2 .2 .6]),'linewidth',2)
    plot(points(:,1), points(:,2), 'g.')
    plot(x_pos, y_pos, 'r.')
    axis([-2000,2000,-2000,2000]); 
    hold off
    grid on
    ylabel('y')
    xlabel('x')
    
    %-----------------------------------
   
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
   
end

if(plot_mode==3)
   
   
   
   
end