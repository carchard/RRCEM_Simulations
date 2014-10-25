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
plot_mode = 1;

% Bike Parameters
% -------------------------------------------------------------------
weight = 226.8;     % 500lbs in kg
hub_rad = 0.4572;   % inner diameter of the tires
rolling_rad = 0.53;   % radius of the wheel
cross_section = 0.75;   % cross sectional area in m^2
%torque_curve = [];  % torque curve from a motor would go here indexed by RPM
%gear_ratio = 0; % a factor to convert engine RPM to velocity -need wheel rad
timestep = .1; % seconds

% Current Values
% -------------------------------------------------------------------
x_pos = 0;
y_pos = 0;
x_velocity = 0;
y_velocity = 0;
v_goal = 0;
x_accel = 0;
y_accel = 0;
power_consumed = 0;
Force_total_X = 0;    % Force totalled over the current frame
Force_total_Y = 0;

% The course
% -------------------------------------------------------------------
%course_x = [];
%course_y = [];

% The main loop
% -------------------------------------------------------------------
for i = 0:length(course_x)
    [v_goal_x, v_goal_y] = getGoalVelocity();
    if (v_goal_x<x_velocity & v_goal_y<y_velocity)
        % engine is on code here
    else
       % Braking code here 
    end
    Force_total_X = Force_total_X + F_drag_X + F_roll_X;
    Force_total_Y = Force_total_Y + F_drag_Y + F_roll_Y;
    
    x_accel = Force_total_X * weight;
    y_accel = Force_total_Y * weight;
    
    x_velocity = x_velocity + x_accel * timestep;
    y_velocity = y_velocity + y_accel * timestep;
    
    x_pos = x_pos + x_velocity * timestep;
    y_pos = y_pos + y_velocity * timestep;
end
velocity = linspace(0,99);     % speed in m/s
c_roll = 0.2+0.2.*(velocity./60.67);   % rolling fric
F_roll = c_roll * weight;

c_drag = .56; % from bike forum, speculated range is .55 to .65
area = .75; %1.05; % 2.5ft wide, 4.5ft tall rhombus
rho = 1.184; % mass density of air at 25C

F_drag = .5.*rho.*velocity.^2.*c_drag.*area;

F_total = F_roll + F_drag;

Power = (F_total .* velocity)./1000;

if(plot_mode)
    set(0,'DefaultAxesFontName','Iskoola Pota')

    hold on
    plot(velocity,Power,'color',([.2 .2 .6]),'linewidth',2)
    plot(26.82*ones(1,100),linspace(0,500),'color',([.2 .6 .2]),'linewidth',2)
    plot(53.6*ones(1,100),linspace(0,500),'color',([.6 .2 .2]),'linewidth',2)
    hold off
    grid on
    legend('Power Required to Maintain Speed','60mph','120mph','Location','SouthOutside')
    xlim([0 75]);
    ylim([0 75]);
    title('Power Dissipated versus Velocity')
    ylabel('Power - KW')
    xlabel('Velocity - m/s')
end