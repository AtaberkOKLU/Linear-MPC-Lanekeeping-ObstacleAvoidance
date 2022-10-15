%% Prepare
clc
clear
close all

%% Constants
g = 9.81; % m/s^2
m = 2050; % kg
I = 3344; % kg-m^2
% mu = .3;  % Slipery
rho = Inf;  % m
W_line = 3; % m
x_dot_ref = 125/9; % m/s

% Geometry
a = 2;
b = 2;
c = 1.8;

C_fu = (4200-(-4200))/(4-(-4));   % ? + 
C_rL = (3800-(-3800))/(5-(-5)*8);   % ? +
C_ru = (2000-(-2000))/(2-(-2))*8;   % ?

%% Variables
T_sampling  = 0.1;
T_duration  = 20;
N_horizon   = 30;
R_rho       = .01;
x_0  = [0 0 0 0 0]';
umin = [-pi/3 1]';
umax = [pi/3 1]';
xmin = [];
xmax = [];
f_x = [
       0 0 0 1  0;      % e_y
       0 0 0 -1 0;
       0 0 0 0  1;      % delta
       0 0 0 0 -1;
       1/x_dot_ref a/x_dot_ref 0 0 -1;  % alpha_f,lim
       -1/x_dot_ref -a/x_dot_ref 0 0 1;
       1/x_dot_ref -b/x_dot_ref 0 0 0;  % alpha_r,lim
       -1/x_dot_ref b/x_dot_ref 0 0 0;
       ];
f_u = [];
f_u_eq = [
    0 1
    ];
v_x = [
       W_line/2; % e_y
       W_line/2; 
       pi/3;     % delta
       pi/3;
       pi/6;  % alpha_f_lim
       pi/6;  % alpha_f_lim
       pi/6;  % alpha_r_lim
       pi/6;  % alpha_r_lim
       ];
v_u = [];
v_u_eq = [
    1
    ];
obstacle1.duration  = [5 6];
obstacle1.width     = [1 -0.5];
obstacle2.duration  = [15 16];
obstacle2.width     = [.2 -0.5];
obstacle3.duration  = [10 12];
obstacle3.width     = [.2 -0.1];
Obstacles = [obstacle1 obstacle2 obstacle3];
[f_x_TV, v_x_TV, TV_x] = CreateObstacle(Obstacles, T_sampling);

showPlots = true;
Psi_r = 1/rho;

%% State Space Modeling
States = ["y_dot", "Phi_dot", "e_Phi", "e_y", "delta"];
A_c = [ (C_fu+C_ru)/(x_dot_ref*m) , (-x_dot_ref + (C_fu*a-b*C_ru)/(x_dot_ref*m)), 0, 0, -C_fu/m ;
      (a*C_fu-b*C_rL)/(I*x_dot_ref), (a*a*C_fu+b*b*C_rL)/(I*x_dot_ref), 0, 0, -a*C_fu/I  ;
      0, 1, 0, 0, 0; % Constant Term
      1, 0, x_dot_ref, 0, 0;
      0, 0, 0, 0, 0 ];
B_c = [0 0 0 0 1; 0 0 -x_dot_ref*Psi_r 0 0]';
C_c = [0 0 0 1 0];
D_c = [0 0];

%% Discritize
[A, B, C, D] = DiscritizeStateSpace(A_c, B_c, C_c, D_c, T_sampling);

[X, U, Y] = MPC_DualMode(A, B, C, D, N_horizon, T_sampling, T_duration, R_rho, x_0, umin, umax, xmin, xmax, f_x, f_u, f_u_eq, f_x_TV, v_x_TV, TV_x, v_x, v_u, v_u_eq, showPlots);


%% Plot Road
figure;
hold on
time_vec = 0:T_sampling:T_duration;
plot(time_vec, Y, "DisplayName", "Position")
plot(time_vec, (W_line/2)*ones(size(time_vec)), "r--", "DisplayName", "Boundry")
plot(time_vec, -(W_line/2)*ones(size(time_vec)), "r--", "HandleVisibility", "off")
plot(time_vec, zeros(size(time_vec)), "b--", "DisplayName", "Center Line")
plotObstacles(Obstacles);
legend
title("e_y(t) vs Time | N_{horizon}: "+N_horizon+", \psi_r: "+Psi_r+" m^{-1}, Speed_{referance}: "+x_dot_ref+" m/s, m_{car}: "+m+" kg, I_{car}:" +I+" kg-m^2")
xlabel("Time [sec]")
ylabel("e_y(t) [m]")



%% MPC_DualMode
% <include>MPC_DualMode.m</include>

%% DiscritizeStateSpace
% <include>DiscritizeStateSpace.m</include>

%% CreateObstacle
% <include>CreateObstacle.m</include>

%% plotObstacles
% <include>plotObstacles.m</include>