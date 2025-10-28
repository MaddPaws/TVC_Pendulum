%% Balancer Initialization Script
% ---------------------------------
% This script defines all the physical parameters, system dynamics,
% and controller gains for the 2D Thrust-Vectored Inverted Pendulum
% Simulink model.
%
% RUN THIS SCRIPT BEFORE RUNNING THE SIMULINK MODEL.
% ---------------------------------

clear;
clc;
close all;

%% 1. Physical Parameters
% Define the physical properties of the system.
% All units are in SI (kg, m, s, radians).

g = 9.81; % Acceleration due to gravity (m/s^2)

% --- Cylinder (The 2m pole)
m_cyl = 0.5;    % Mass of the cylinder (kg)
L_cyl = 2.0;    % Length of the cylinder (m)
r_cyl = 0.1;    % Radius of the cylinder (m)
L_cyl_cg = L_cyl / 2; % CG of cylinder, measured from pivot (m)

% --- Propulsion System (Motor, Battery, Servos, Gimbal, Electronics)
% We assume this is all a point mass at the pivot point (L=0).
m_prop = 1.5;   % ESTIMATED mass of propulsion system (kg)
L_prop_cg = 0.0;  % CG of propulsion system, measured from pivot (m)

%% 2. Calculated System Dynamics
% These values are calculated from the parameters above.
% The Simulink "Gain" blocks will use these variable names.

% --- Total System Properties
m_total = m_cyl + m_prop; % Total system mass (kg)
T_hover = m_total * g;      % Thrust required to hover (N)

% --- Total System Center of Gravity (L_cg)
% This is the 'L' in our equations.
L_cg = (m_cyl * L_cyl_cg + m_prop * L_prop_cg) / m_total;

% --- Total Moment of Inertia about the PIVOT (I_total)
% We use the Parallel Axis Theorem: I_pivot = I_cg + m*d^2
% I_cyl_cg (slender rod approx.) = (1/12) * m * L^2
I_cyl_cg = (1/12) * m_cyl * L_cyl^2;
% I_cyl about pivot
I_cyl_pivot = I_cyl_cg + m_cyl * (L_cyl_cg)^2;

% I_prop about pivot (assuming point mass at pivot)
I_prop_pivot = 0 + m_prop * (L_prop_cg)^2;

% I_total is the 'I' in our equations.
I_total = I_cyl_pivot + I_prop_pivot;

% --- Gains for Simulink Plant Model
% These are the exact variable names you should type into your
% Simulink "Gain" blocks.
mgL_I = (m_total * g * L_cg) / I_total;  % Gain for the unstable gravity loop
TL_I  = (T_hover * L_cg) / I_total;     % Gain for the control input loop
                                       % NOTE: We assume hover thrust (T=mg).
                                       % This means mgL_I and TL_I are equal,
                                       % giving the controller 1:1 authority.

%% 3. Controller Gains & Initial Conditions
% These are the tuning parameters for your controllers.
%
% !!! WARNING: THESE ARE STARTING GUESSES ONLY !!!
% You will need to TUNE these values to get a stable system.
%
% Strategy:
% 1. Set Kp_angle, P_rate, I_rate, D_rate to small values.
% 2. Tune the INNER loop (P_rate, I_rate, D_rate) first.
% 3. Once the inner loop is fast and stable, tune the OUTER loop (Kp_angle).

% --- Outer Loop (Angle P-Controller)
% Type 'Kp_angle' into the Gain block
Kp_angle = 1.0; % Proportional gain for angle error

% --- Inner Loop (Rate PID-Controller)
% Open the PID Controller block and enter these variable names
P_rate = 0.5;   % Proportional gain for rate error
I_rate = 0.1;   % Integral gain for rate error
D_rate = 0.2;   % Derivative gain for rate error

% --- Initial Conditions
% We start the simulation with a small tilt to see if it recovers.
%
% In Simulink:
% - Open the Integrator block for 'theta' (angle) and set "Initial condition" to 'theta_0'
% - Open the Integrator block for 'theta_dot' (rate) and set "Initial condition" to 'theta_dot_0'
%
theta_0 = 0.5;    % Initial tilt angle (radians, approx 5.7 deg)
theta_dot_0 = 0.1;% Initial angular rate (rad/s)

%% 4. Display Calculated Values
% Prints the key dynamics values to the command window.
fprintf('--- Balancer System Parameters ---\n');
fprintf('Total Mass (m_total):   %.2f kg\n', m_total);
fprintf('Total CG (L_cg):        %.2f m\n', L_cg);
fprintf('Total Inertia (I_total):  %.2f kg*m^2\n', I_total);
fprintf('Hover Thrust (T_hover):   %.2f N\n\n', T_hover);
fprintf('--- Simulink Plant Gains ---\n');
fprintf('Unstable Gain (mgL/I):  %.3f\n', mgL_I);
fprintf('Control Gain (TL/I):    %.3f\n\n', TL_I);
fprintf('Initialization Complete. Ready to run Simulink model.\n');
