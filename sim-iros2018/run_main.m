%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize robots to a
%particular pose
%Paul Glotfelter 
%3/24/2016

% Modified by Sangjun Lee
% lee1424@purdue.edu
% 2/16/2018

%% Main Run for Robotarium
% Initialize
clear; close all; clc;
init

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a specific value for this algorithm
%N = rb.get_available_agents(); % Random number
N = 8; % Custom number

% Set the number of agents and whether we would like to save data.  Then, build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();

% Initialize x so that we don't run into problems later.  This isn't always necessary
x = r.get_poses(); % Initial pose matrix
r.step();

% Set some parameters for use with the barrier certificates.  We don't want our agents to collide

% Create a barrier certificate for use with the above parameters 
unicycle_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', 0.06, ... 
    'ProjectionDistance', 0.03);
        
% Get randomized initial conditions in the robotarium arena
%initial_conditions = generate_initial_conditions(N, 'Width',
%r.boundaries(2), 'Height', r.boundaries(4), 'Spacing', 0.2); % random goal pose
initial_conditions = ones(3,8)*1; % custom goal pose

%args = {'PositionError', 0.01, 'RotationError', 0.1}; % Hard error bound
args = {'PositionError', 0.3, 'RotationError', 0.5}; % Custom error bound
init_checker = create_is_initialized(args{:});
automatic_parker = create_automatic_parking_controller(args{:});

i = 1;
while(~init_checker(x, initial_conditions))

    x = r.get_poses(); % Get current pose
    dxu = automatic_parker(x, initial_conditions);
    dxu = unicycle_barrier_certificate(dxu, x);      

    r.set_velocities(1:N, dxu);
    r.step();
    
    % Save the current pose for data plotting
    traj([i i+1 i+2],:) = x;
    i = i + 3;
    
    % Attack injection
    if i == 1501 % 3a+1
       initial_conditions(randi(3),randi(8)) = -1;
    end
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

%% Generate Figure
% [lim_row, lim_col] = size(traj);
% 
% hold on;
% 
% for j = 1:N
%     plot(traj(1:3:lim_row,j),traj(2:3:lim_row,j));
% end
% 
% legend('limit','1','2','3','4','5','6','7','8');