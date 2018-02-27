%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize robots to a
%particular pose
%Paul Glotfelter 
%3/24/2016

% Modified by Sangjun Lee for fault detection research
% lee1424@purdue.edu
% 2/16/2018

% Short Summary
% 10 agents, randomly selected 3 agents being attacked, custom attack mag
% random initial pose to custom goal pose

%% Main Run for Robotarium
% Initialize
clear; close all; clc;
init

N = 10; % Number of agents
att_k = 1051; % 3a+1
att_mag = 0.6; % 0.6
att_agent = randperm(N,3); % three random agents of N agents


% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a specific value for this algorithm
%N = rb.get_available_agents(); % Random number

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
initial_conditions = ones(3,N)*0.9; % custom goal pose

%args = {'PositionError', 0.01, 'RotationError', 0.1}; % Hard error bound
args = {'PositionError', 0.3, 'RotationError', 0.7}; % Custom error bound
init_checker = create_is_initialized(args{:});
automatic_parker = create_automatic_parking_controller(args{:});

i = 1;
j = 1;
while(~init_checker(x, initial_conditions))

    x = r.get_poses(); % Get current pose
    dxu = automatic_parker(x, initial_conditions);
    dxu = unicycle_barrier_certificate(dxu, x);      

    r.set_velocities(1:N, dxu);
    r.step();
    
    % Save the current pose and input for data plotting
    data.pose([i i+1 i+2],:) = x; % [x y z]'
    i = i + 3;
    
    data.u([j j+1],:) = dxu; % [ux uy]'
    j = j + 2;
    
    % Attack injection
    if i == att_k
       initial_conditions(1:2,att_agent(1)) = [-att_mag  att_mag];
       initial_conditions(1:2,att_agent(2)) = [ att_mag -att_mag];
       initial_conditions(1:2,att_agent(3)) = [-att_mag -att_mag];
       
       data.att_agent = att_agent;
       data.att_k = att_k;
       data.att_mag = att_mag;
       data.N = N;
       data.goal = initial_conditions;
    end
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

%% Generate Figure
save rundata.mat data
[lim_row, lim_col] = size(data.pose);

hold on
for k = 1:N
    plot(data.pose(1:3:lim_row,k),data.pose(2:3:lim_row,k));
    text(data.pose(end-2,k)+0.03,data.pose(end-1,k)+0.03,...
        sprintf('%d',k),'FontSize',10)
end
hold off
