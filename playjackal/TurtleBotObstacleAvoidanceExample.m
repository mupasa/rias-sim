%% Obstacle Avoidance Using TurtleBot
%% Introduction
% This example demonstrates an implementation of the VFH+ obstacle
% avoidance algorithm with the TurtleBot(R). The use of timers is explored
% to expose their power in autonomous algorithms. 
%
% The VFH+ algorithm is a simple, local method to help a robot
% navigate a space without hitting obstacles. Because the algorithm
% incorporates only local information, the robot is not guaranteed to reach the
% target point. It can get stuck in corners (local minimum). It can
% hit walls if the goal targeting gain is too large compared to that of 
% obstacle avoidance. It can move aimlessly if goal 
% targeting gain is too low compared to that of obstacle avoidance. 
% Experiment with the gains and parameters of the algorithm
% to investigate different types of behaviors for the robot.
% 
% This example shows you how to develop and test simple obstacle 
% avoidance on the TurtleBot. 
% Robotics System Toolbox(TM) contains a more powerful implementation of the VFH+
% obstacle avoidance algorithm in |<docid:robotics_ref.buv7g7y robotics.VectorFieldHistogram>|.
% The performance of the obstacle avoidance algorithm is subject 
% to the limitations of the Kinect(R) sensor, namely its minimum range and its
% limited field-of-view.
%
% Prerequisites: <docid:robotics_examples.example-TurtleBotCommunicationExample>,
% <docid:robotics_examples.example-TurtleBotBasicExplorationExample>, 
% <docid:robotics_examples.example-TurtleBotTeleoperationExample>

% Copyright 2014-2017 The MathWorks, Inc.

%% Hardware Support Package for TurtleBot
% This example gives an overview of working with a TurtleBot using its native ROS interface.
% The Robotics System Toolbox(TM) Support Package for TurtleBot(R)-Based Robots
% provides a more streamlined interface to TurtleBot. It allows you to:
%
% * Acquire sensor data and send control commands without explicitly calling ROS commands
% * Communicate transparently with a simulated robot in Gazebo or with a physical TurtleBot
%
% To install the support package, open *Add-Ons > Get Hardware Support
% Packages* on the MATLAB(R) *Home* tab and select "TurtleBot-Based Robots". 
% Alternatively, use the |<docid:robotics_ref.bu5jj8h roboticsAddons>| command.
%
%% Connect to the TurtleBot
% Make sure you have a TurtleBot running either in simulation through
% Gazebo(R) or on real hardware. Refer to 
% <docid:robotics_examples.example-GettingStartedWithGazeboExample> or
% <docid:robotics_examples.example-GettingStartedWithRealTurtleBotExample> for the startup 
% procedure.
% For this example, using the Gazebo(R) TurtleBot World provides
% the most interesting environment.
%
% * Initialize ROS. Connect to the TurtleBot by replacing 
% the sample IP address (192.168.1.1) with the IP address of the TurtleBot
%
%   ipaddress = '192.168.1.1'
%
%% 
%rosinit(ipaddress)
%%
% Make sure that you have started the Kinect camera if you are working with real 
% TurtleBot hardware. The command to start the camera is: 
% |roslaunch turtlebot_bringup 3dsensor.launch|. Run this command in a 
% terminal on the TurtleBot.


%% Initialize the Obstacle Avoidance Algorithm
% * Generate a struct that contains the gains used in the VFH+ algorithm. To
% change the behavior of the robot, change these gains before initializing
% the timer. The gains control four behaviors: target
% the goal, move in a straight line, move along a continuous path, and 
% avoid running into obstacles. Different gains cause different
% behaviors. For instance, if |obstaclesAvoid = 0|, the robot tries
% to plow through an obstacle regardless of distance. If |goalTargeting = 0|, the
% robot wanders aimlessly while avoiding obstacles. Selecting
% appropriate parameters is difficult and dependent on the robot's surroundings.  
% This example provides more information to help you select appropriate
% parameters. These gains attempt to balance 
% goal targeting with obstacle avoidance fairly.
%%
  gains.goalTargeting = 100;          % Gain for desire to reach goal
  gains.forwardPath = 0;            % Gain for moving forward 
  gains.continuousPath = 0;         % Gain for maintaining continuous path
  gains.obstacleAvoid = 5;        % Gain for avoiding obstacles
%%
% *Note:* The Kinect laser scan has a minimum range. Because
% of this minimum range, the TurtleBot can avoid some obstacles well and then turn and
% drive into them when they are very close. This behavior is because the laser does
% not see them. This movement can often happen in doorways, where the TurtleBot does
% not fully cross the threshold before turning toward the goal
% location. The frame of the door can be too close to see at this point,
% and the TurtleBot drives into it without knowing it is there. This issue is
% a drawback of local planning algorithms combined with the
% limited range of the laser scanner. With real hardware, the bump sensor
% must be activated in this case, but in simulation the bump sensor
% will not work, so the robot can get stuck against the door
% frame.
%
% * Create publishers and subscribers and make them part of a struct
% (|timerHandles|) which you pass into the timer when it is created. The
% publisher is for velocity and the subscribers are for the laser scanner,
% the odometry, and the bump sensor.
%%     
  timerHandles.pub = rospublisher('/jackal_velocity_controller/cmd_vel'); % Set up publisher
  timerHandles.pubmsg = rosmessage('geometry_msgs/Twist');

  timerHandles.sublaser = rossubscriber('/scan');  % Set up subscribers
  timerHandles.subodom = rossubscriber('/jackal_velocity_controller/odom');
  timerHandles.subbump = rossubscriber('/velodyne_points');
%%
% * If you want to reset the odometry before proceeding, you must
% subscribe to the |reset_odometry| topic and send an empty message to it:
% %%
%   odomresetpub = rospublisher('/mobile_base/commands/reset_odometry');  % Reset odometry 
%   odomresetmsg = rosmessage('std_msgs/Empty');
%   send(odomresetpub,odomresetmsg)
%   pause(2);     % Wait until odometry is reset
%%
% * Add the gains to the |timerHandles|
% struct:
%%
  timerHandles.gains = gains;

  
%% Test Obstacle Avoidance
% * Initialize the timer. The timer function takes a series of
% Name-Value pair arguments. The first pair is the callback function for
% the timer, which also includes the struct previously defined. The second
% defines the period of the timer (in this case it is 0.1 seconds per
% loop). The third and final Name-Value pair defines the mode of execution
% which is fixed spacing. You can also define a stop function for the
% timer, which in this case shuts down ROS when the timer is stopped.
%%
  timer1 = timer('TimerFcn',{@exampleHelperTurtleBotObstacleTimer,timerHandles},'Period',0.1,'ExecutionMode','fixedSpacing');
  timer1.StopFcn = {@exampleHelperTurtleBotStopCallback};
%%
% * Before starting the timer, you can visualize some steps of the algorithm
% at the command line. You can see the basic way the VFH+ algorithm works
% using the |exampleHelperTurtleBotShowGrid| function. 
% Three plots are displayed. Figure
% 1 shows the raw laser data after it has been sorted into a 2D histogram.
% Figure 2 shows the histogram after it has been smoothed to account for
% the robot width. Figure 3 shows the angular histogram, which is created
% by binning the obstacles into groups according to directions the robot
% can travel. The VFH+ algorithm uses these steps to determine how to
% avoid obstacles while targeting a goal point.
%%
  exampleHelperTurtleBotShowGrid(timerHandles.sublaser);
%%
% * In the Gazebo simulation, the histograms look something like
% the following:
%
% <<histograms.png>>
%
% The "Angle bin" plot is visually reversed from the two x-y grid
% plots. The grids represent the real 2D plane and the actual X and Y axes
% (though the coordinates do not correspond to the real world because they
% are representing bins). On the angle plot, the angle bins are listed from
% left to right in increasing order, but on a 2D plane these angles
% correspond to the space by moving from the right to the left, by
% convention.

%% Run the Robot
% * Enter the following commands to start the timer. The TurtleBot will
% begin to move and avoid any detected obstacles.
% The timer will be stopped automatically once the robot reaches its
% target.
%%
  start(timer1);
  while strcmp(timer1.Running, 'on')
    exampleHelperTurtleBotShowGrid(timerHandles.sublaser);
    pause(0.5);    
  end
%% 
% * To stop the timer while in the middle of a loop, close the
% figure window. If the timer does not clear, a safe way to clear all
% timers is to use the following command:
%%
  delete(timerfindall)
%%
% * At startup, the Gazebo simulation with the world plotting can look
% like this figure:
%
% <<obstacle_begin.png>>
%
% * After selecting a point in the next room to target, you can see
% something like this figure:
%
% <<obstacle_end.png>>

%%
displayEndOfDemoMessage(mfilename)

return

%% More Information
%
% *_NOTE: Code in this section is not for MATLAB command line execution_*
%
% In this example the code can be altered to allow you more freedom
% and exploration with the TurtleBot. The following is a description of features of
% the example along with suggestions for modification and alternative
% usage.
%
% This script uses <docid:matlab_ref.btwomz1-1 timers>, which you can use
% in many different ways. In this example
% you use the TimerFcn and StopFcn callbacks.
%
% The timer callback and the base workspace share information 
% through the |timerHandles| struct, which contains the gains,
% publishers, and subscribers. If you need additional
% information from the base workspace included in the timer callback,
% add it to the |timerHandles| struct.
%
% Once the timer has started, the timer callback runs according to the
% parameters. In this example the timer callback 
% (|exampleHelperTurtleBotObstacleTimer|)
% calls two primary functions that execute the main functions of the
% algorithm. Here is the basic structure (excluding declarations, 
% initializations, and callback functions):
%%
%  function exampleHelperTurtleBotObstacleTimer(mTimer, event, handles)
% 
%     % Declarations and Initializations would be here
%
%     % Determine current time
%     currentTime = datetime(event.Data.time);
%
%     % Execute the VFH+ algorithm to determine the desired angle trajectory
%     angleTarget = exampleHelperTurtleBotComputeTargetAngle(goal, data, pose, anglePrev, handles.gains);
%     
%     % Execute the main control loop        
%     [linV, angV] = turtlebotController(currentTime,angleTarget,bumper);
%     
%     % Set the velocities
%     handles.pubmsg.Linear.X = linV;
%     handles.pubmsg.Angular.Z = angV;
%     send(handles.pub,handles.pubmsg);
%     
%     % Set the previous angle
%     anglePrev = angleTarget;
%     
%     % Callback functions would be here

%%
% The first major function is |exampleHelperTurtleBotComputeTargetAngle|, which returns an angle the
% robot will target to turn toward (relative to its current orientation).
% In this example, computeTargetAngle is an implementation of the VFH+
% algorithm. The computeTargetAngle function can be replaced by any
% user-defined function that returns a target angle. You can explore many potential
% algorithms. The basic steps of the VFH+ algorithm are taken
% from a paper by Ulrich and Borenstein [1]. These are the crucial
% steps in the algorithm:
%
% <<vfh_algorithm.png>>
%
% If you want to adjust the bin sizes for the histograms and the
% maximum and minimum values for laser scan and angular histograms, refer
% to the local function in  |exampleHelperTurtleBotComputeTargetAngle| called setBin.
% You can adjust any of the parameters, including the x and y minimum and
% maximum and the step size. Be aware of the physical
% limitations of the robot (for instance, the y minimum must never be negative
% because the robot cannot see behind itself).
%
% The second primary method of the timer callback is the
% |turtlebotController|, which returns linear and angular velocities when
% given arguments such as goal position, current pose, and target angle.
% This controller can be replaced with any user-defined controller that returns
% linear and angular velocities (in m/s). Within the |turtlebotController|
% function is the |ExampleHelperPIDControl| class.
% As currently implemented, the
% turtlebotController uses only proportional control. However, the PID control
% class has options for proportional, derivative, and integral control,
% which can be used in the following manner:
%%
linGains.pgain = 0.2;
linGains.dgain = 0;
linGains.igain = 0;
linGains.maxwindup = 0;
linGains.setpoint = 0;
linPID = ExampleHelperPIDControl(linGains);
%%
% A faster way to set the gains is:
%%
linGains = struct('pgain',0.2,'dgain',0,'igain',0,'maxwindup',0','setpoint',0);
%%
% Be sure to test and tune the gains that you select, as they can result in
% very different robot behaviors. |setpoint| is the point that you want to
% control around. The other elements of the struct are clearly named.
% To update the controller and return the control value, use the update
% function.
%%
controlvalue = update(linPID,currentpoint);
%%
% The callback functions for subscribers are defined as nested
% functions in the timer callback function. Important variables such as
% |pose|, |goal|, and |bumper| are defined in those callbacks. Overall the
% modularity of the timer callback function and supporting functions
% allows for a great deal of flexibility in customization.


%% Next Steps
%
% * Refer to the final example: <docid:robotics_examples.example-TurtleBotObjectTrackingExample>


%% References
% [1] I. Ulrich, J. Borenstein, "VFH+: reliable obstacle avoidance for fast mobile robots," 
% In Proceedings of IEEE(R) International Conference on Robotics and Automation (ICRA), 1998, vol. 2, 
% pp. 1572-1577
