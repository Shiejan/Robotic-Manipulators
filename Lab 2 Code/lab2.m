close all;
clear;

% Defining the Model Robot (link lengths may be different) 
Link1 = Link('a',0,'d',117.1,'alpha',pi/2); 
Link2 = Link('a',172.6,'d',0,'alpha',0); 
Link3 = Link('a',0,'d',0,'alpha',pi/2); 
Link4 = Link('a',0,'d',108.7,'alpha',-pi/2); 
Link5 = Link('a',0,'d',0,'alpha',pi/2);
Link6 = Link('a',0,'d',74.4,'alpha',0);

ModelBot = SerialLink([Link1, Link2, Link3, Link4, Link5, Link6], 'name','MSE');

% % Joint limits are defined (in terms of actual robot angles in degrees)
Q_UpperLimits = [300, 271, 260, 360, 257.8, 360, 208.5];
Q_LowerLimits = [0, 28.2, 28, 0, 70.4, 0, 156.6];

% Points through path
Q_via = [ ... 
    145.2, 263.6, 39, 66, 244, 50, 178; ... % point #1
    145.2, 263.6, 75, 66, 75, 50, 178; ... % point #2
    145.2, 263.6, 75, 66, 75, 50, 200; ... % point #3
    145.2, 137, 75, 66, 75, 50, 200; ... % point #4
    145.2, 65, 147, 66, 166, 50, 200; ... % point #5
    145.2, 65, 147, 66, 166, 50, 178; ... % point #6
    145.2, 65, 147, 66, 166, 50, 178]; % point #7
len = length(Q_via);

%% Calculate function
% Loop through each configuration and find unknown variables
t = [0, 2, 4, 6, 8, 10, 12]; % time
totalTrajectory = []; % holds the total trajectory of each point for each interval

%% Equation solving
allACoefficients = [];
for outerNumber=1:7 % for each theta
    % holds trajectory, velocity, acceleration, and time for each joint
    trajectoryJoint = [];
    velocityJoint = [];
    accelerationJoint = [];
    timeJoint = [];
    
    for innerNumber=1:6 % for each position
        syms a0 a1 a2 a3 a4 a5
        
        % define initial conditions
        pi = Q_via(innerNumber,outerNumber);
        pf = Q_via(innerNumber+1,outerNumber);
        vi = 0;
        vf = 0;
        ai = 0;
        af = 0;
        ti = t(innerNumber);
        tf = t(innerNumber+1);
        
        % Set up equations
        eq1 = a0 + a1*ti + a2*ti^2 + a3*ti^3 + a4*ti^4 + a5*ti^5 == pi;
        eq2 = a0 + a1*tf + a2*tf^2 + a3*tf^3 + a4*tf^4 + a5*tf^5 == pf;
        eq3 = a1 + 2*a2*ti + 3*a3*ti^2 + 4*a4*ti^3 + 5*a5*ti^4 == vi;
        eq4 = a1 + 2*a2*tf + 3*a3*tf^2 + 4*a4*tf^3 + 5*a5*tf^4 == vf;
        eq5 = 2*a2 + 6*a3*ti + 12*a4*ti^2 + 20*a5*ti^3 == ai;
        eq6 = 2*a2 + 6*a3*tf + 12*a4*tf^2 + 20*a5*tf^3 == af;
        
        % Convert equations to matrix for solving
        [A,B] = equationsToMatrix([eq1, eq2, eq3, eq4, eq5, eq6], [a0, a1, a2, a3, a4, a5]);
        % Solve for coefficients
        X = linsolve(A,B);
        allACoefficients = [allACoefficients, X]; % all coefficients arranged in row
        
        % let's make it a row vector
        % sample for time
        sampleTime = ti:(tf-ti)/10:tf;
        timeJoint = [timeJoint,sampleTime];
        % calculate trajectory, velocity, acceleration
        trajectory = X(1) + X(2)*sampleTime + X(3)*sampleTime.^2 + X(4)*sampleTime.^3 + X(5)*sampleTime.^4 + X(6)*sampleTime.^5;
        velocity = X(2) + 2*X(3)*sampleTime + 3*X(4)*sampleTime.^2 + 4*X(5)*sampleTime.^3 + 5*X(6)*sampleTime.^4;
        acceleration = 2*X(3) + 6*X(4)*sampleTime + 12*X(5)*sampleTime.^2 + 20*X(6)*sampleTime.^3;
        % combine into the trajectory for all time intervals of the joint
        trajectoryJoint = [trajectoryJoint,trajectory];
        velocityJoint = [velocityJoint,velocity];
        accelerationJoint = [accelerationJoint,acceleration];
    end
    
    % Plot trajectory
    subplot(3,1,1);
    plot(timeJoint,trajectoryJoint);
    title('Position of Each Joint')
    xlabel('Time (sec)');
    ylabel('Position (deg)');
    hold on;
    totalTrajectory = [totalTrajectory;trajectoryJoint]; % totalTrajectory holds trajectory where each row represents the trajectory of each joint
    % Plot velocity
    subplot(3,1,2)
    plot(timeJoint,velocityJoint);
    title('Velocity of Each Joint')
    xlabel('Time (sec)');
    ylabel('Velocity (deg/sec)');
    hold on;
    % Plot acceleration
    subplot(3,1,3)
    plot(timeJoint,accelerationJoint);
    title('Acceleration of Each Joint')
    xlabel('Time (sec)');
    ylabel('Acceleration (deg/sec^2)');
    hold on;
end
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

coefficients = double(allACoefficients);

%% Back to code template
Q = totalTrajectory.'; % transpose totalTrajectory so each row represents a point

[points, ~] = size(Q); % Obtain number of points to send

Robot = MSE4401BOT(1234,4321); % Create robot object

pause on; % Enable use of pause command

% Prepare the robot to move, set to maximum torque and low speed
disp('Ready to move the robot. Press a key to continue...');
pause;
Robot.sendTorque([1700, 1700, 1700, 1700, 1700, 1700, 1700]);
Robot.sendSpeed([2, 2, 2, 2, 2, 2, 2]); % can be increased to up to 5 for each joint

%% Send points to robot
for i = 1:points
    Q_robot = Q(i,:); % Get the next point to be sent
    
    % Convert from model angles to actual robot angles
    % Should have a function like 'convertRobotToModel' from Lab 1
    % May have to modify function to accomodate 7th gripper
    %Q_model = convertRobotToModel(Q_robot);
    Q_model = Q_robot;
    
   % Check for being close to singularity using the model, only first 6 angles 
    J=ModelBot.jacob0(Q_model(1:6));
    DetJ = det(J);
    if abs(DetJ) < 0.0001
        Alarm_Singularity = 1;
        display('Singularity error');
    else
        Alarm_Singularity = 0;
    end
    
    % Check the joint limits
    Alarm_Limits = 0;
    for j = 1:7 % change later
        if Q_robot(j) > Q_UpperLimits(j) || Q_robot(j) < Q_LowerLimits(j) 
            Alarm_Limits = 1;
            display('Limit error');
        end
    end
    
    Alarm = Alarm_Limits + Alarm_Singularity; % Any Fault in the system
    
    % Send the point to the robot and wait for the robot to reach the point
    if Alarm == 0
        Robot.sendPosition(Q_robot);
        Q_current = transpose(Robot.getPosition); 
        while norm(Q_current - Q_robot,2) > 20
            Q_current = transpose(Robot.getPosition);
        end
    else
        disp('The position is out of reach! Press enter to continue');
        pause;
        % do something
    end
    pause(0.5)
end