%% Init goals
%  Sets up goal trajectories for controller to use.

%% Discrete goals
%  Get starting position from Baxter's current position
states = ones(16,1)*-1000;
bax_sub = rossubscriber('/robot/joint_states', rostype.sensor_msgs_JointState);
%  Get updates for all joints
msg = receive(bax_sub);
states = joint_states(msg,length(msg.Position),states);
while (min(states) < -500)
    % Get another message
    msg = receive(bax_sub);
    % Get states from it
	states = joint_states(msg);
end
initial_position = [0 states'];

%% Goals
%  Setup your goals and interpolate positions here
goals = trajectoryForm;
