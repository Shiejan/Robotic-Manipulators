clear;

% Variables
time = [0, 4, 8, 12, 16, 20, 24, 28];
Q_via = [0.0, -0.04947088040930459, -0.002684466378799474, 0.7481991292911105, -0.0015339807878854137, -0.5483981316690354, -0.0019174759848567672, 1.2582477412630106, -0.0019174759848567672, -0.16260196351585385, 0.877820505867428, 0.4414029717140278, -0.451373846835283, 0.06442719309118737, 1.0937283017623, 0.09970875121255189, -12.565987119160338; ...
         0.0, -0.049854375606275945, 0.11236409271260656, 0.8674661355492015, -1.105616652868412, 0.036432043712278574, -0.07094661143970038, 0.006902913545484362, -1.6854613906890983, -0.15033011721277054, 1.0607477148227635, 1.0089758632316308, -0.054072822772960834, 0.07094661143970038, -0.1614514779249398, 1.6279371111433953, -12.565987119160338; ...
         0.0, -0.04947088040930459, 0.16988837225830958, -0.04908738521233324, -0.9081166264281649, -0.5963350312904546, -0.4007524808350643, 1.2183642407779898, -2.620422680905258, -0.14035924209151535, 0.3581845139712441, 0.8329515678217797, -0.5806117282146291, 0.4425534573049419, 0.5821457090025145, 2.4992381986623102, -12.565987119160338; ...
         0.0, -0.049854375606275945, 0.17832526659167935, -0.04908738521233324, 0.34284470609239, -1.2505778373235836, -0.603237944835939, 0.054072822772960834, -2.9575149590430776, -0.14304370847031483, 0.3508981052287884, -0.4686311306989939, -1.281257453081292, 0.66306319556347, -0.6557767868210144, 2.8585731982244686, -12.565987119160338; ...
         0.0, -0.049854375606275945, 0.25464081078897866, 0.3654709227136998, 0.15493205957642678, -0.010354370318226542, -1.1524030668989171, -0.1794757521825934, -3.058757691043515, -0.21130585353121575, 0.7861651537912745, -0.2626942099253771, 0.026077673394052033, 1.2459758949599273, -0.8824224482310843, 2.9786071948765023, -12.565987119160338; ...
         0.0, -0.04947088040930459, 0.4371845245473429, 0.604388430426853, -1.0519273252924224, -0.12233496783386175, -0.7907670961549308, -0.14150972768242942, 0.4801359866081345, -0.40382044241083515, 1.003606930474032, 0.9848156658224356, -0.09894176081860918, 0.8559612796400609, -0.7961360289125298, -0.7259564078667721, -12.565987119160338; ...
         0.0, -0.04947088040930459, 0.4479223900625408, 0.8601797268067457, -1.426985627930406, 0.5319078381992672, 1.4074273728848672, -0.553383569229663, 2.987427584406843, -0.4536748180171111, 1.190369091399081, 0.9756117810951231, 0.5276893910325823, -1.2425244381871852, -1.3058011456874585, -2.919932429739885, -12.565987119160338; ...
         0.0, -0.049854375606275945, 0.603237944835939, 0.41762626950180387, -1.1673593795808, -0.43219908698671533, 2.5974129690869767, -1.4492283493547446, 2.854738246254755, -0.6055389160177671, 0.7857816585943032, 0.6695826139119831, -0.457893265183796, -1.9167089944628244, -1.5366652542642132, -2.753879009451289, -12.565987119160338; ...
         ];
totalTrajectory = [];
totalAcceleration = [];

% Loop to find coefficients
for i=1:17 % all joints from robot
    % holds trajectory, velocity, acceleration, and time for each joint
    trajectoryJoint = [];
    velocityJoint = [];
    accelerationJoint = [];
    timeJoint = [];
    
    for j=1:7 % for each position
        % Setup symbols to be solved
        syms a0 a1 a2 a3 a4 a5
        
        % Initial conditions
        pi = Q_via(j,i);
        pf = Q_via(j+1,i);
        vi = 0;
        vf = 0;
        ai = 0;
        af = 0;
        ti = time(j);
        tf = time(j+1);
        
        % Setup equations to be solved
        eq1 = a0 + a1*ti + a2*ti^2 + a3*ti^3 + a4*ti^4 + a5*ti^5 == pi;
        eq2 = a0 + a1*tf + a2*tf^2 + a3*tf^3 + a4*tf^4 + a5*tf^5 == pf;
        eq3 = a1 + 2*a2*ti + 3*a3*ti^2 + 4*a4*ti^3 + 5*a5*ti^4 == vi;
        eq4 = a1 + 2*a2*tf + 3*a3*tf^2 + 4*a4*tf^3 + 5*a5*tf^4 == vf;
        eq5 = 2*a2 + 6*a3*ti + 12*a4*ti^2 + 20*a5*ti^3 == ai;
        eq6 = 2*a2 + 6*a3*tf + 12*a4*tf^2 + 20*a5*tf^3 == af;
        
        % Convert equations to matrix for solving
        [A,B] = equationsToMatrix([eq1, eq2, eq3, eq4, eq5, eq6], [a0, a1, a2, a3, a4, a5]);
        % Solve for coefficients
        X = linsolve(A,B); % X = [a0; a1; a2; a3; a4; a5]
        
        % Sample for time
        sampleTime = ti:(tf-ti)/10:tf;
        timeJoint = [timeJoint,sampleTime];
        % calculate trajectory, velocity, acceleration
        trajectory = X(1) + X(2)*sampleTime + X(3)*sampleTime.^2 + X(4)*sampleTime.^3 + X(5)*sampleTime.^4 + X(6)*sampleTime.^5;
        % Combine into the trajectory for all time intervals of the joint
        trajectoryJoint = [trajectoryJoint,trajectory];
        
        % Calculate acceleration
        acceleration = 2*X(3) + 6*X(4)*sampleTime + 12*X(5)*sampleTime.^2 + 20*X(6)*sampleTime.^3;
        accelerationJoint = [accelerationJoint, acceleration];
    end
    figure(i);
    subplot(2,1,1);
    plot(timeJoint,trajectoryJoint);
    title('Trajectory');
    totalTrajectory = [totalTrajectory; trajectoryJoint]; % each joint's trajectory is saved in a new row
    totalTrajectory = double(totalTrajectory);
    % Acceleration
    subplot(2,1,2);
    plot(timeJoint,accelerationJoint);
    title('Acceleration');
    totalAcceleration = [totalAcceleration; accelerationJoint];
    totalAcceleration = double(totalAcceleration);
end

% we have totalTrajectory --> arrange in proper form
trajectoryForm = zeros(length(timeJoint), 17);
trajectoryForm(:,1) = timeJoint;
trajectoryForm(:,2) = totalTrajectory(5,:);
trajectoryForm(:,3) = totalTrajectory(6,:);
trajectoryForm(:,4) = totalTrajectory(3,:);
trajectoryForm(:,5) = totalTrajectory(4,:);
trajectoryForm(:,6) = totalTrajectory(7,:);
trajectoryForm(:,7) = totalTrajectory(8,:);
trajectoryForm(:,8) = totalTrajectory(9,:);

% arrange accelerations
ddQ1 = [totalAcceleration(5,:).'];
ddQ2 = [totalAcceleration(6,:).'];
ddQ3 = [totalAcceleration(3,:).'];
ddQ4 = [totalAcceleration(4,:).'];
ddQ5 = [totalAcceleration(7,:).'];
ddQ6 = [totalAcceleration(8,:).'];
ddQ7 = [totalAcceleration(9,:).'];
ddTime = timeJoint.';
accelerationForm = [ddTime ddQ1 ddQ2 ddQ3 ddQ4 ddQ5 ddQ6 ddQ7];
accelerationForm(:,9) = zeros(length(timeJoint), 1);
accelerationForm(:,10) = zeros(length(timeJoint), 1);
accelerationForm(:,11) = zeros(length(timeJoint), 1);
accelerationForm(:,12) = zeros(length(timeJoint), 1);
accelerationForm(:,13) = zeros(length(timeJoint), 1);
accelerationForm(:,14) = zeros(length(timeJoint), 1);
accelerationForm(:,15) = zeros(length(timeJoint), 1);
accelerationForm(:,16) = zeros(length(timeJoint), 1);
accelerationForm(:,17) = zeros(length(timeJoint), 1);