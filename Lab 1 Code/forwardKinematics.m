function transformationMatrix = forwardKinematics(theta1,theta2,theta3,theta4,theta5,theta6)
% Returns the corresponding transformation matrix from the angle positions
    a = [0, 0, 172.6, 0, 0, 0];
    d = [117.1, 0, 0, 108.7, 0, 74.4];
    alpha = [0, 90,0, 90, -90, 90];
    % thetas are variables, so they are taken from the parameters
    theta = [theta1, theta2, theta3, theta4, theta5, theta6];

    % calculate initial transform for the first link
    initialLinks = concatenateLinks(a(1),d(1),alpha(1),theta(1));
    % loop through the rest of the links and compute transform
    for i = 2:6
        initialLinks = initialLinks * concatenateLinks(a(i),d(i),alpha(i),theta(i));
    end
    % return transformationMatrix from {0} to {6}
    transformationMatrix = initialLinks;
end