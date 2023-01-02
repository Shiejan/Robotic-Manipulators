a = [0, 0, 172.6, 0, 0, 0];
d = [117.1, 0, 0, 108.7, 0, 74.4];
alpha = [0, 90,0, 90, -90, 90];
theta = [sym('theta1'), sym('theta2'), sym('theta3'), sym('theta4'), sym('theta5'), sym('theta6')];

initialLinks = concatenateLinks(a(1),d(1),alpha(1),theta(1));
for i = 2:6
    initialLinks = initialLinks * concatenateLinks(a(i),d(i),alpha(i),theta(i));
end
initialLinks