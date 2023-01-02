clear all;

syms t1 t2 t3 t4 t5 t6
t = [t1, t2, t3, t4, t5, t6];
% DH = [alpha, a, d, theta]
DH = [0, 0, 117.1, t1; pi/2, 0, 0, t2; 0, 172.6, 0, t3; pi/2, 0, 108.7, t4; -pi/2, 0, 0, t5; pi/2, 0, 74.4, t6];
intermediateTransform={0,0,0,0,0,0};
% loop through each matrix and multiply them together
T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

for j=1:6
    % create each intermediate transform
    intTransform = [cos(DH(j,4)), -sin(DH(j,4)), 0, DH(j,2); sin(DH(j,4))*cos(DH(j,1)), cos(DH(j,4))*cos(DH(j,1)), -sin(DH(j,1)), -sin(DH(j,1))*DH(j,3); sin(DH(j,4))*sin(DH(j,1)), cos(DH(j,4))*sin(DH(j,1)), cos(DH(j,1)), cos(DH(j,1))*DH(j,3); 0, 0, 0, 1];
    % save this intermediate transform into an array to save for future use
    intermediateTransform{1,j}=intTransform;
    T = T*intTransform;
end

px = T(1,4);
py = T(2,4);
pz = T(3,4);

Jacob_half1 = {0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0};
%Jacob_half1 = [simplify(diff(px,t1)), simplify(diff(px,t2)), simplify(diff(px,t3)), simplify(diff(px,t4)), simplify(diff(px,t5)), simplify(diff(px,t6)); simplify(diff(py,t1)), simplify(diff(py,t2)), simplify(diff(py,t3)), simplify(diff(py,t4)), simplify(diff(py,t5)), simplify(diff(py,t6)); simplify(diff(pz,t1)), simplify(diff(pz,t2)), simplify(diff(pz,t3)), simplify(diff(pz,t4)), simplify(diff(pz,t5)), simplify(diff(pz,t6))]

% Second half
syms dt1 dt2 dt3 dt4 dt5 dt6 real
dt = [dt1, dt2, dt3, dt4, dt5, dt6];
w = {0,0,0,0,0,0}; % create cell table --> first row is nwn, second row is 0wn

% % Loop for all values of i
% for i=2:7
%     % Equation (5.45) from textbook
% %     R = [intermediateTransform{1,i-1}(1,1), intermediateTransform{1,i-1}(1,2), intermediateTransform{1,i-1}(1,3); intermediateTransform{1,i-1}(2,1), intermediateTransform{1,i-1}(2,2), intermediateTransform{1,i-1}(2,3); intermediateTransform{1,i-1}(3,1), intermediateTransform{1,i-1}(3,2), intermediateTransform{1,i-1}(3,3)];
%     % transpose R
%     RT = R.';
%     w{1,i} = RT*w(i-1)+[0, 0, dt(i-1)].';
%     %We have w(6 of 6), we want w(0 to 6) and we have R(0 to 6), and
%     %multiply R(0 to 6) by w(6 of 6) to get [wx; wy; wz]
%     % Multiply each nwn by Rotation matrix to get 0wn
%     w{2,i} = R*w{1,i};
%     
% end

rotationToZero = eye(3);
N = 6; % 6 joints
w = cell(N+1,2); % Store omega (w) 6x1 matrices in w cell
% in w, first column represents w in respect to itself, second column
% represents w in respect to {0} frame
w{1,1} = [0;0;0]; % Initialize 0w0 as 0
w{1,2} = [0;0;0]; % 0w0 is 0
v = cell(N+1,2);
v{1,1} = [0;0;0];
v{1,2} = [0;0;0];
P = [[0;0;0],[0;0;117.1],[172.6;0;0],[0;66;0],[0;0;42.7],[0;74.4;0]]

%Jacobian 2nd half
Jacob_half2 = {0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0};

for k=1:N
    transformToOneLess = [cos(DH(k,4)), -sin(DH(k,4)), 0, DH(k,2); sin(DH(k,4))*cos(DH(k,1)), cos(DH(k,4))*cos(DH(k,1)), -sin(DH(k,1)), -sin(DH(k,1))*DH(k,3); sin(DH(k,4))*sin(DH(k,1)), cos(DH(k,4))*sin(DH(k,1)), cos(DH(k,1)), cos(DH(k,1))*DH(k,3); 0, 0, 0, 1];
    % get rotation matrix from One Less
    rotationOneLess = transformToOneLess(1:3,1:3);
    rotationToZero = rotationToZero*rotationOneLess; % each 0 to N matrix
    % transpose rotationOneLess to switch reference frames
    reverseRotationOneLess = rotationOneLess.';
    
    thetaDot = dt(k);
    % Calculate omega (w) with respect to itself (NwN)
    w{k+1,1} = reverseRotationOneLess*w{k,1} + [0;0;thetaDot];
    % Calculate omega (w) with respect to {0} frame (0wN)
    w{k+1,2} = rotationToZero*w{k+1,1};
    % So now we have 0w1, 0w2, 0w3, 0w4, 0w5, and 0w6! :)
    % Calculate linear velocity (v) with respect to itself (NvN)
    pValue = P(:,k);
    v{k+1,1} = reverseRotationOneLess*(v{k,1} + cross(w{k,1},pValue));
    v{k+1,2} = rotationToZero*v{k+1,1};
end

% 0w6 is w{7,2}
wx = w{7,2}(1);
wy = w{7,2}(2);
wz = w{7,2}(3);

% % 0v6 is v{7,2}
% vx = v{7,2}(1);
% vy = v{7,2}(2);
% vz = v{7,2}(3);

% Construct bottom matrix
for c=1:N
    Jacob_half1{1,c} = simplify(diff(px,t(c)));
    Jacob_half1{2,c} = simplify(diff(py,t(c)));
    Jacob_half1{3,c} = simplify(diff(pz,t(c)));
    Jacob_half2{1,c} = simplify(diff(wx,dt(c)));
    Jacob_half2{2,c} = simplify(diff(wy,dt(c)));
    Jacob_half2{3,c} = simplify(diff(wz,dt(c)));
end

jacobian = [Jacob_half1; Jacob_half2];
jmatrix = cell2sym(jacobian)
d = det(jmatrix)