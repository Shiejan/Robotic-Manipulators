function [jacobianMatrix, determinant] = computeJacobian(DH,t,dt,N)
    % computeJacobian computes the Jacobian matrix for given parameters
    
    rotationToZero = eye(3);
    transformToZero = eye(4);
    w = cell(N+1,1);
    w{1} = [0;0;0];
    
    % Loop to get each transform
    for k=1:N
        % compute transform
        transformToOneLess = [cos(DH(k,4)), -sin(DH(k,4)), 0, DH(k,2); sin(DH(k,4))*cos(DH(k,1)), cos(DH(k,4))*cos(DH(k,1)), -sin(DH(k,1)), -sin(DH(k,1))*DH(k,3); sin(DH(k,4))*sin(DH(k,1)), cos(DH(k,4))*sin(DH(k,1)), cos(DH(k,1)), cos(DH(k,1))*DH(k,3); 0, 0, 0, 1];
        % get rotation matrix from transformToOneLess
        rotationOneLess = transformToOneLess(1:3,1:3);
        rotationToZero = rotationToZero*rotationOneLess; % each 0 to N matrix
        transformToZero = transformToZero*transformToOneLess;
        % transpose rotationOneLess to switch reference frames
        reverseRotationOneLess = rotationOneLess.';
    
        thetaDot = dt(k);
        % Calculate omega (w) with respect to itself (NwN)
        w{k+1,1} = reverseRotationOneLess*w{k,1} + [0;0;thetaDot];
    end
    
    % transformToZero holds 0T6
    px = transformToZero(1,4);
    py = transformToZero(2,4);
    pz = transformToZero(3,4);
    
    % 0w6 is w{N+1,1}
    angularToZero = rotationToZero*w{N+1,1};
    wx = angularToZero(1);
    wy = angularToZero(2);
    wz = angularToZero(3);
    
    % Create jacobian cell
    jacobian = cell(N,N);
    
    % Construct bottom matrix
    for c=1:N
        jacobian{1,c} = simplify(diff(px,t(c)));
        jacobian{2,c} = simplify(diff(py,t(c)));
        jacobian{3,c} = simplify(diff(pz,t(c)));
        jacobian{4,c} = simplify(diff(wx,dt(c)));
        jacobian{5,c} = simplify(diff(wy,dt(c)));
        jacobian{6,c} = simplify(diff(wz,dt(c)));
    end

    % Show determinant and matrix
    jmatrix = cell2sym(jacobian);
    jacobianMatrix = simplify(jmatrix);
    d = simplify(det(jmatrix));
    determinant = d;
    
%     [t2, t3, t5] = solve(determinant == 0, [t(2), t(3), t(5)]);
%     singularities = [simplify(t2), simplify(t3), simplify(t5)];
end