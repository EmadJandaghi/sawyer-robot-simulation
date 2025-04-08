function pos = Direct_Kinematic(Teta)
    % Computes the forward kinematics of a 7-DOF robot using DH parameters.
    % Input: Teta - Joint angles (rad) as a 7x1 vector
    % Output: pos - End-effector position [x, y, z] in meters

    nL = 7; % Number of links

    % Initial DH Parameters (in meters and radians)
    d0 = [317*cos(asin(81/317)), 194.5, 400, 168.5, 400, 136.3, 134.75]' * 1e-3; % Link offsets
    a0 = [0, 81, 0, 0, 0, 0, 0]' * 1e-3; % Link lengths
    alpha0 = [0, -pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2]'; % Twist angles
    Teta0 = zeros(7,1); % Initial joint angles (unused here)
    Teta(2) = Teta(2) - pi/2; % Adjust second joint angle for DH convention

    % Preallocate transformation matrices
    Ti_im = cell(nL, 1); % Intermediate transformations (relative to previous frame)
    Ti = cell(nL, 1);    % Transformations relative to base frame

    % Compute transformation matrices for each joint
    for i = 1:nL
        Ti_im{i} = [cos(Teta(i)), -sin(Teta(i)), 0, a0(i);
                    sin(Teta(i))*cos(alpha0(i)), cos(Teta(i))*cos(alpha0(i)), -sin(alpha0(i)), -sin(alpha0(i))*d0(i);
                    sin(Teta(i))*sin(alpha0(i)), cos(Teta(i))*sin(alpha0(i)), cos(alpha0(i)), cos(alpha0(i))*d0(i);
                    0, 0, 0, 1];
    end

    % Compute cumulative transformations from base frame
    for i = 1:nL
        if i == 1
            Ti{i} = Ti_im{i};
        else
            Ti{i} = Ti{i-1} * Ti_im{i};
        end
    end

    % Extract end-effector position
    loc = Ti{nL}(1:3, 4); % Position vector [x, y, z]

    % Extract orientation (Euler angles: alpha, beta, gamma)
    R = Ti{nL}(1:3, 1:3); % Rotation matrix
    bet = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2)); % Beta angle
    if bet == pi/2 % Degenerate case
        alph = 0;
        gam = atan2(R(1,2), R(2,2));
    elseif bet == -pi/2 % Degenerate case
        alph = 0;
        gam = -atan2(R(1,2), R(2,2));
    else
        alph = atan2(R(2,1)/cos(bet), R(1,1)/cos(bet)); % Alpha angle
        gam = atan2(R(3,2)/cos(bet), R(3,3)/cos(bet));  % Gamma angle
    end
    orientation = [alph, bet, gam]'; % Orientation vector

    % Output end-effector position (and optionally orientation)
    pos = loc; % Currently returns only position [x, y, z]
end