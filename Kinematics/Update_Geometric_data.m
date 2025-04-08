% Script to update geometric and inertia data for a 7-DOF robot
clc; close all; clear;

nL = 7; % Number of links

% Center of mass data in global frame [Link#, mass, x, y, z]
Data_cgl = [1, 5.32, 0.0244, 0.0110, 0.2236;
            2, 4.51, 0.1078, 0.1425, 0.3201;
            3, 1.75, 0.03568, 0.1775, 0.3172;
            4, 2.51, 0.5091, 0.0663, 0.3218;
            5, 1.12, 0.7301, 0.0309, 0.3189;
            6, 1.56, 0.9047, 0.1314, 0.3109;
            7, 0.33, 0.9860, 0.1517, 0.3170];

% Inertia tensors in body frame [Link#, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
Inertia_cg = [1, 53.31, 4.71, 11.73, 57.90, 8.02, 23.66;
              2, 22.40, -0.24, -0.29, 14.61, -6.09, 17.30;
              3, 25.51, 0.00, 0.012, 5.30, -3.32, 3.42;
              4, 10.16, -0.01, 0.27, 6.57, 3.03, 6.91;
              5, 13.56, 0.02, 0.141, 3.56, 1.06, 1.37;
              6, 4.73, 0.12, 0.052, 0.97, -1.16, 3.18;
              7, 0.31, 0.00, 0.00, 0.22, -0.01, 0.36];

% Rotation matrices from standard to non-standard frame for each link
A_T_B{1} = eye(3);
A_T_B{2} = [0, -1, 0; 1, 0, 0; 0, 0, 1]';
A_T_B{3} = eye(3);
A_T_B{4} = eye(3);
A_T_B{5} = [-1, 0, 0; 0, -1, 0; 0, 0, 1]';
A_T_B{6} = [-1, 0, 0; 0, -1, 0; 0, 0, 1]';
A_T_B{7} = eye(3);

% Compute inertia tensors in non-standard frame
for i = 1:nL
    Iv = Inertia_cg(i, 2:end); % Inertia components
    A_IC{i} = [Iv(1), -Iv(2), -Iv(3); -Iv(2), Iv(4), -Iv(5); -Iv(3), -Iv(5), Iv(6)]; % Inertia tensor
    m(i) = Data_cgl(i, 2); % Mass of each link
end

% DH Parameters
d = [317*cos(asin(81/317)), 194.5, 400, 168.5, 400, 136.3, 134.75]' * 1e-3; % Offsets
a = [0, 81, 0, 0, 0, 0, 0]' * 1e-3; % Lengths
alpha = [0, -pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2]'; % Twist angles
q = zeros(7,1); % Initial joint angles
teta = q; teta(2) = teta(2) - pi/2; % Adjust second joint

% Compute transformation matrices
Ti_im = cell(nL, 1); Ti = cell(nL, 1);
for i = 1:nL
    Ti_im{i} = [cos(teta(i)), -sin(teta(i)), 0, a(i);
                sin(teta(i))*cos(alpha(i)), cos(teta(i))*cos(alpha(i)), -sin(alpha(i)), -sin(alpha(i))*d(i);
                sin(teta(i))*sin(alpha(i)), cos(teta(i))*sin(alpha(i)), cos(alpha(i)), cos(alpha(i))*d(i);
                0, 0, 0, 1];
end
for i = 1:nL
    if i == 1
        Ti{i} = Ti_im{i};
    else
        Ti{i} = Ti{i-1} * Ti_im{i};
    end
end

% Update center of mass in non-standard frame
for i = 1:nL
    Data_cgl_Li = Data_cgl(i, 3:end)'; % Center of mass in global frame
    pf_Li = Ti{i}(1:3, 4); % Frame origin in global frame
    i_R_0 = Ti{i}(1:3, 1:3)'; % Rotation from global to link frame
    i_PC(i, :) = (i_R_0 * (Data_cgl_Li - pf_Li))'; % Center of mass in link frame
end

% Update inertia in non-standard frame
for i = 1:nL
    B_ICi = (A_T_B{i}') * A_IC{i} * A_T_B{i}; % Transform inertia tensor
    Table_I(i, 1) = B_ICi(1,1); % Ixx
    Table_I(i, 2) = -B_ICi(1,2); % Ixy
    Table_I(i, 3) = -B_ICi(1,3); % Ixz
    Table_I(i, 4) = B_ICi(2,2); % Iyy
    Table_I(i, 5) = -B_ICi(2,3); % Iyz
    Table_I(i, 6) = B_ICi(3,3); % Izz
end

% Display results
disp('Center of Mass in Non-Standard Frame:'); disp(i_PC);
disp('Inertia Tensor in Non-Standard Frame:'); disp(Table_I);

% Update and save data
Data_cgl(:, 3:5) = i_PC;
Inertia_cg(:, 2:end) = Table_I;
xlswrite('Data_cg', Data_cgl); % Save center of mass data
xlswrite('Data_inertia', Inertia_cg); % Save inertia data