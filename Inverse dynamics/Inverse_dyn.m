function tau = Inverse_dyn(state)
    % Computes joint torques for a 7-DOF robot using Newton-Euler inverse dynamics.
    % Input: state - [q; qd; qdd] (21x1 vector: joint angles, velocities, accelerations)
    % Output: tau - Joint torques (7x1 vector in Nm)

    nL = 7; % Number of links/DOF
    g = 9.81; % Gravitational acceleration (m/s^2)

    % Extract joint angles, velocities, and accelerations from state
    q = state(1:nL);         % Joint angles (rad)
    qd = state(nL+1:2*nL);   % Joint velocities (rad/s)
    qdd = state(2*nL+1:3*nL); % Joint accelerations (rad/s^2)

    % Geometric and inertia data (updated from Part 1)
    Data_cgl = [1, 5.32, 0.0244, 0.011, -0.082876753;
                2, 4.51, 0.013623247, 0.0268, -0.052;
                3, 1.75, 0.010723247, 0.017, -0.44532;
                4, 2.51, 0.015323247, -0.0281, -0.0403;
                5, 1.12, 0.012423247, -0.0049, -0.1509;
                6, 1.56, 0.004423247, 0.0237, -0.0309;
                7, 0.33, 0.010523247, 0.0106, -0.02975];
    m = Data_cgl(:, 2); % Mass (kg)
    xc = Data_cgl(:, 3); % Center of mass x (m)
    yc = Data_cgl(:, 4); % Center of mass y (m)
    zc = Data_cgl(:, 5); % Center of mass z (m)

    % Inertia tensors (converted to kg·m^2)
    Inertia_cg = [1, 53.31, 4.71, 11.73, 57.9, 8.02, 23.66;
                  2, 14.61, 0.24, 6.09, 22.4, -0.29, 17.3;
                  3, 25.51, 0, 0.012, 5.3, -3.32, 3.42;
                  4, 10.16, -0.01, 0.27, 6.57, 3.03, 6.91;
                  5, 13.56, 0.02, -0.141, 3.56, -1.06, 1.37;
                  6, 4.73, 0.12, -0.052, 0.97, 1.16, 3.18;
                  7, 0.31, 0, 0, 0.22, -0.01, 0.36];
    Inertia_cg(:, 2:end) = Inertia_cg(:, 2:end) / 1000; % Convert from g·m^2 to kg·m^2
    Ixx = Inertia_cg(:, 2); Ixy = Inertia_cg(:, 3); Ixz = Inertia_cg(:, 4);
    Iyy = Inertia_cg(:, 5); Iyz = Inertia_cg(:, 6); Izz = Inertia_cg(:, 7);

    % DH Parameters
    teta = q; teta(2) = teta(2) - pi/2; % Adjust second joint angle
    d = [317*cos(asin(81/317)), 194.5, 400, 168.5, 400, 136.3, 134.75]' * 1e-3; % Offsets (m)
    a = [0, 81, 0, 0, 0, 0, 0]' * 1e-3; % Lengths (m)
    alpha = [0, -pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2]'; % Twist angles (rad)

    % Preallocate arrays
    IC = cell(nL, 1); PC = cell(nL, 1); Ti_im = cell(nL, 1); Ti = cell(nL, 1);
    w = cell(nL, 1); wd = cell(nL, 1); vd = cell(nL, 1); vd_C = cell(nL, 1);
    F = cell(nL, 1); N = cell(nL, 1);

    % Initialize inertia and center of mass
    for i = 1:nL
        IC{i} = [Ixx(i), -Ixy(i), -Ixz(i); -Ixy(i), Iyy(i), -Iyz(i); -Ixz(i), -Iyz(i), Izz(i)]; % Inertia tensor
        PC{i} = [xc(i); yc(i); zc(i)]; % Center of mass position
    end

    % Forward Kinematics (Transformation Matrices)
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

    % Outward Iterations (Compute velocities and forces)
    for i = 1:nL
        Ri = Ti_im{i}(1:3, 1:3); % Rotation matrix
        Pi = Ti_im{i}(1:3, 4);   % Position vector
        if i == 1
            w{i} = qd(i) * [0; 0; 1];          % Angular velocity
            wd{i} = qdd(i) * [0; 0; 1];        % Angular acceleration
            vd{i} = Ri' * [0; 0; g];           % Linear acceleration (gravity)
        else
            w{i} = Ri' * w{i-1} + qd(i) * [0; 0; 1]; % Angular velocity
            wd{i} = Ri' * wd{i-1} + cross(Ri' * w{i-1}, qd(i) * [0; 0; 1]) + qdd(i) * [0; 0; 1]; % Angular accel
            vd{i} = Ri' * (cross(wd{i-1}, Pi) + cross(w{i-1}, cross(w{i-1}, Pi)) + vd{i-1}); % Linear accel
        end
        vd_C{i} = cross(wd{i}, PC{i}) + cross(w{i}, cross(w{i}, PC{i})) + vd{i}; % CoM linear accel
        F{i} = m(i) * vd_C{i}; % Linear force
        N{i} = IC{i} * wd{i} + cross(w{i}, IC{i} * w{i}); % Torque about CoM
    end

    % Inward Iterations (Compute joint torques)
    for i = nL:-1:1
        if i == nL
            f{i} = F{i}; % Force at end-effector
            n{i} = N{i} + cross(PC{i}, F{i}); % Torque at end-effector
        else
            Rip = Ti_im{i+1}(1:3, 1:3); % Next frame rotation
            Pip = Ti_im{i+1}(1:3, 4);   % Next frame position
            f{i} = F{i} + Rip * f{i+1}; % Force propagation
            n{i} = N{i} + Rip * n{i+1} + cross(PC{i}, F{i}) + cross(Pip, Rip * f{i+1}); % Torque propagation
        end
        tau(i) = n{i}' * [0; 0; 1]; % Project torque onto joint axis
    end
end