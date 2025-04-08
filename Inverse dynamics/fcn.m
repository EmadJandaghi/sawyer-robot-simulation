function qv = fcn(t, A, phi)
    % Defines joint angles, velocities, and accelerations as sinusoidal functions of time.
    % Input: 
    %   t - Time (scalar, seconds)
    %   A - Amplitudes for each joint (7x1 vector, optional, default: [1, 0.5, 0.3, 0.2, 0.1, 0.05, 0])
    %   phi - Phase shifts for each joint (7x1 vector, optional, default: zeros(7,1))
    % Output: qv - [q; qd; qdd] (21x1 vector: angles, velocities, accelerations)

    % Default amplitudes (rad) for interesting variation
    if nargin < 2 || isempty(A)
        A = [1, 0.5, 0.3, 0.2, 0.1, 0.05, 0]'; % Joint 1 has largest amplitude, Joint 7 is static
    end

    % Default phase shifts (rad) for staggered motion
    if nargin < 3 || isempty(phi)
        phi = zeros(7, 1); % No phase shift by default
    end

    % Frequencies (rad/s) for each joint to create diversity
    omega = [1, 1.5, 2, 2.5, 3, 3.5, 4]'; % Increasing frequency for each joint

    % Joint angles (rad)
    q = A .* sin(omega * t + phi);

    % Joint velocities (rad/s)
    qd = A .* omega .* cos(omega * t + phi);

    % Joint accelerations (rad/s^2)
    qdd = -A .* omega.^2 .* sin(omega * t + phi);

    % Combine into state vector
    qv = [q; qd; qdd];
end