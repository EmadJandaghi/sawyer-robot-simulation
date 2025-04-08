function NEq=NEquation(state)

nL = 7; % DOF/number of Links
q=state(1:nL);
qd=state(nL+1:2*nL);
g=9.81;

% Inertia:
Data_cgl=[1	5.32	0.0244	0.011	-0.082876753
2	4.51	0.013623247	0.0268	-0.052
3	1.75	0.010723247	0.017	-0.44532
4	2.51	0.015323247	-0.0281	-0.0403
5	1.12	0.012423247	-0.0049	-0.1509
6	1.56	0.004423247	0.0237	-0.0309
7	0.33	0.010523247	0.0106	-0.02975];

Inertia_cg=[1	53.31	4.71	11.73	57.9	8.02	23.66
2	14.61	0.24	6.09	22.4	-0.29	17.3
3	25.51	0	0.012	5.3	-3.32	3.42
4	10.16	-0.01	0.27	6.57	3.03	6.91
5	13.56	0.02	-0.141	3.56	-1.06	1.37
6	4.73	0.12	-0.052	0.97	1.16	3.18
7	0.31	0	0	0.22	-0.01	0.36];
Inertia_cg(:,2:end)=Inertia_cg(:,2:end)/1000 ;%convert to kg.m^2

m=Data_cgl(:,2);
xc=Data_cgl(:,3);
yc=Data_cgl(:,4);
zc=Data_cgl(:,5);

Ixx=Inertia_cg(:,2);
Ixy=Inertia_cg(:,3);
Ixz=Inertia_cg(:,4);
Iyy=Inertia_cg(:,5);
Iyz=Inertia_cg(:,6);
Izz=Inertia_cg(:,7);



IC={zeros(3,3),zeros(3,3),zeros(3,3),zeros(3,3),zeros(3,3),zeros(3,3),zeros(3,3)};
PC={zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1)};
Ti_im={zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4)};
Ti={zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4)};
w={zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1)};
wd={zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1)};
vd={zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1)};
vd_C={zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1)};
F={zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1)};
N={zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1)};

for i=1:nL
    IC{i}=[Ixx(i),-Ixy(i),-Ixz(i);-Ixy(i),Iyy(i),-Iyz(i);-Ixz(i),-Iyz(i),Izz(i)]; % Inertia about C.G 
    PC{i}=[xc(i);yc(i);zc(i)];

end

teta=q;
teta(2)=teta(2)-pi/2;
d=[317*cos(asin(81/317)) 194.5 400 168.5 400 136.3 134.75]'*1e-3;
a=[0 81 0 0 0 0 0]'*1e-3; %a1=81*1e-3
alpha=[0 -pi/2 -pi/2 -pi/2 pi/2 pi/2 -pi/2]';

%==========================================================================
%% Direct Kinematic Analysis
%==========================================================================
% i-1_T_i
for i = 1:nL
    Ti_im{i} =[ cos(teta(i)) -sin(teta(i)) 0 a(i);
        sin(teta(i))*cos(alpha(i)) cos(teta(i))*cos(alpha(i)) -sin(alpha(i)) -sin(alpha(i))*d(i);
        sin(teta(i))*sin(alpha(i)) cos(teta(i))*sin(alpha(i)) cos(alpha(i)) cos(alpha(i))*d(i);
        0 0 0 1];
    
end
% 0_T_i
for i = 1:nL
    if i==1
        Ti{i}=Ti_im{i};
        
    else
        Ti{i}=Ti{i-1}*Ti_im{i};
    end
end


%==========================================================================
%%  Dynamic Model
%==========================================================================
% Outward iterations Equations:
%==============================
for i=1:nL
    % orientation of the next frame relative to the i_th frame
    Ri=Ti_im{i}(1:3,1:3);
    % position of the next frame relative to the i_th frame
    Pi=Ti_im{i}(1:3,4);
    if i==1
        % Angular velocity of i-th link
        w{i}= qd(i)*[0;0;1];
        % Angular acc of i-th link
        wd{i}= 0*[0;0;1];
        % linear acc of the origin of i-th frame
        vd{i}=Ri'*[0;0;g];
        
    else
        
        % Angular velocity of i-th link
        w{i}= Ri'*w{i-1}+qd(i)*[0;0;1];
        % Angular acc of i-th link
        wd{i}= Ri'*wd{i-1}+cross(Ri'*w{i-1},qd(i)*[0;0;1]);
        % linear acc of the origin of i-th frame
        vd{i}=Ri'*(cross(wd{i-1},Pi)+cross(w{i-1},cross(w{i-1},Pi))+vd{i-1});
        
    end
    
    % linear acc of the origin of i-th frame
    vd_C{i}=cross(wd{i},PC{i})+cross(w{i},cross(w{i},PC{i}))+vd{i};
    %  Linear Inertia force
    F{i}=m(i)*vd_C{i};
    %   Inertia torques about c.g
    N{i}=IC{i}*wd{i}+cross(w{i},IC{i}*w{i});
    
end

% Inward iterations
%==================
for i=nL:-1:1
    % orientation of the next frame relative to the i_th frame
    
    if i==nL
        
        f{i}=F{i};
        n{i}=N{i}+cross(PC{i},F{i});
    else
        Rip=Ti_im{i+1}(1:3,1:3);
        % position of the next frame relative to the i_th frame
        Pi=Ti_im{i}(1:3,4);
        
        Tip_i=Ti_im{i+1};
        Pip_i=Tip_i(1:3,4);
        
        f{i}=F{i}+Rip*f{i+1};
        n{i}=N{i}+Rip*n{i+1}+cross(PC{i},F{i})+cross(Pip_i,Rip*f{i+1});
    end
    NEq(i)=n{i}'*[0;0;1];
    
end



end