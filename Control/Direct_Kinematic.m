function pos=Direct_Kinematic(q)

nL=7;
% Denavit Hartenberg Parameters:
d=[317*cos(asin(81/317)) 194.5 400 168.5 400 136.3 134.75]'*1e-3;
a=[0 81 0 0 0 0 0]'*1e-3; 
alpha=[0 -pi/2 -pi/2 -pi/2 pi/2 pi/2 -pi/2]';
teta=q;
teta(2)=teta(2)-pi/2;
%transformation matrix
Ti_im={zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4),zeros(4,4)};
Ti=Ti_im;
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
% End Effector position:
Te=Ti{nL};                                                                                                                                                                                                                                                                                                                                                                                                                                                         
loc=Te(1:3,4);


%Z–  Y'– X'' Euler angles or X   Y    Z
%al bet ga                  ga   bet  al
R=Te(1:3,1:3);
bet=atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));
if bet==pi/2
alph=0;
gam=atan2(R(1,2),R(2,2));
elseif bet==-pi/2
alph=0;
gam=-atan2(R(1,2),R(2,2));
else
alph=atan2(R(2,1)/cos(bet),R(1,1)/cos(bet));
gam=atan2(R(3,2)/cos(bet),R(3,3)/cos(bet));
end
orientation=[alph bet  gam]';
pos=[loc;orientation];

end