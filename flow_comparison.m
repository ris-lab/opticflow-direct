clear all;
close all;

% run python script for getting the flow
status = system('python compute_optic_flow.py')	% if this doesnt work, just run it in command line.


load('mocap_data.mat')

load('directflow.mat')	% variable name is directflow
load('lkflow.mat')	% variable name is opticflow




rxy_scale = 0.5*235.5^-1;	% from calibration, Odroid cam
fps = 60; 					% based on 60 fps, assumed constant


T = 0.002;	% MOCAP data acquisition at 500 Hz
diff_kernel = [1,-1];
Fc=20; 		%cutoff frequency of the low pass filter
[pplpb, pplpa]=cheby2(3, 30, Fc*T); 

ml_time = mocap.time;		% time stamp
ml_frame = mocap.frame;		% frame id

ml_X = mocap.X;			% position of the camera in the world frame (from the MOCAP)
ml_Y = mocap.Y;
ml_Z = mocap.Z-0.013;
ml_qx =mocap.qx;	% quaternion
ml_qy =mocap.qy;
ml_qz =mocap.qz;
ml_qw =mocap.qz;
R11 = 1-2*ml_qy.^2-2*ml_qz.^2;			% rotation matrix
R12 = 2*(ml_qx.*ml_qy-ml_qz.*ml_qw);
R13 = 2*(ml_qx.*ml_qz+ml_qy.*ml_qw);
R21 = 2*(ml_qx.*ml_qy+ml_qz.*ml_qw);
R22 = 1-2*ml_qx.^2-2*ml_qz.^2;
R23 = 2*(ml_qy.*ml_qz-ml_qx.*ml_qw);
R31 = 2*(ml_qx.*ml_qz-ml_qy.*ml_qw);
R32 = 2*(ml_qy.*ml_qz+ml_qx.*ml_qw);
R33 = 1-2*ml_qx.^2-2*ml_qy.^2;

[ff, iml, iff] = unique(ml_frame, 'first');


% flow scaling

% direct flow
dr_raw_rx = -directflow(max(ml_frame,1),2)*fps*rxy_scale;	% /vartheta_x+\omega_y
dr_raw_ry = -directflow(max(ml_frame,1),3)*fps*rxy_scale;	% /vartheta_y-\omega_y
dr_raw_rz = directflow(max(ml_frame,1),4)*fps;				% /vartheta_z
% lk flow
lk_raw_rx = opticflow(max(ml_frame,1),2)*fps*rxy_scale;
lk_raw_ry = opticflow(max(ml_frame,1),3)*fps*rxy_scale;
lk_raw_rz = opticflow(max(ml_frame,1),4)*fps;


R11d = conv(filtfilt(pplpb,pplpa,R11), diff_kernel, 'same')/T;	% time derivatives of the rotation matrix
R12d = conv(filtfilt(pplpb,pplpa,R12), diff_kernel, 'same')/T;
R13d = conv(filtfilt(pplpb,pplpa,R13), diff_kernel, 'same')/T;
R21d = conv(filtfilt(pplpb,pplpa,R21), diff_kernel, 'same')/T;
R22d = conv(filtfilt(pplpb,pplpa,R22), diff_kernel, 'same')/T;
R23d = conv(filtfilt(pplpb,pplpa,R23), diff_kernel, 'same')/T;
R31d = conv(filtfilt(pplpb,pplpa,R31), diff_kernel, 'same')/T;
R32d = conv(filtfilt(pplpb,pplpa,R32), diff_kernel, 'same')/T;
R33d = conv(filtfilt(pplpb,pplpa,R33), diff_kernel, 'same')/T;
ml_wx = R13.*R12d+R23.*R22d+R33.*R32d;	% angular velocities calculated from the MOCAP
ml_wy = R11.*R13d+R21.*R23d+R31.*R33d;
ml_wz = R12.*R11d+R22.*R21d+R32.*R31d;

ml_d = ml_Z./R33;		% altitude

ml_Xd = conv(filtfilt(pplpb,pplpa,ml_X), diff_kernel, 'same')/T;	% velocities in the MOCAP frame
ml_Yd = conv(filtfilt(pplpb,pplpa,ml_Y), diff_kernel, 'same')/T;
ml_Zd = conv(filtfilt(pplpb,pplpa,ml_Z), diff_kernel, 'same')/T;

ml_rx = (R11.*ml_Xd + R21.*ml_Yd + R31.*ml_Zd)./ml_d;	% ground truth of vartheta_x,y,z
ml_ry = (R12.*ml_Xd + R22.*ml_Yd + R32.*ml_Zd)./ml_d;
ml_rz = (R13.*ml_Xd + R23.*ml_Yd + R33.*ml_Zd)./ml_d;

ml_rx(isnan(ml_rx))=0; ml_rx(isinf(ml_rx))=0;
ml_ry(isnan(ml_ry))=0; ml_ry(isinf(ml_ry))=0;
ml_rz(isnan(ml_rz))=0; ml_rz(isinf(ml_rz))=0;

% re-alignment as the body-frame of the camera from the MOCAP is not aligned with the camera frame
temp = ml_rx;
ml_rx = -ml_ry;
ml_ry = -temp;
ml_rz = -ml_rz;
temp = ml_wx;
ml_wx = -ml_wy;
ml_wy = -temp;
ml_wz = -ml_wz;


% for error calculation/comparision
iind = 3000;
lind = 18000;
offset = 42;		% to account for the latency between the two systems.

figure(1)
plot(ml_frame(iind+offset:lind), ml_rx(iind:lind-offset), 'r');
hold on
plot(ml_frame(iind+offset:lind), 1*dr_raw_rx(iind+offset:lind)+(-1)*ml_wy(iind:lind-offset) , 'b')
plot(ml_frame(iind+offset:lind), 1*lk_raw_rx(iind+offset:lind)+(-1)*ml_wy(iind:lind-offset), 'g');
hold off
ylim([-1,1])
title('vartheta x')
xlabel('frame id')

figure(2)
plot(ml_frame(iind+offset:lind), ml_ry(iind:lind-offset), 'r');
hold on
plot(ml_frame(iind+offset:lind), 1*dr_raw_ry(iind+offset:lind)+ml_wx(iind:lind-offset) , 'b')
plot(ml_frame(iind+offset:lind), 1*lk_raw_ry(iind+offset:lind)+ml_wx(iind:lind-offset), 'g');
hold off
ylim([-1,1])
title('vartheta y')
xlabel('frame id')


figure(3)
plot(ml_frame(iind+offset:lind), ml_rz(iind:lind-offset), 'r');
hold on
plot(ml_frame(iind+offset:lind), 1*dr_raw_rz(iind+offset:lind) , 'b')
plot(ml_frame(iind+offset:lind), 1*lk_raw_rz(iind+offset:lind) , 'g');
hold off
ylim([-1,1])
title('vartheta z')
xlabel('frame id')

%% RMS error

% Direct Flow
disp('RMS errors of Direct Flow')
([rms( 1*dr_raw_rx(iind+offset:lind)+ml_wx(iind:lind-offset) - -ml_ry(iind:lind-offset) ), rms( 1*dr_raw_ry(iind+offset:lind)-ml_wy(iind:lind-offset) - -ml_rx(iind:lind-offset) ), rms( 1*dr_raw_rz(iind+offset:lind) - -ml_rz(iind:lind-offset) )])
% LK Flow
disp('RMS errors of LK Flow')
([rms( 1*lk_raw_rx(iind+offset:lind)+ml_wx(iind:lind-offset) - -ml_ry(iind:lind-offset) ),rms( 1*lk_raw_ry(iind+offset:lind)-ml_wy(iind:lind-offset) - -ml_rx(iind:lind-offset) ), rms( 1*lk_raw_rz(iind+offset:lind) - -ml_rz(iind:lind-offset) )])
