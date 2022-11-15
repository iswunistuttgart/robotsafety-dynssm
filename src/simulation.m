%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @copyright: (c) 2022, Institute for Control Engineering of Machine Tools and Manufacturing Units,
%             University of Stuttgart
%             All rights reserved. Licensed under the Apache License, Version 2.0 (the "License");
%             you may not use this file except in compliance with the License.
%             You may obtain a copy of the License at
%                  http://www.apache.org/licenses/LICENSE-2.0
%             Unless required by applicable law or agreed to in writing, software
%             distributed under the License is distributed on an "AS IS" BASIS,
%             WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%             See the License for the specific language governing permissions and
%             limitations under the License.
% @author: Marc Fischer <marc.fischer@isw.uni-stuttgart.de>
% @description: Simulation of dynamic SSM evaluation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
clc

%% Load Robot as RigidTreeObject
robot = getKR500;
numJoints = numel(robot.homeConfiguration);

%% Load trajectories generated by the virtual commissioning tool ISG Virtuos and load the sphere radii.
close all
sphere_table = readtable('trajectory/sphere_model.csv','Delimiter',';','Format','auto');

sphere_r = table2array(sphere_table(:,14:19));
sphere_h = table2array(sphere_table(:,1:13));

trajec =  readtable('trajectory/trn1.csv','Delimiter',';','DecimalSeparator',',');
%Filter for unique time stamps
[~,unique_i,~] = unique(trajec(:,1));
trajec_un = trajec(unique_i,:);

%Down sample the trajectory data.
trajec_un = trajec_un(1:50:end,:);

%Extract time array
[simLength,~] = size(trajec_un);
ts = table2array(trajec_un(2,1))-table2array(trajec_un(1,1));
tspan = table2array(trajec_un(1:end,1));

%% Get position p and velocity v of robot in cartesian space
clearvars p_r v_r
for j=0:numJoints-1
    begin = 41+j*3;
    vend = 41+j*3+2;
    p_r{j+1}=table2array(trajec_un(:,begin:vend));
    v_r{j+1} = diff(p_r{j+1})./ts;
end

%% Get robot configuration q in the configuration space
clearvars q qd
for j=0:numJoints-1
    begin = 59+j;
    q(:,j+1)=table2array(trajec_un(:,begin));
    qd(:,j+1) = diff(q(:,j+1))./ts;
end
%Correct home position form trajectory. The generated trajectory assumes a
%wrong robot home position.
q(:,2) = q(:,2) + deg2rad(36.5168);
q(:,3) = q(:,3) - deg2rad(36.5168) + deg2rad(2.3956);

%% Get position p and velocity v of the human in cartesian space
clearvars p_h v_h
numJointsHuman = 13;
for j=0:numJointsHuman-1
    begin = 2+j*3;
    vend = 2+j*3+2;
    p_h{j+1}=table2array(trajec_un(:,begin:vend));
    v_h{j+1} = diff(p_h{j+1})./ts;
end

%% Plot robot and human trajectories
f1 = figure();
h2 = plot3(p_r{numJoints}(:,1),p_r{numJoints}(:,2),p_r{numJoints}(:,3));
hold on
h3 = plot3(p_h{1}(:,1),p_h{1}(:,2),p_h{1}(:,3),'LineWidth', 1.5);
legend([h2 h3],'Robot Link6','Human Head');
xlabel('x in m');
ylabel('y in m');
zlabel('z in m');


%% Plot max velocity of human and robot
clearvars v_h_j v_r_j v_h_max v_r_max
v_h_max = max(squeeze(sqrt(sum(cat(3,v_h{:}).^2,2)))');
v_r_max = max(squeeze(sqrt(sum(cat(3,v_r{:}).^2,2)))');

f2 = figure();
hold on
plot(tspan(1:end-1),v_h_max');
plot(tspan(1:end-1),v_r_max');
hold off
legend('max v_{human}','max v_{robot}');
xlabel('t in s');
ylabel('v in m/s');

%% Saftydistance by DIN EN 15066
%Calculate the required safety distance between each joint
m = 200; %kg = Payload
S_lightfence =     safetyDistance(robot,tspan,q,qd,m,p_r,p_h,v_r,v_h,'safetyCase','LightFence');
fprintf("Sim lightfence done\n");
S_safetyeye = safetyDistance(robot,tspan,q,qd,m,p_r,p_h,v_r,v_h,'safetyCase','SafetEye');
fprintf("Sim safetyeye done\n");
S_robotEst = safetyDistance(robot,tspan,q,qd,m,p_r,p_h,v_r,v_h,'safetyCase','RobotModeling');
fprintf("Sim robot done\n");
S_humaneEst = safetyDistance(robot,tspan,q,qd,m,p_r,p_h,v_r,v_h,'safetyCase','HumanModeling');
fprintf("Sim humanrobot done\n");

%Extract the maximum
clearvars  S_lightfence_max S_robotEst_max S_humaneEst_max
S_lightfence_max = max(squeeze(max(S_lightfence,[],2)),[],2);
S_safetyeye_max = max(squeeze(max(S_safetyeye,[],2)),[],2);
S_robotEst_max = max(squeeze(max(S_robotEst,[],2)),[],2);
S_humaneEst_max = max(squeeze(max(S_humaneEst,[],2)),[],2);


S_global_max = max([max(S_lightfence_max),max(S_safetyeye_max),max(S_humaneEst_max),max(S_robotEst_max)]);

xTimeLF = tspan(1:length(S_lightfence_max));
xTimeSE = tspan(1:length(S_safetyeye_max));
xTimeRE = tspan(1:length(S_robotEst_max));
xTimeHE = tspan(1:length(S_humaneEst_max));

%% Make plots
f3 = figure();
subplot(2,2,1);
plot(xTimeLF,S_lightfence_max);
title("S with light fence");
xlabel('t in s');
ylabel('S(t)');
ylim([0 S_global_max]);
legend({"joint1","joint2","joint3","joint4","joint5","joint6"});

subplot(2,2,2);
plot(xTimeSE,S_safetyeye_max);
title("S with safety eye");
ylim([0 S_global_max]);
xlabel('t in s');
ylabel('S(t)');

subplot(2,2,3);
plot(xTimeRE,S_robotEst_max);
title("S with robot estimation");
ylim([0 S_global_max]);
xlabel('t in s');
ylabel('S(t)');

subplot(2,2,4);
plot(xTimeHE,S_humaneEst_max);
title("S with human modeling");
ylim([0 S_global_max]);
xlabel('t in s');
ylabel('S(t)');

%Calculate min distance between human and robot
clearvars dist_rh_k dist_rh
for j=1:numJoints
    for k=1:numJointsHuman
        vecDist_rh = p_r{j}(:,:)-p_h{k}(:,:);
        dist_rh_k(k,:) = sqrt(sum(vecDist_rh.^2,2));
        %substract sphere radius
        dist_rh_k(k,:) =  dist_rh_k(k,:)-sphere_r(j)-sphere_h(k);
    end
    dist_rh(j,:) = min(dist_rh_k);
end
dist_rh_min = min(dist_rh);

f4=figure();
xTimeDist = tspan(1:length(dist_rh_min));
plot(xTimeDist,dist_rh_min','LineWidth',2);
hold on
plot(xTimeLF,S_lightfence_max',xTimeSE,S_safetyeye_max',xTimeRE,S_robotEst_max',xTimeHE,S_humaneEst_max');
legend('min dist. h-r','S light fence','S SafetyEye','S r moni.','S r+h moni.','Orientation','horizontal');
xlim([0 15]);
xlabel('t in s');
ylabel('Distance in m');
%% Geneate plots
% You will need https://github.com/matlab2tikz/matlab2tikz
% Add the extension to the path with
% addpath(genpath('<...>\matlab2tikz-master'))
cleanfigure;
file_name = 'safety_dist';
full_export_path = fullfile('figs', [file_name, '.tex']);
matlab2tikz('figurehandle',f4,'filename',full_export_path,'height', '\figureheight', 'width', '\figurewidth');

