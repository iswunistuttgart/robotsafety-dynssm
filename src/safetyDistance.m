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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function S=safetyDistance(robot,tspan,q,qd,m,p_r,p_h,v_r,v_h,varargin)
%safetyDistance  Saftey Distance after DIN ISO 1506:2017-04.
%   S =
%   safetyDistance(robot,tspan,ts,q,qd,p_r,p_h,v_r,v_h,'safetycase',case)
%   calculates the required minimum safety distance for a simulation with
%   robot=RigrifRobotTree of the robot
%   tspan = timespan is the time vector
%   q = angles of the joints
%   qd = angles velocity of the joints
%   m = payload mass
%   p_r = robot position for each joint in cartesian space.
%   p_h = human position for each joint in cartesian space.
%   v_r = robot veolcity for each joint in cartesian space.
%   v_h = human velocity for each joint in cartesian space.   
%   'safetycase' = 'LightFence','SafetEyeye','HumanModeling'
%
%
%Comments from the ISO/TS 15066:2016
%
%       S = S_h + S_r + S_s + C + Z_d + 
%
%S      the protective separation distance at time i for joint j and k;
%S_h    the contribution to the protective separation distance attributable to the operator’s change in location;
%S_r    the contribution to the protective separation distance attributable to the robot system’s reaction time;
%S_s    the contribution to the protective separation distance due to the robot system’s stopping distance;
%C      the intrusion distance, as defined in ISO 13855; this is the distance that a part of the body can
%       intrude into the sensing field before it is detected;
%Z_d    the position uncertainty of the operator in the collaborative workspace, as measured by the
%       presence sensing device resulting from the sensing system measurement tolerance;
%Z_r    the position uncertainty of the robot system, resulting from the accuracy of the robot position
%       measurement system.
%T_r    the reaction time of the robot system, including times required for detection of operator position,
%       processing of this signal, activation of a robot stop, but excluding the time it takes the robot
%       to come to a stop;
%T_s    the stopping time of the robot, from the activation of the stop command until the robot has
%       halted; Ts is not a constant, but rather a function of robot configuration, planned motion, speed,
%       end effector and load;
    
    numJoints = numel(robot.homeConfiguration);
    numJointsHuman = numel(v_h);
    
    safetycase = 0;%Safetycase:0=LightFence(lf),1=SafetEyeye(se),2=HumanModeling
    for i=1:2:nargin-10
        if strcmpi(varargin{i},'safetycase')==1
            %Robot specific definitins
            T_s     = 0.91;     % from [6] sopping time of KR500 R2830 WorstCase for max payload and velocity
            max_m = 611;        % from [6] 
            max_qd = [pi/2 deg2rad(80) deg2rad(75)]; %  from [6] 
            q_aus = robot.homeConfiguration;
            qdeg_aus = [0 deg2rad(90) deg2rad(-90) 0 0 0];
            for j=1:numJoints
                q_aus(j).JointPosition = qdeg_aus(j);
            end
            T_maxaus = getTransform(robot,q_aus,'kr500_link_5');
            c_maxuas = T_maxaus(1:3,4)';
            max_aus = c_maxuas(1);

            %[6] KUKA KR500 Spezification
            if strcmpi(varargin{i+1},'LightFence')==1
                safetycase=0;
                %Assume: Ligth fence with Pilz PSEN_opII4H + EtherCatSafe
                %Bus and SafetyPLC TwinSafe
                C       = 0.128;    % C=8(d-14) in mm where d=30mm from DIN EN 13855
                t_sensor= 0.0123;   % afromus [1]
                t_io    = 0.004;    % e.g. Beckhoff EL1904 [2]
                t_bus   = 3*0.003;  % from [2] 3*CycleTime estiamted to 3ms
                t_logic = 0.001;    % estimated
                T_r     = t_sensor+t_io+t_bus+t_logic+t_bus+t_io;
                %[1] Pilz GmbH & Co. KG (Hg.) (2019): PSEN opII4H-Serie.
                %[2] Beckhoff Automation GmbH & Co. KG (Hg.) (2020): Operation Manual EL6910 TwinSAFE Logic Terminal.
               
            elseif strcmpi(varargin{i+1},'SafetEye')==1
                safetycase=1;
                C       = 0.208;% from [4]
                T_r     = 0.3;  % from [4] standard configuration
                % [4] Pilz GmbH & Co. KG (Hg.) (2018): SafetyEYE Bedienungsanleitung-21743-DE-22.
            elseif strcmpi(varargin{i+1},'RobotModeling')==1
                safetycase=2;
                % We use the same values as in HumanModeling as we think of
                % an optimized vision system compared to the
                C       = 0.1;  % from [5]
                T_r     = 0.3;  % from [5]  Estimation of the vision pipeline
            elseif strcmpi(varargin{i+1},'HumanModeling')==1
                safetycase=3;
                C       = 0.1;  % from [5]
                T_r     = 0.3;  % from [5]  Estimation of the vision pipeline
                %[5] Ramer, Christina (2018): Arbeitsraumüberwachung und autonome Bahnplanung für ein sicheres und flexibles Roboter-Assistenzsystem in der Fertigung. Dissertation. FAU, Erlangen.
            end
        end
    end
    simLength = length(tspan);
    timeDiff = tspan(end)-tspan(1);
    v_h_max=1.6; %from standard
    Z_d = 0.1;
    Z_r = 0.001;
      
    
    for i=1:simLength*((timeDiff-(T_r+T_s))/timeDiff)-1
        fprintf("step %d\n",i);
        %calculate the current ausladung
        q_aus = robot.homeConfiguration;
        numJoints = numel(robot.homeConfiguration);
        for j=1:numJoints
            q_aus(j).JointPosition = q(i,j);
        end
        T_aus = getTransform(robot,q_aus,'kr500_link_5');
        aus = norm(T_aus(1:2,4));

        stoptime = max(stoppingTime(qd(i,:),max_qd,m,max_m,aus,max_aus));
        stopdist = stoppingDistance(q(i,:),qd(i,:),max_qd,m,max_m,aus,max_aus);
        for j=1:numJoints
            v_r_max = max(sqrt(sum(v_r{j}.^2,2)));
            %s_r    The quantity S_r represents the contribution to the separation distance due to the robot’s motion upon
            %       the person entering the sensing field up to the control system activating a stop. Here, vr is a function of
            %       time, and can vary due to either the robot’s speed or direction changing. The system shall be designed
            %       to account for vr varying in the manner that reduces the separation distance the most:
            %       S_r = int{t0...t0+T_r}  v_r(t) dt
            %S_s    The quantity Ss represents the contribution to the separation distance due to the robot’s motion during
            %       robot stopping. Here, vs is a function of time and can vary due to either the robot’s speed or direction
            %       changing. The system shall be designed to account for vs varying in the manner that reduces the
            %       separation distance the most
            %S_r_torobot = 0;
            %S_s_torobot = 0;
            if safetycase>1
                %if the robot’s speed is being monitored, the system design may use the current speed of the robot, but
                %shall account for the acceleration capability of the robot in the manner that reduces the separation
                %distance the most. IN contrast to the norm, we do not consider the acceleration instead we use the
                %speed of the known trajectory,
                t_start = tspan(i);
                t_end = t_start + T_r;
                x_t = [t_start t_end]';
                S_r_interpolated = interp1(tspan(1:end-1), v_r{j}, x_t);       % Interpolate
                S_r = trapz(x_t,S_r_interpolated);                

                %is the stopping time of the robot, from the activation of the stop command until the robot has
                %halted; Ts is not a constant, but rather a function of robot configuration, planned motion, speed,
                %end effector and load;
                T_s_cur = stoptime;

                %if the robot’s speed is being monitored, the system design may use the robot’s stopping distance
                %from that speed, applied in the direction that reduces the separation distance the most.

                t_start = tspan(i)+T_r;
                t_end = t_start + T_s_cur;
                x_t = [t_start t_end]';
                q_stop_interpolated = interp1(tspan(1:end), q(:,:), x_t(1));        % Interpolate current q
                q_stop_interpolated = q_stop_interpolated + stopdist;               % Add stop distance to current q
                bodieNames = {'kr500_link_1','kr500_link_2','kr500_link_3','kr500_link_4','kr500_link_5','kr500_link_6'};
                p_r_curr = interp1(tspan(1:end), p_r{j}, x_t(1));
                S_s = q2cartesian(robot,q,bodieNames{j}) - p_r_curr;
               
            else
                %if the robot’s speed is not being monitored, the system design shall assume that vr is the maximum
                %speed of the robot;
                S_r_torobot = v_r_max*T_r;
                %if the robot’s speed is not being monitored, the system design shall assume that this integral is the
                %robot’s stopping distance in the direction that reduces the separation distance the most;
                S_s_torobot = v_r_max*T_s;
            end

            for k=1:numJointsHuman
                vec = p_r{j}(i,:)-p_h{k}(i,:);
                if safetycase>1
                    S_s_torobot = projectOn(S_s,vec);
                    S_r_torobot = projectOn(S_r,vec);
                end

                %S_h    The contribution to the protective separation distance attributable to the operator’s change in 
                %       location, Sh is
                %       S_h=int{t0...t0+T_r+T_s} v_h(t) dt
                if safetycase>2
                    %The system shall be designed to account for vh varying in the
                    %manner that reduces the separation distance the most.
                    
                    % the stopping time of the robot, from the activation of the stop command until the robot has
                    %halted; Ts is not a constant, but rather a function of robot configuration, planned motion, speed,
                    %end effector and load;
                    T_s_cur = stoptime;
    
                    %is the directed speed of an operator in the collaborative workspace in the direction of the moving
                    %part of the robot, and can be positive or negative depending on whether the separation distance
                    %is increasing or decreasing;
                    v_h_torobot = norm(projectOn(v_h{k}(i,:),vec));
    
                    %Consideration of the additional possible acceleration of the human in
                    %direction of the robot
                    %Numerical approximation by assuming a constant
                    %acceleration in the direction of the robot
                    steps = 20; 
                    acc = 8; %m/s^2 assumption of the maximum acceleration 
                    t = linspace(0,T_r+T_s_cur,steps);
                    vel = cumtrapz(t,ones(1,steps).*acc)+v_h_torobot;
                    %The resulting speed is limited by the maximum speed of the standard 
                    vel = min(vel,v_h_max);
                    
                    S_h_withacceleration = trapz(t,vel) ;

                    %Without the additional acceleration, S_h can be calculated by
                    %S_h_noacceleration = norm((v_h_torobot)*(T_r+T_s_cur));
    
                    S_h_torobot = S_h_withacceleration;
                    
                else
                    %If the person’s speed is not being monitored, the system design shall assume that vh is 1,6 m/s
                    %in the direction that reduces the separation distance the
                    %most. According to ISO 13855 and IEC/TS 62046:2008, 4.4.2.3, vh may be a value other than 1,6 m/s
                    %depending on a risk assessment.
                    S_h_torobot = v_h_max*(T_r+T_s);
                end

                S(i,j,k) = norm(S_h_torobot) + norm(S_r_torobot) + norm(S_s_torobot) + C + Z_d + Z_r;
            end
        end
    end
end

