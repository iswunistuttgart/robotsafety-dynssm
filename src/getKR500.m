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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function robot = getKR500()
%Returns the rigidBodyTree of the Kuka KR500
%   Detailed explanation goes here

body1 = rigidBody('base_link');
body1.Inertia = [0 0 0 0 0 0];
body2 = rigidBody('kr500_base_link');
body2.Inertia = [0.0300 0.0300 0.0300 0 0 0];
body3 = rigidBody('kr500_link_1');
body3.Inertia = [0.0300 0.0300 0.0300 0 0 0];
body4 = rigidBody('kr500_link_2');
body4.Inertia = [0.0300 0.0300 0.0300 0 0 0];
body5 = rigidBody('kr500_link_3');
body5.Inertia = [0.0300 0.0300 0.0300 0 0 0];
body6 = rigidBody('kr500_link_4');
body6.Inertia = [0.0300 0.0300 0.0300 0 0 0];
body7 = rigidBody('kr500_link_5');
body7.Inertia = [0.0300 0.0300 0.0300 0 0 0];
body8 = rigidBody('kr500_link_6');
body8.Inertia = [0.0300 0.0300 0.0300 0 0 0];

joint1 = rigidBodyJoint('fixed_world','fixed');
joint2 = rigidBodyJoint('fixed_robot','fixed');

joint3 = rigidBodyJoint('kr500_joint_a1','revolute');
joint3.HomePosition = 0;
joint3.JointAxis = [0 0 1];
joint3.PositionLimits = [-3.2289 3.2289];

joint4 = rigidBodyJoint('kr500_joint_a2','revolute');
joint4.HomePosition = 0;
joint4.JointAxis = [0 1 0];
joint4.PositionLimits = [-0.6981 1.9199];
setFixedTransform(joint4,trvec2tform([0.5, 0, 1.0450]))

joint5 = rigidBodyJoint('kr500_joint_a3','revolute');
joint5.HomePosition = 0;
joint5.JointAxis = [0 1 0];
joint5.PositionLimits = [-3.3161 0.9425];
setFixedTransform(joint5,trvec2tform([0, 0, 1.3]))

joint6 = rigidBodyJoint('kr500_joint_a4','revolute');
joint6.HomePosition = 0;
joint6.JointAxis = [1 0 0];
joint6.PositionLimits = [-6.1087 6.1087];
setFixedTransform(joint6,trvec2tform([0.525, 0, -0.0550]))

joint7 = rigidBodyJoint('kr500_joint_a5','revolute');
joint7.HomePosition = 0;
joint7.JointAxis = [0 1 0];
joint7.PositionLimits = [-2.0944 2.0944];
setFixedTransform(joint7,trvec2tform([0.5, 0, 0]))

joint8 = rigidBodyJoint('kr500_joint_a6','revolute');
joint8.HomePosition = 0;
joint8.JointAxis = [1 0 0];
joint8.PositionLimits = [-6.1087 6.1087];
setFixedTransform(joint8,trvec2tform([0.29, 0, 0]))

body1.Joint = joint1;
body2.Joint = joint2;
body3.Joint = joint3;
body4.Joint = joint4;
body5.Joint = joint5;
body6.Joint = joint6;
body7.Joint = joint7;
body8.Joint = joint8;
robot = rigidBodyTree;
addBody(robot,body1,'base');
addBody(robot,body2,'base_link');
addBody(robot,body3,'kr500_base_link');
addBody(robot,body4,'kr500_link_1');
addBody(robot,body5,'kr500_link_2');
addBody(robot,body6,'kr500_link_3');
addBody(robot,body7,'kr500_link_4');
addBody(robot,body8,'kr500_link_5');
end

