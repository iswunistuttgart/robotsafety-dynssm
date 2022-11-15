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
function q_stop = stoppingDistance(q,qd,max_qd,m,max_m,aus,max_aus)
%stoppingDistance
%   q_stop = stoppingDistance(robot,q,qd,m, max_qd,max_m,max_aus)
%   
%   Lookup table for the stopping distance for the Kuka KR500 R2830. This table is based on [1].
%   The stopping distance depends on the current pose, the payload and velocity.
%   [1]KUKA Deutschland GmbH (Hg.) (2021): Robots KR 500 FORTEC Mit F‑ und C-Variante Montageanleitung.
% 

    A1(:,:,1,1) = [6 6 12 19; 6 6 12 20; 6 6 12 20];
    A1(:,:,2,1) = [6 6 13.5 27.5; 6 6 15 30 ; 5 5 13 23];
    A1(:,:,3,1) = [6 6 17 36; 6 6 19 41; 5 5 12 23];

    A1(:,:,1,2) = [6 6 12 20; 6 6 12 21; 6 6 12 23];
    A1(:,:,2,2) = [6 6 13 22.5; 6 6 13 25 ; 6 6 13 25];
    A1(:,:,3,2) = [6 6 14 25; 6 6 15 30; 6 6 16.5 34];

    A1(:,:,1,3) = [5 5 12 17; 5 5 12 20; 4 4 7.5 12];
    A1(:,:,2,3) = [5 5 12 17; 5 5 12 20; 4 4 7.5 12];
    A1(:,:,3,3) = [5 5 12 17; 5 5 12 20; 4 4 7.5 12];
    for ax=1:3
        for ausl = 1:3
            override = abs(qd(ax))/max_qd(ax);
            override = min(override,1);
            val_m1 = interp1([0 0.33 0.66 1], A1(1,:,ausl,ax), override);
            val_m2 = interp1([0 0.33 0.66 1], A1(2,:,ausl,ax), override);
            val_m3 = interp1([0 0.33 0.66 1], A1(3,:,ausl,ax), override);
            overmass(ausl) = interp1([0 0.33 0.66 1], [val_m1 val_m1 val_m2 val_m3],m/max_m);
        end
        deg(ax) = interp1([0 0.33 0.66 1], [overmass(1) overmass(1) overmass(2) overmass(3)],abs(aus/max_aus));
    end
    for ax=1:6
        if ax<=3
            if(qd(ax)>0)
                q_stop(ax) = q(ax)+deg2rad(deg(ax));
            else
                q_stop(ax) = q(ax)-deg2rad(deg(ax));
            end
        else
            q_stop(ax) = q(ax);
        end
    end
end
