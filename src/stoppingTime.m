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
function t_stop = stoppingTime(qd,max_qd,m,max_m,aus,max_aus)
%stoppingDistance
%   q_stop = stoppingTime(qd,max_qd,m,max_m,aus,max_aus)
%   
%
%   Lookup table for the stopping time for the Kuka KR500 R2830. This table is based on [1].
%   The stopping time depends on the current pose, the payload and velocity.
%   [1]KUKA Deutschland GmbH (Hg.) (2021): Robots KR 500 FORTEC Mit F‑ und C-Variante Montageanleitung.
% 

    A1(:,:,1,1) = [0.35 0.35 0.36 0.38; 0.35 0.35 0.37 0.4; 0.35 0.35 0.37 0.42];
    A1(:,:,2,1) = [0.37 0.37 0.54 0.55; 0.37 0.37 0.54 0.6 ; 0.37 0.37 0.54 0.7];
    A1(:,:,3,1) = [0.4 0.4 0.58 0.75 ; 0.52 0.52 0.2 0.91; 0.4 0.4 0.58 0.75];

    A1(:,:,1,2) = [0.4 0.4 0.42 0.49; 0.4 0.4 0.42 0.58; 0.4 0.4 0.42 0.6];
    A1(:,:,2,2) = [0.4 0.4 0.42 0.58; 0.4 0.4 0.43 0.6 ; 0.4 0.4 0.59 0.67];
    A1(:,:,3,2) = [0.4 0.4 0.45 0.6; 0.42 0.42 0.59 0.75; 0.45 0.45 0.62 0.81];

    A1(:,:,1,3) = [0.38 0.38 0.4 0.41;0.38 0.38 0.41 0.49 ;0.38 0.38 0.4 0.42];
    A1(:,:,2,3) = [0.38 0.38 0.4 0.41;0.38 0.38 0.41 0.49 ;0.38 0.38 0.4 0.42];
    A1(:,:,3,3) = [0.38 0.38 0.4 0.41;0.38 0.38 0.41 0.49 ;0.38 0.38 0.4 0.42];
    for ax=1:3
        for ausl = 1:3
            override = abs(qd(ax))/max_qd(ax);
            override = min(override,1);
            val_m1 = interp1([0 0.33 0.66 1], A1(1,:,ausl,ax), override);
            val_m2 = interp1([0 0.33 0.66 1], A1(2,:,ausl,ax), override);
            val_m3 = interp1([0 0.33 0.66 1], A1(3,:,ausl,ax), override);
            overmass(ausl) = interp1([0 0.33 0.66 1], [val_m1 val_m1 val_m2 val_m3],m/max_m);
        end
        t(ax) = interp1([0 0.33 0.66 1], [overmass(1) overmass(1) overmass(2) overmass(3)],abs(aus/max_aus));
    end
    for ax=1:6
        if ax<=3
            t_stop(ax) = t(ax);
        else
            t_stop(ax) = 0;
        end
    end
end
