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
function C=projectOn(A,B)
%projectOn
%   C = projectOn(A,B) calculates hte projection of vector A to vector B
%
%   Class support for inputs A, B:
%      float: double, single
    C=(sum(A.*B)/(norm(B)^2))*B;
end