function [K R C] = getCamera(ax)
%Matti Jukola 2010
%Slight modification 2012-10-28 (return as K, R, C)
if nargin < 1
    ax = gca;
end
set(ax,'projection','perspective')
P = view(gca);
P(2,:) = -P(2,:);
P = -P;
P = P(1:3,:);
[K R C] = decomposeP(P);
C = C(1:3);