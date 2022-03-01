function [R, theta, w] = Vec2Rot(v, u)
% This function computes the rotation matrix (R) between the vectors v and
% u.

% Find the orthgonal axis
w = Skew(v)*u;

% Find the angle
s_theta = norm(w)/(norm(v)*norm(u));
c_theta = v.'*u/(norm(v)*norm(u));
theta = atan2(s_theta, c_theta);

% Build rotation
R = eye(3);
if norm(w) ~= 0
    R = R + sin(theta)*Skew(w)/norm(w) + (1 - cos(theta))*...
        (Skew(w)/norm(w))^2;
end

end