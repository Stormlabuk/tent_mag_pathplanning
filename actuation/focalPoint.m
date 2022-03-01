%%Focal Point Calculation 
function [p0, error] = focalPoint(Ud) 

    error = 0;
    B_des = [Ud(1), Ud(2), Ud(3)];   % Bx        By        Bz
    dB_des = [Ud(4), Ud(5), Ud(6), Ud(7), Ud(8)];      % dBx/dx    dBx,dy    dBx,dz    dBy,dy    dBy,dz  

    %Transform dB into jacobian
    jac = [dB_des(1), dB_des(2), dB_des(3);
           dB_des(2), dB_des(4), dB_des(5);
           dB_des(3), dB_des(5), (-dB_des(1) -dB_des(4))];

    %Finding the new focal point
    try
        p0 = -eye(3)*(pinv(jac)*B_des');
    catch ME
        %If error when finding the pseudo inverse
        error = 1;
    end

end