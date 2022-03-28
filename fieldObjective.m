function [Bmin] = fieldObjective(m, mOld, r, rOld, Ud)
    %function to find the dirivitive of the field
    %to be used for optimisation 

    %m [6 x 1]: Magnets Orientation we want to minimize
    %mOld [6 x 1]: Past Orientation of magnets
    %r [6 x 1]: Position of magnets at current point
    %rOld [6 x 1]: Position of magnets at previous point
    %Ud [8 x 1]: Desired Field 
    
    mu0 = 4*pi*1e-7; % air magnetic permeability

    B = zeros(3, 1);
    BOld = zeros(3, 1);
    
    rOld = reshape(rOld, [3,2]);
    r = reshape(r, [3,2]);

    mOld = reshape(mOld, [3,2]);
    m = reshape(m, [3,2]);

    for i = 1:2
        ro = norm(rOld(:,i));
        BOld = BOld + mu0/(4*pi*(ro^3))*(3*(rOld(:,i)/ro)*(rOld(:,i)/ro).' - eye(3))*mOld(:,i);   
    end
    
    for i = 1:2
        ro = norm(r(:,i));
        B = B + mu0/(4*pi*(ro^3))*(3*(rOld(:,i)/ro)*(rOld(:,i)/ro).' - eye(3))*m(:,i);   
    end

    %Dirivative with respect to path (taking step to be 1)
    %Bdir = abs(B(1) - BOld(1));
    Bdir = 0;
    %Error we want to minimize
    Bmin = ((Bdir) + norm(Ud(1:3) - B));
end