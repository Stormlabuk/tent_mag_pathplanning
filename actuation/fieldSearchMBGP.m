function [UFinal, XFinal] = fieldSearchMBGP(Ud)
%Uses both Gradient & Focal Point Planning (Michael) + Jacobian Descent Solution Method (Giovanni)

rmpath(genpath('functions'))
addpath(genpath('functions'))
%Supressing warnings about Deficient Matrix Rank
warning('off','MATLAB:rankDeficientMatrix')


% Magnetics
mu = 970.1; % EPMs magnetization
mu0 = 4*pi*1e-7; % air magnetic permeability

% Control
step = 1e-5; % derivative difference
K = 1e-2; % primary task gain
numIt = 500; % max iterations to convergence
k0 = 1e-3; % secondary task gain

XFinal = zeros([12,1]);
UFinal = zeros([10,1]);
errFinal = zeros([1,1]);


%Fail counters
nanErr = 0;
muErr = 0;
xbigErr = 0;
xsmallErr = 0;
mag2closeErr = 0;
totErr = 0;
failFPinv = 0;
fpOutErr = 0;


if (norm(Ud(4:8)) ~= 0)   %checking asking for a gradient
    %Calculating parameters for gradient control
    X = magPosition_alphas(Ud(4:8)');
    %Ucal(:,a) = field(X);
    Ucal = field(X);
    
    %Finding the focal point
    UFocal = [Ud(1:3) - Ucal(1:3) ;Ucal(4:8)];
    [p0,errorFP] = focalPoint(UFocal);     %Desired Field with found gradient
    failFPinv = failFPinv + errorFP;
    
    %Adding focal Point
    Xfp = X;
    Xfp(1:3) = Xfp(1:3) - p0;
    Xfp(7:9) = Xfp(7:9) - p0;
    
    %Caclulating Error
    UcalFP(:) = field(Xfp);
    Uerr = norm(Ud- Ucal);
    UerrFP = norm(Ud- UcalFP(:));
    
    %Checking if either of the magnets are too close to center  
    if norm(X(1:3)) < 0.1 ||  norm(Xfp(1:3)) < 0.1 || norm(X(7:9)) < 0.1 ||  norm(Xfp(7:9)) < 0.1    
        %fpOutErr = fpOutErr + 1;
        
        %Use Jacobian Descent
        if Uerr < UerrFP
           %If error without FP is smaller use that 
            [XFinal(:), UFinal(:), errFinal] = fieldControl(Ud, X, K, numIt, step, k0);
        else
            [XFinal(:), UFinal(:), errFinal] = fieldControl(Ud, Xfp, K, numIt, step, k0);
        end
    
        
    %Check if magnets are too close
    elseif norm(X(1:3) - X(7:9)) < 0.2 || norm(Xfp(1:3) - Xfp(7:9)) < 0.2 
        %mag2closeErr = mag2closeErr + 1;
        
        %Use Jacobian Descent
        if Uerr < UerrFP
           %If error without FP is smaller use that 
            [XFinal(:), UFinal(:,a), errFinal] = fieldControl(Ud, X, K, numIt, step, k0);
        else
            [XFinal(:), UFinal(:), errFinal] = fieldControl(Ud, Xfp, K, numIt, step, k0);
        end
        
        
    %If norm(mu) is not right send to Jac Descent
    elseif norm(Xfp(4:6)) < 969.1 || norm(Xfp(10:12)) < 969.1 || norm(Xfp(4:6)) > 971.1 || norm(Xfp(10:12)) > 971.1
        %Use Jacobian Descent
        if Uerr < UerrFP
           %If error without FP is smaller use that 
            [XFinal(:), UFinal(:), errFinal] = fieldControl(Ud, X, K, numIt, step, k0);
        else
            [XFinal(:), UFinal(:), errFinal] = fieldControl(Ud, Xfp, K, numIt, step, k0);
        end
    
    %Check Error between desired and calculated field
    elseif UerrFP > 0.1 || Uerr > 0.1
        %Use Jacobian Descent
        if Uerr < UerrFP
           %If error without FP is smaller use that 
            [XFinal(:), UFinal(:), errFinal] = fieldControl(Ud, X, K, numIt, step, k0);
        else
            [XFinal(:), UFinal(:), errFinal] = fieldControl(Ud, Xfp, K, numIt, step, k0);
        end
        
    
        
    else
    %Just use Gradient Method,Store result with least error
        if Uerr < UerrFP
           %If error without FP is smaller use that 
            XFinal(:) = X;
            UFinal(:) = Ucal;
            errFinal = Uerr;
        else
            XFinal(:) = Xfp;
            UFinal(:) = UcalFP(:);
            errFinal = UerrFP;
        end 
    
    end

else  %If only asking for field

    %Estimating an Initial Position to start search from
    x1 = [1; 0; 0];
    x2 = -x1;
    id = eye(6);
    
    dU_dm = [];
    for i = 1:6
        m1 = mu*id(1:3, i);
        m2 = mu*id(4:6, i);
        X = [x1; m1; x2; m2];
    
        dU_dm = [dU_dm, field_2(mu0, X)];
    end
    
    m = dU_dm\Ud(1:end - 2);
    ro1 = (mu/norm(m(1:3)))^(1/3) / 10;     %Added div(10) When doing planner (wasnt in Gio's code), not sure why but it works
    ro2 = (mu/norm(m(1:3)))^(1/3) / 10;
    m(1:3) = m(1:3)/norm(m(1:3))*mu;
    m(4:6) = m(4:6)/norm(m(4:6))*mu;
    x1 = ro1*x1;
    x2 = ro2*x2;
    x = [x1; m(1:3); x2; m(4:6)];
    
    % Constants
    Id = eye(length(x));
    
    % Initial Conditions
    X(:, 1) = x;
    [XFinal(:), UFinal(:), errFinal] = fieldControl(Ud, X, K, numIt, step, k0);
end

if(isreal(XFinal) == 0)
    XFinal = real(XFinal);
end

if(isreal(UFinal) == 0)
    UFinal = real(UFinal);
end

end