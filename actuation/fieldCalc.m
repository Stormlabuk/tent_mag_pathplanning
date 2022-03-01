function B = fieldCalc(p, phi, psi)
    %Function to calculate the field for given magnet positions

    %p is position of magnet
    mu0 = 4*pi*(1e-7);  %permeability of free space
    m = [970.1;0;0];
    B = [0;0;0;0;0;0];
    
    for a = 1:1:2
        B(a,:) = (mu0/(4*pi*(norm(p(a,:)))^3)) * (3* p/norm(p(a,:)) * p/norm(p(a,:))' - eye(3))* rotz(phi(a)) * roty(psi(a)) * m;
    end

end