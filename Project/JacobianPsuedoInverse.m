function dtheta = JacobianPsuedoInverse(ds, J)
    J = double(J);
    J_inv = pinv(J); % use the moore penrose psuedo inverse
    dtheta = J_inv * ds;
    % say dtheta ~ ddtheta
%     ddtheta = dtheta;
end