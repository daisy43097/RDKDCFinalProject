function result = ur5RRcontrol(gdesired, K, ur5)
    %% instantiations
    Tstep = 0.2;
    angle_thresh = pi/180; %radians
    pos_thresh = 0.01; % mm
    sing_thresh = 0.005;
     %offset = [-pi/2; -pi/2; 0; -pi/2; 0; 0];
    offset = zeros(6,1);
    % desired positions
    pdesired = gdesired(1:3,4);
    invgdesired = FINV(gdesired);
   
    %% RRcontrol
    q = ur5.get_current_joints();
    gst = ur5FwdKin(q);
    xi = getXi(invgdesired*gst);
    warning('off','all');
    while norm(xi(4:6)) > angle_thresh || norm(xi(1:3)) > pos_thresh
        
        % read current state
        q = ur5.get_current_joints() - offset;
        gk = ur5FwdKin(q);
%         Tstep = 0.8*max(norm(gk(1:3,4)- pdesired));
        gst = ur5FwdKin(q);
        
        % measure distance from goal state
        xi = getXi(invgdesired*gst);
        
        % prepare RR control parameters
        Jbst = ur5BodyJacobian(q);
        invJbst = pinv(Jbst);
        
        % check for singularity
        singularity_check = manipulability(Jbst,'invcond');
        if abs(singularity_check) < sing_thresh
            disp(singularity_check)
            result = -1;
            return;
        end
        
        % update joints
        q = q - K * Tstep * invJbst * xi + offset;
        ur5.move_joints(q, 5);
        pause(3);

    end
    
    %% Error analysis
    q = ur5.get_current_joints();
    gst = ur5FwdKin(q);
    p = gst(1:3,4);
    
    result = norm(p-pdesired)/10; % cm
    
end

