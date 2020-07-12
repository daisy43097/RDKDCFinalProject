function result = ur5gradientdescent(gdesired, K, ur5)
% ur5 gradient descent algorithm using the transpose of the Jacobian

    %% instantiations
    Tstep0 = .2; Tstep = Tstep0;
    angle_thresh = pi/180; %radians
    pos_thresh = 0.035; % mm
    sing_thresh = 0.05;
    dzlim = 0.001;
    offset = [-pi/2; -pi/2; 0; -pi/2; 0; 0];
    
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
  
        gst = ur5FwdKin(q);
%         if norm(gst(1:3,4)-pdesired) < 0.1 
%             Tstep = 1.5*max(norm(gst(1:3,4)- pdesired));
% %             Tstep = max([Tstep Tstep0]);
%         else
%             Tstep = Tstep0;
%         end
%         
        % measure distance from goal state
        xi = getXi(invgdesired*gst);
        
        % prepare RR control parameters
        Jbst = ur5BodyJacobian(q);
        transJbst = Jbst';
        
        % check for singularity
        singularity_check = manipulability(Jbst,'invcond');
        if abs(singularity_check) < sing_thresh
            disp(singularity_check)
            result = -1;
            return;
        end
        
        % update joints
        q = q - K * Tstep * transJbst * xi + offset;
        
        % check to see if we are going to puncture 
        gt = ur5FwdKin(q - offset);
        if gst(3,3) > gdesired(3,3) - dzlim
            result = -1;
            disp("Next stemp was going to go too far in the z-axis: Breaking..."); 
            return;
        end
                
        disp("Moving joints to")
        disp(q')
        ur5.move_joints(q, 2);
        pause(3);

    end
    warning('on','all')
    %% Error analysis
    q = ur5.get_current_joints();
    gst = ur5FwdKin(q);
    p = gst(1:3,4);
    
    result = norm(p-pdesired)/10; % cm
    
end

