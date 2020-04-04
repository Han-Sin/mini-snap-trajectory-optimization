function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    %
    %
    %
    %
    for k = 0:3
        Aeq_start(k+1,1:n_order+1) = aeq(k,0);
    end
    beq_start =start_cond';
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    %
    %
    %
    %
    for k =0:3
        Aeq_end(k+1,end-n_order:end) = aeq(k,ts(end));
    end
    beq_end  = end_cond';
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    %
    %
    %
    %
    for i =1:n_seg-1
        Aeq_wp(i,(i-1)*(n_order+1)+1:(i-1)*(n_order+1)+1+n_order)=aeq(0,ts(i));
        beq_wp(i) = waypoints(i+1);
    end
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    %
    %
    %
    %Î»ÖÃÁ¬Ðø
    for i =1:n_seg-1
        Aeq_con_p(i,(i-1)*(n_order+1)+1:(i-1)*(n_order+1)+1+n_order)=aeq(0,ts(i));
        Aeq_con_p(i,i*(n_order+1)+1:i*(n_order+1)+1+n_order) = -aeq(0,0);
        beq_con_p(i) = 0;
    end
    
    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    %
    %
    %
    %
    for i =1:n_seg-1
        Aeq_con_v(i,(i-1)*(n_order+1)+1:(i-1)*(n_order+1)+1+n_order)=aeq(1,ts(i));
        Aeq_con_v(i,i*(n_order+1)+1:i*(n_order+1)+1+n_order) = -aeq(1,0);
        beq_con_v(i) = 0;
    end
    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    %
    %
    %
    %
    for i =1:n_seg-1
        Aeq_con_a(i,(i-1)*(n_order+1)+1:(i-1)*(n_order+1)+1+n_order)=aeq(2,ts(i));
        Aeq_con_a(i,i*(n_order+1)+1:i*(n_order+1)+1+n_order) = -aeq(2,0);
        beq_con_a(i) = 0;
    end
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    %
    %
    %
    %
    for i =1:n_seg-1
        Aeq_con_j(i,(i-1)*(n_order+1)+1:(i-1)*(n_order+1)+1+n_order)=aeq(3,ts(i));
        Aeq_con_j(i,i*(n_order+1)+1:i*(n_order+1)+1+n_order) = -aeq(3,0);
        beq_con_j(i) = 0;
    end
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end

function a = aeq(k,T)
     a=[];
     for i = 0:7
        if i < k
            a =[a,0];
        else
            a=[a,(factorial(i)/factorial(i-k))*T^(i-k)];   
        end
     end
end

