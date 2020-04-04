function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = [];
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment 
        %
        %
        %
        %
        t = ts(k);
        for i=0:7
            temp =[];
            for j=0:7
                temp =[temp,fij(i,j,t)];
            end
            Q_k = [Q_k;temp];   
        end
        Q = blkdiag(Q, Q_k);
    end
end

function f = fij(i,j,ts)
    if (i<4||j<4)
        f=0;
    else
        f=i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*ts^(i+j-7)/(i+j-7);
    end
end
