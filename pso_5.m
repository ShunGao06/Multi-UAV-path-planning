%           Particle Swarm Optimization Simulation
%           PSO 1 all design variables are randomized based on cp locations


clc     ;
clear
temp_fitness = 100000 ;

for run = 1:5
    
    n           = 10            ;   % Size of the swarm " no of birds "
    bird_step   = 100          ;   % Maximum number of "birds steps"
    n_uav       = 2             ;   % a number of UAVs
    Alti_Lim    = 26            ;   % altitude limitation for UAVs
    s_Dia    = 3    ;   % Sensor half_diameter
    Clear    = 1    ;   % clearance
    Curve_R  = 60   ;   % curvature radius
    vel      = 200  ;   % constant uav velocity km/h  

    a1          = 10             ;   % line crossing
    a2          = 50            ;   % unvisited points
    a3          = 50             ;   % curve length and revisit time
    a4          = 50            ;   % surface clearance
    
    c1          = 2.05          ;   % pso parameter C1 "particle memory influence"
    c2          = 2.05          ;   % pso parameter C2 "swarm influence"
    w_i         = 0.6           ;   % pso momentum or inertia - inital
    w_e         = 0.2           ;   % pso momentum or inertia - final
    dt          = 10            ;   % delta time

    fq          = 10             ;   % frequency for NN application
    amp         = 0.5           ;   % the amplitude of mutation
    
    %  checkpoints - cp
    load('checkpoint_1.mat') ;
    % terrain data
    load('rural_1.mat') ;
    tx = cp(:,1) ;
    ty = cp(:,2) ;
    tz = cp(:,3) ;
    
    % clustering control points
    [center,U] = fcm([tx ty], n_uav)        ;
    maxU = max(U)                           ;
    
    if n_uav==2
        index1 = find(U(1,:) == maxU)          ;
        index2 = find(U(2,:) == maxU)          ;
        
        % ACO process
        data_1 = [tx(index1) ty(index1)] ;
        tour_1 = aco(data_1)        ;
        % [tour_1, cost_1] = aco(data_1);
        cp_1_reg = data_1(tour_1,:) ;
        temp_x_1 = cp_1_reg(:,1);
        temp_y_1 = cp_1_reg(:,2);
        
        data_2 = [tx(index2) ty(index2)] ;
        tour_2 = aco(data_2)        ;
        cp_2_reg = data_2(tour_2,:) ;
        temp_x_2 = cp_2_reg(:,1);
        temp_y_2 = cp_2_reg(:,2);
        
    elseif n_uav==4
        index1 = find(U(1,:) == maxU)          ;
        index2 = find(U(2,:) == maxU)          ;
        index3 = find(U(3,:) == maxU)          ;
        index4 = find(U(4,:) == maxU)          ;
        
        data_1 = [tx(index1) ty(index1)] ;
        tour_1 = aco(data_1)        ;
        cp_1_reg = data_1(tour_1,:) ;
        temp_x_1 = cp_1_reg(:,1);
        temp_y_1 = cp_1_reg(:,2);
        
        data_2 = [tx(index2) ty(index2)] ;
        tour_2 = aco(data_2)        ;
        cp_2_reg = data_2(tour_2,:) ;
        temp_x_2 = cp_2_reg(:,1);
        temp_y_2 = cp_2_reg(:,2);
        
        data_3 = [tx(index3) ty(index3)] ;
        tour_3 = aco(data_3)        ;
        cp_3_reg = data_3(tour_3,:) ;
        temp_x_3 = cp_3_reg(:,1);
        temp_y_3 = cp_3_reg(:,2);
        
        data_4 = [tx(index4) ty(index4)] ;
        tour_4 = aco(data_4)        ;
        cp_4_reg = data_4(tour_4,:) ;
        temp_x_4 = cp_4_reg(:,1);
        temp_y_4 = cp_4_reg(:,2);
        
    else
        index1 = find(U(1,:) == maxU)          ;
        index2 = find(U(2,:) == maxU)          ;
        index3 = find(U(3,:) == maxU)          ;
        index4 = find(U(4,:) == maxU)          ;
        index5 = find(U(5,:) == maxU)          ;
        index6 = find(U(6,:) == maxU)          ;
        
        data_1 = [tx(index1) ty(index1)] ;
        tour_1 = aco(data_1)        ;
        cp_1_reg = data_1(tour_1,:) ;
        temp_x_1 = cp_1_reg(:,1);
        temp_y_1 = cp_1_reg(:,2);
        
        data_2 = [tx(index2) ty(index2)] ;
        tour_2 = aco(data_2)        ;
        cp_2_reg = data_2(tour_2,:) ;
        temp_x_2 = cp_2_reg(:,1);
        temp_y_2 = cp_2_reg(:,2);
        
        data_3 = [tx(index3) ty(index3)] ;
        tour_3 = aco(data_3)        ;
        cp_3_reg = data_3(tour_3,:) ;
        temp_x_3 = cp_3_reg(:,1);
        temp_y_3 = cp_3_reg(:,2);
        
        data_4 = [tx(index4) ty(index4)] ;
        tour_4 = aco(data_4)        ;
        cp_4_reg = data_4(tour_4,:) ;
        temp_x_4 = cp_4_reg(:,1);
        temp_y_4 = cp_4_reg(:,2);

        data_5 = [tx(index5) ty(index5)] ;
        tour_5 = aco(data_5)        ;
        cp_5_reg = data_5(tour_5,:) ;
        temp_x_5 = cp_5_reg(:,1);
        temp_y_5 = cp_5_reg(:,2);
        
        data_6 = [tx(index6) ty(index6)] ;
        tour_6 = aco(data_6)        ;
        cp_6_reg = data_6(tour_6,:) ;
        temp_x_6 = cp_6_reg(:,1);
        temp_y_6 = cp_6_reg(:,2);
        
    end

    % surface clearance and collosion point determination
    
    % curve points via spline "cscvn"
    current_position_x_1 = [] ;
    current_position_y_1 = [] ;
    current_position_x_2 = [] ;
    current_position_y_2 = [] ;
    current_position_x_3 = [] ;
    current_position_y_3 = [] ;
    current_position_x_4 = [] ;
    current_position_y_4 = [] ;    
    current_position_x_5 = [] ;
    current_position_y_5 = [] ;
    current_position_x_6 = [] ;
    current_position_y_6 = [] ;
    
    cp_reg_update_short_1 = [] ;
    cp_reg_update_short_2 = [] ;
    cp_reg_update_short_3 = [] ;
    cp_reg_update_short_4 = [] ;
    cp_reg_update_short_5 = [] ;
    cp_reg_update_short_6 = [] ;
    
    add_1 = [] ;
    add_2 = [] ;
    add_3 = [] ;
    add_4 = [] ;
    add_5 = [] ;
    add_6 = [] ;
    
    % prediction strategy elements
    
    temp_pun_clear_1_best = 1000 ;
    temp_pun_clear_2_best = 1000 ;    
    temp_pun_clear_3_best = 1000 ;
    temp_pun_clear_4_best = 1000 ;
    temp_pun_clear_5_best = 1000 ;
    temp_pun_clear_6_best = 1000 ;
    
    predict_2x = [] ;
    predict_2y = [] ;    
    predict_4x = [] ;
    predict_4y = [] ;    
    predict_6x = [] ;
    predict_6y = [] ;    
    
    if n_uav ==2
        coll_points_1 = [] ;
        coll_points_2 = [] ;
        
        q1 = length(index1) ;
        q2 = length(index2) ;
        cp_reg_1 = [temp_x_1 temp_y_1] ;
        cp_reg_loop_1 = [cp_reg_1;cp_reg_1(1,:)] ;
        
        xyz_1 = [cp_reg_loop_1(:,1)';cp_reg_loop_1(:,2)';Alti_Lim*ones(1,q1+1)] ;
        points_1 = fnplt_1(cscvn(xyz_1)); % curve points
        
        x_aco_1 = points_1(1,:) ;
        y_aco_1 = points_1(2,:) ;
        z_aco_1 = points_1(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_1 = cp_reg_1(:,1:2) ;
        Nc_1 = 1 ;
        for j = 1:size(z_aco_1,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_1(j)) && x_aco_1(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_1(j))&& y_aco_1(j)<(YY(tt+1,1))
                            if z_aco_1(j)<ZZ(tt,t)
                                coll_points_1(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    
                                else
                                    if coll_points_1(k1-1)-coll_points_1(k1-2) ~= 1
                                        Nc_1 = Nc_1 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_1) ~=1
            data_col_1 = [x_aco_1(coll_points_1)' y_aco_1(coll_points_1)'] ;
            [~,centers_1] = kmeans(data_col_1,Nc_1) ;
            
            k1          = 1 ;
            cp_reg_update_1 = cp_reg_1(:,1:2) ;
            
            for j = 1:size(z_aco_1,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_1(j)) && x_aco_1(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_1(j))&& y_aco_1(j)<(YY(tt+1,1))
                                if z_aco_1(j)<ZZ(tt,t)
                                    coll_points_1(k1) = j ;
                                    distance_to_cc_1 = sqrt((centers_1(:,1)-x_aco_1(j)).^2+(centers_1(:,2)-y_aco_1(j)).^2) ;
                                    [~, order_dist_1] = sort(distance_to_cc_1) ;
                                    c_centers_1(k1) = order_dist_1(1) ;
                                    for j1 = j:length(z_aco_1)
                                        for i1=1:q1
                                            if x_aco_1(j1)==cp_reg_1(i1,1) && y_aco_1(j1)==cp_reg_1(i1,2)
                                                if i1==1
                                                    add_cp_1(k1) = q1 ;
                                                else
                                                    add_cp_1(k1) = i1-1 ;
                                                end
                                                
                                                temp_1 = cp_reg_1(add_cp_1(k1),1:2) ;
                                                for p1 = 1:q1+k1-1
                                                    if cp_reg_update_1(p1,:)==temp_1
                                                        cp_reg_temp_1 = zeros(q1+k1,2) ;
                                                        
                                                        cp_reg_temp_1 = [cp_reg_update_1(1:p1,1:2);centers_1(c_centers_1(k1),1) centers_1(c_centers_1(k1),2);cp_reg_update_1(p1+1:length(cp_reg_update_1),1:2)] ;
                                                        cp_reg_update_1 = cp_reg_temp_1 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_1(1,:) = cp_reg_update_1(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_1)-1
                if cp_reg_update_1(i+1,1)~=cp_reg_update_1(i,1) || cp_reg_update_1(i+1,2)~=cp_reg_update_1(i,2)
                    cp_reg_update_short_1(k1,:) = cp_reg_update_1(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_1)
                if kk>length(cp_reg_1)
                    add_1(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_1(i,1)==cp_reg_1(kk,1) && cp_reg_update_short_1(i,2)==cp_reg_1(kk,2)
                        kk = kk + 1 ;
                    else
                        add_1(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_1 = cp_reg_update_short_1(add_1,1) ;
            current_position_y_1 = cp_reg_update_short_1(add_1,2) ;            
        else
            
        end
        % the second curve
        
        cp_reg_2 = [temp_x_2 temp_y_2] ;
        cp_reg_loop_2 = [cp_reg_2;cp_reg_2(1,:)] ;
        
        xyz_2 = [cp_reg_loop_2(:,1)';cp_reg_loop_2(:,2)';Alti_Lim*ones(1,q2+1)] ;
        points_2 = fnplt_1(cscvn(xyz_2)); % curve points
        
        x_aco_2 = points_2(1,:) ;
        y_aco_2 = points_2(2,:) ;
        z_aco_2 = points_2(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_2 = cp_reg_2(:,1:2) ;
        Nc_2 = 1 ;
        for j = 1:size(z_aco_2,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_2(j)) && x_aco_2(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_2(j))&& y_aco_2(j)<(YY(tt+1,1))
                            if z_aco_2(j)<ZZ(tt,t)
                                coll_points_2(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    
                                else
                                    if coll_points_2(k1-1)-coll_points_2(k1-2) ~= 1
                                        Nc_2 = Nc_2 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_2) ~=1
            
            data_col_2 = [x_aco_2(coll_points_2)' y_aco_2(coll_points_2)'] ;
            [~,centers_2] = kmeans(data_col_2,Nc_2) ;
            
            k1          = 1 ;
            cp_reg_update_2 = cp_reg_2(:,1:2) ;
            
            for j = 1:size(z_aco_2,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_2(j)) && x_aco_2(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_2(j))&& y_aco_2(j)<(YY(tt+1,1))
                                if z_aco_2(j)<ZZ(tt,t)
                                    coll_points_2(k1) = j ;
                                    distance_to_cc_2 = sqrt((centers_2(:,1)-x_aco_2(j)).^2+(centers_2(:,2)-y_aco_2(j)).^2) ;
                                    [~, order_dist_2] = sort(distance_to_cc_2) ;
                                    c_centers_2(k1) = order_dist_2(1) ;
                                    for j1 = j:length(z_aco_2)
                                        for i1=1:q2
                                            if x_aco_2(j1)==cp_reg_2(i1,1) && y_aco_2(j1)==cp_reg_2(i1,2)
                                                if i1==1
                                                    add_cp_2(k1) = q2 ;
                                                else
                                                    add_cp_2(k1) = i1-1 ;
                                                end
                                                
                                                temp_2 = cp_reg_2(add_cp_2(k1),1:2) ;
                                                for p1 = 1:q2+k1-1
                                                    if cp_reg_update_2(p1,:)==temp_2
                                                        cp_reg_temp_2 = zeros(q2+k1,2) ;
                                                        
                                                        cp_reg_temp_2 = [cp_reg_update_2(1:p1,1:2);centers_2(c_centers_2(k1),1) centers_2(c_centers_2(k1),2);cp_reg_update_2(p1+1:length(cp_reg_update_2),1:2)] ;
                                                        cp_reg_update_2 = cp_reg_temp_2 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_2(1,:) = cp_reg_update_2(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_2)-1
                if cp_reg_update_2(i+1,1)~=cp_reg_update_2(i,1) || cp_reg_update_2(i+1,2)~=cp_reg_update_2(i,2)
                    cp_reg_update_short_2(k1,:) = cp_reg_update_2(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_2)
                if kk>length(cp_reg_2)
                    add_2(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_2(i,1)==cp_reg_2(kk,1) && cp_reg_update_short_2(i,2)==cp_reg_2(kk,2)
                        kk = kk + 1 ;
                    else
                        add_2(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_2 = cp_reg_update_short_2(add_2,1) ;
            current_position_y_2 = cp_reg_update_short_2(add_2,2) ;            
        else
        end
        current_position_x_template = [current_position_x_1' current_position_x_2'] ;
        current_position_y_template = [current_position_y_1' current_position_y_2'] ;
        total_dim = length(current_position_x_template) ;
    for i = 1:n
        current_position_x(i,:) = current_position_x_template + current_position_x_template .* (0.5-rand(1,total_dim))/10 ;
        current_position_y(i,:) = current_position_y_template + current_position_y_template .* (0.5-rand(1,total_dim))/10 ;
    end
        current_position_z(1:n,:) = Alti_Lim * ones(n,total_dim) ;
        
    elseif n_uav ==4
        q1 = length(index1) ;
        q2 = length(index2) ;
        q3 = length(index3) ;
        q4 = length(index4) ;
        coll_points_1 = [] ;
        coll_points_2 = [] ;
        coll_points_3 = [] ;
        coll_points_4 = [] ;
        
        cp_reg_1 = [temp_x_1 temp_y_1] ;
        cp_reg_loop_1 = [cp_reg_1;cp_reg_1(1,:)] ;
        
        xyz_1 = [cp_reg_loop_1(:,1)';cp_reg_loop_1(:,2)';Alti_Lim*ones(1,q1+1)] ;
        points_1 = fnplt_1(cscvn(xyz_1)); % curve points
        
        x_aco_1 = points_1(1,:) ;
        y_aco_1 = points_1(2,:) ;
        z_aco_1 = points_1(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_1 = cp_reg_1(:,1:2) ;
        Nc_1 = 1 ;
        for j = 1:size(z_aco_1,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_1(j)) && x_aco_1(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_1(j))&& y_aco_1(j)<(YY(tt+1,1))
                            if z_aco_1(j)<ZZ(tt,t)
                                coll_points_1(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    
                                else
                                    if coll_points_1(k1-1)-coll_points_1(k1-2) ~= 1
                                        Nc_1 = Nc_1 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_1) ~=1
            
            data_col_1 = [x_aco_1(coll_points_1)' y_aco_1(coll_points_1)'] ;
            [~,centers_1] = kmeans(data_col_1,Nc_1) ;
            
            k1          = 1 ;
            cp_reg_update_1 = cp_reg_1(:,1:2) ;
            
            for j = 1:size(z_aco_1,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_1(j)) && x_aco_1(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_1(j))&& y_aco_1(j)<(YY(tt+1,1))
                                if z_aco_1(j)<ZZ(tt,t)
                                    coll_points_1(k1) = j ;
                                    distance_to_cc_1 = sqrt((centers_1(:,1)-x_aco_1(j)).^2+(centers_1(:,2)-y_aco_1(j)).^2) ;
                                    [~, order_dist_1] = sort(distance_to_cc_1) ;
                                    c_centers_1(k1) = order_dist_1(1) ;
                                    for j1 = j:length(z_aco_1)
                                        for i1=1:q1
                                            if x_aco_1(j1)==cp_reg_1(i1,1) && y_aco_1(j1)==cp_reg_1(i1,2)
                                                if i1==1
                                                    add_cp_1(k1) = q1 ;
                                                else
                                                    add_cp_1(k1) = i1-1 ;
                                                end
                                                
                                                temp_1 = cp_reg_1(add_cp_1(k1),1:2) ;
                                                for p1 = 1:q1+k1-1
                                                    if cp_reg_update_1(p1,:)==temp_1
                                                        cp_reg_temp_1 = zeros(q1+k1,2) ;
                                                        
                                                        cp_reg_temp_1 = [cp_reg_update_1(1:p1,1:2);centers_1(c_centers_1(k1),1) centers_1(c_centers_1(k1),2);cp_reg_update_1(p1+1:length(cp_reg_update_1),1:2)] ;
                                                        cp_reg_update_1 = cp_reg_temp_1 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_1(1,:) = cp_reg_update_1(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_1)-1
                if cp_reg_update_1(i+1,1)~=cp_reg_update_1(i,1) || cp_reg_update_1(i+1,2)~=cp_reg_update_1(i,2)
                    cp_reg_update_short_1(k1,:) = cp_reg_update_1(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_1)
                if kk>length(cp_reg_1)
                    add_1(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_1(i,1)==cp_reg_1(kk,1) && cp_reg_update_short_1(i,2)==cp_reg_1(kk,2)
                        kk = kk + 1 ;
                    else
                        add_1(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_1 = cp_reg_update_short_1(add_1,1) ;
            current_position_y_1 = cp_reg_update_short_1(add_1,2) ;            
        else
        end
        
        % the second curve
        
        cp_reg_2 = [temp_x_2 temp_y_2] ;
        cp_reg_loop_2 = [cp_reg_2;cp_reg_2(1,:)] ;
        
        xyz_2 = [cp_reg_loop_2(:,1)';cp_reg_loop_2(:,2)';Alti_Lim*ones(1,q2+1)] ;
        points_2 = fnplt_1(cscvn(xyz_2)); % curve points
        
        x_aco_2 = points_2(1,:) ;
        y_aco_2 = points_2(2,:) ;
        z_aco_2 = points_2(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_2 = cp_reg_2(:,1:2) ;
        Nc_2 = 1 ;
        for j = 1:size(z_aco_2,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_2(j)) && x_aco_2(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_2(j))&& y_aco_2(j)<(YY(tt+1,1))
                            if z_aco_2(j)<ZZ(tt,t)
                                coll_points_2(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    
                                else
                                    if coll_points_2(k1-1)-coll_points_2(k1-2) ~= 1
                                        Nc_2 = Nc_2 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_2) ~=1
            
            data_col_2 = [x_aco_2(coll_points_2)' y_aco_2(coll_points_2)'] ;
            [~,centers_2] = kmeans(data_col_2,Nc_2) ;
            
            k1          = 1 ;
            cp_reg_update_2 = cp_reg_2(:,1:2) ;
            
            for j = 1:size(z_aco_2,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_2(j)) && x_aco_2(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_2(j))&& y_aco_2(j)<(YY(tt+1,1))
                                if z_aco_2(j)<ZZ(tt,t)
                                    coll_points_2(k1) = j ;
                                    distance_to_cc_2 = sqrt((centers_2(:,1)-x_aco_2(j)).^2+(centers_2(:,2)-y_aco_2(j)).^2) ;
                                    [~, order_dist_2] = sort(distance_to_cc_2) ;
                                    c_centers_2(k1) = order_dist_2(1) ;
                                    for j1 = j:length(z_aco_2)
                                        for i1=1:q2
                                            if x_aco_2(j1)==cp_reg_2(i1,1) && y_aco_2(j1)==cp_reg_2(i1,2)
                                                if i1==1
                                                    add_cp_2(k1) = q2 ;
                                                else
                                                    add_cp_2(k1) = i1-1 ;
                                                end
                                                
                                                temp_2 = cp_reg_2(add_cp_2(k1),1:2) ;
                                                for p1 = 1:q2+k1-1
                                                    if cp_reg_update_2(p1,:)==temp_2
                                                        cp_reg_temp_2 = zeros(q2+k1,2) ;
                                                        
                                                        cp_reg_temp_2 = [cp_reg_update_2(1:p1,1:2);centers_2(c_centers_2(k1),1) centers_2(c_centers_2(k1),2);cp_reg_update_2(p1+1:length(cp_reg_update_2),1:2)] ;
                                                        cp_reg_update_2 = cp_reg_temp_2 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_2(1,:) = cp_reg_update_2(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_2)-1
                if cp_reg_update_2(i+1,1)~=cp_reg_update_2(i,1) || cp_reg_update_2(i+1,2)~=cp_reg_update_2(i,2)
                    cp_reg_update_short_2(k1,:) = cp_reg_update_2(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_2)
                if kk>length(cp_reg_2)
                    add_2(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_2(i,1)==cp_reg_2(kk,1) && cp_reg_update_short_2(i,2)==cp_reg_2(kk,2)
                        kk = kk + 1 ;
                    else
                        add_2(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_2 = cp_reg_update_short_2(add_2,1) ;
            current_position_y_2 = cp_reg_update_short_2(add_2,2) ;            
        else
        end
        
        % the third curve
        cp_reg_3 = [temp_x_3 temp_y_3] ;
        cp_reg_loop_3 = [cp_reg_3;cp_reg_3(1,:)] ;
        
        xyz_3 = [cp_reg_loop_3(:,1)';cp_reg_loop_3(:,2)';Alti_Lim*ones(1,q3+1)] ;
        points_3 = fnplt_1(cscvn(xyz_3)); % curve points
        
        x_aco_3 = points_3(1,:) ;
        y_aco_3 = points_3(2,:) ;
        z_aco_3 = points_3(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_3 = cp_reg_3(:,1:2) ;
        Nc_3 = 1 ;
        for j = 1:size(z_aco_3,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_3(j)) && x_aco_3(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_3(j))&& y_aco_3(j)<(YY(tt+1,1))
                            if z_aco_3(j)<ZZ(tt,t)
                                coll_points_3(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    break
                                else
                                    if coll_points_3(k1-1)-coll_points_3(k1-2) ~= 1
                                        Nc_3 = Nc_3 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_3) ~=1
            
            data_col_3 = [x_aco_3(coll_points_3)' y_aco_3(coll_points_3)'] ;
            [~,centers_3] = kmeans(data_col_3,Nc_3) ;
            
            k1          = 1 ;
            cp_reg_update_3 = cp_reg_3(:,1:2) ;
            
            for j = 1:size(z_aco_3,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_3(j)) && x_aco_3(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_3(j))&& y_aco_3(j)<(YY(tt+1,1))
                                if z_aco_3(j)<ZZ(tt,t)
                                    coll_points_3(k1) = j ;
                                    distance_to_cc_3 = sqrt((centers_3(:,1)-x_aco_3(j)).^2+(centers_3(:,2)-y_aco_3(j)).^2) ;
                                    [~, order_dist_3] = sort(distance_to_cc_3) ;
                                    c_centers_3(k1) = order_dist_3(1) ;
                                    for j1 = j:length(z_aco_3)
                                        for i1=1:q3
                                            if x_aco_3(j1)==cp_reg_3(i1,1) && y_aco_3(j1)==cp_reg_3(i1,2)
                                                if i1==1
                                                    add_cp_3(k1) = q3 ;
                                                else
                                                    add_cp_3(k1) = i1-1 ;
                                                end
                                                
                                                temp_3 = cp_reg_3(add_cp_3(k1),1:2) ;
                                                for p1 = 1:q3+k1-1
                                                    if cp_reg_update_3(p1,:)==temp_3
                                                        cp_reg_temp_3 = zeros(q3+k1,2) ;
                                                        
                                                        cp_reg_temp_3 = [cp_reg_update_3(1:p1,1:2);centers_3(c_centers_3(k1),1) centers_3(c_centers_3(k1),2);cp_reg_update_3(p1+1:length(cp_reg_update_3),1:2)] ;
                                                        cp_reg_update_3 = cp_reg_temp_3 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_3(1,:) = cp_reg_update_3(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_3)-1
                if cp_reg_update_3(i+1,1)~=cp_reg_update_3(i,1) || cp_reg_update_3(i+1,2)~=cp_reg_update_3(i,2)
                    cp_reg_update_short_3(k1,:) = cp_reg_update_3(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_3)
                if kk>length(cp_reg_3)
                    add_3(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_3(i,1)==cp_reg_3(kk,1) && cp_reg_update_short_3(i,2)==cp_reg_3(kk,2)
                        kk = kk + 1 ;
                    else
                        add_3(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_3 = cp_reg_update_short_3(add_3,1) ;
            current_position_y_3 = cp_reg_update_short_3(add_3,2) ;            
        else
        end
        
        % the fourth curve
        
        cp_reg_4 = [temp_x_4 temp_y_4] ;
        cp_reg_loop_4 = [cp_reg_4;cp_reg_4(1,:)] ;
        
        xyz_4 = [cp_reg_loop_4(:,1)';cp_reg_loop_4(:,2)';Alti_Lim*ones(1,q4+1)] ;
        points_4 = fnplt_1(cscvn(xyz_4)); % curve points
        
        x_aco_4 = points_4(1,:) ;
        y_aco_4 = points_4(2,:) ;
        z_aco_4 = points_4(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_4 = cp_reg_4(:,1:2) ;
        Nc_4 = 1 ;
        for j = 1:size(z_aco_4,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_4(j)) && x_aco_4(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_4(j))&& y_aco_4(j)<(YY(tt+1,1))
                            if z_aco_4(j)<ZZ(tt,t)
                                coll_points_4(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    break
                                else
                                    if coll_points_4(k1-1)-coll_points_4(k1-2) ~= 1
                                        Nc_4 = Nc_4 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_4) ~=1
            
            data_col_4 = [x_aco_4(coll_points_4)' y_aco_4(coll_points_4)'] ;
            [~,centers_4] = kmeans(data_col_4,Nc_4) ;
            
            k1          = 1 ;
            cp_reg_update_4 = cp_reg_4(:,1:2) ;
            
            for j = 1:size(z_aco_4,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_4(j)) && x_aco_4(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_4(j))&& y_aco_4(j)<(YY(tt+1,1))
                                if z_aco_4(j)<ZZ(tt,t)
                                    coll_points_4(k1) = j ;
                                    distance_to_cc_4 = sqrt((centers_4(:,1)-x_aco_4(j)).^2+(centers_4(:,2)-y_aco_4(j)).^2) ;
                                    [~, order_dist_4] = sort(distance_to_cc_4) ;
                                    c_centers_4(k1) = order_dist_4(1) ;
                                    for j1 = j:length(z_aco_4)
                                        for i1=1:q4
                                            if x_aco_4(j1)==cp_reg_4(i1,1) && y_aco_4(j1)==cp_reg_4(i1,2)
                                                if i1==1
                                                    add_cp_4(k1) = q4 ;
                                                else
                                                    add_cp_4(k1) = i1-1 ;
                                                end
                                                
                                                temp_4 = cp_reg_4(add_cp_4(k1),1:2) ;
                                                for p1 = 1:q4+k1-1
                                                    if cp_reg_update_4(p1,:)==temp_4
                                                        cp_reg_temp_4 = zeros(q4+k1,2) ;
                                                        
                                                        cp_reg_temp_4 = [cp_reg_update_4(1:p1,1:2);centers_4(c_centers_4(k1),1) centers_4(c_centers_4(k1),2);cp_reg_update_4(p1+1:length(cp_reg_update_4),1:2)] ;
                                                        cp_reg_update_4 = cp_reg_temp_4 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_4(1,:) = cp_reg_update_4(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_4)-1
                if cp_reg_update_4(i+1,1)~=cp_reg_update_4(i,1) || cp_reg_update_4(i+1,2)~=cp_reg_update_4(i,2)
                    cp_reg_update_short_4(k1,:) = cp_reg_update_4(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_4)
                if kk>length(cp_reg_4)
                    add_4(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_4(i,1)==cp_reg_4(kk,1) && cp_reg_update_short_4(i,2)==cp_reg_4(kk,2)
                        kk = kk + 1 ;
                    else
                        add_4(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_4 = cp_reg_update_short_4(add_4,1) ;
            current_position_y_4 = cp_reg_update_short_4(add_4,2) ;            
        else
        end
        current_position_x_template = [current_position_x_1' current_position_x_2' current_position_x_3' current_position_x_4'] ;
        current_position_y_template = [current_position_y_1' current_position_y_2' current_position_y_3' current_position_y_4'] ;
        total_dim = length(current_position_x_template) ;
    for i = 1:n
        current_position_x(i,:) = current_position_x_template + current_position_x_template .* (0.5-rand(1,total_dim))/10 ;
        current_position_y(i,:) = current_position_y_template + current_position_y_template .* (0.5-rand(1,total_dim))/10 ;
    end
        current_position_z(1:n,:) = Alti_Lim * ones(n,total_dim) ;        
    else
        q1 = length(index1) ;
        q2 = length(index2) ;
        q3 = length(index3) ;
        q4 = length(index4) ;
        q5 = length(index5) ;
        q6 = length(index6) ;
        
        coll_points_1 = [] ;
        coll_points_2 = [] ;
        coll_points_3 = [] ;
        coll_points_4 = [] ;
        coll_points_5 = [] ;
        coll_points_6 = [] ;
        
        cp_reg_1 = [temp_x_1 temp_y_1] ;
        cp_reg_loop_1 = [cp_reg_1;cp_reg_1(1,:)] ;
        
        xyz_1 = [cp_reg_loop_1(:,1)';cp_reg_loop_1(:,2)';Alti_Lim*ones(1,q1+1)] ;
        points_1 = fnplt_1(cscvn(xyz_1)); % curve points
        
        x_aco_1 = points_1(1,:) ;
        y_aco_1 = points_1(2,:) ;
        z_aco_1 = points_1(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_1 = cp_reg_1(:,1:2) ;
        Nc_1 = 1 ;
        for j = 1:size(z_aco_1,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_1(j)) && x_aco_1(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_1(j))&& y_aco_1(j)<(YY(tt+1,1))
                            if z_aco_1(j)<ZZ(tt,t)
                                coll_points_1(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    
                                else
                                    if coll_points_1(k1-1)-coll_points_1(k1-2) ~= 1
                                        Nc_1 = Nc_1 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_1) ~=1
            
            data_col_1 = [x_aco_1(coll_points_1)' y_aco_1(coll_points_1)'] ;
            [~,centers_1] = kmeans(data_col_1,Nc_1) ;
            
            k1          = 1 ;
            cp_reg_update_1 = cp_reg_1(:,1:2) ;
            
            for j = 1:size(z_aco_1,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_1(j)) && x_aco_1(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_1(j))&& y_aco_1(j)<(YY(tt+1,1))
                                if z_aco_1(j)<ZZ(tt,t)
                                    coll_points_1(k1) = j ;
                                    distance_to_cc_1 = sqrt((centers_1(:,1)-x_aco_1(j)).^2+(centers_1(:,2)-y_aco_1(j)).^2) ;
                                    [~, order_dist_1] = sort(distance_to_cc_1) ;
                                    c_centers_1(k1) = order_dist_1(1) ;
                                    for j1 = j:length(z_aco_1)
                                        for i1=1:q1
                                            if x_aco_1(j1)==cp_reg_1(i1,1) && y_aco_1(j1)==cp_reg_1(i1,2)
                                                if i1==1
                                                    add_cp_1(k1) = q1 ;
                                                else
                                                    add_cp_1(k1) = i1-1 ;
                                                end
                                                
                                                temp_1 = cp_reg_1(add_cp_1(k1),1:2) ;
                                                for p1 = 1:q1+k1-1
                                                    if cp_reg_update_1(p1,:)==temp_1
                                                        cp_reg_temp_1 = zeros(q1+k1,2) ;
                                                        
                                                        cp_reg_temp_1 = [cp_reg_update_1(1:p1,1:2);centers_1(c_centers_1(k1),1) centers_1(c_centers_1(k1),2);cp_reg_update_1(p1+1:length(cp_reg_update_1),1:2)] ;
                                                        cp_reg_update_1 = cp_reg_temp_1 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_1(1,:) = cp_reg_update_1(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_1)-1
                if cp_reg_update_1(i+1,1)~=cp_reg_update_1(i,1) || cp_reg_update_1(i+1,2)~=cp_reg_update_1(i,2)
                    cp_reg_update_short_1(k1,:) = cp_reg_update_1(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_1)
                if kk>length(cp_reg_1)
                    add_1(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_1(i,1)==cp_reg_1(kk,1) && cp_reg_update_short_1(i,2)==cp_reg_1(kk,2)
                        kk = kk + 1 ;
                    else
                        add_1(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_1 = cp_reg_update_short_1(add_1,1) ;
            current_position_y_1 = cp_reg_update_short_1(add_1,2) ;            
        else
        end
        
        % the second curve
        
        cp_reg_2 = [temp_x_2 temp_y_2] ;
        cp_reg_loop_2 = [cp_reg_2;cp_reg_2(1,:)] ;
        
        xyz_2 = [cp_reg_loop_2(:,1)';cp_reg_loop_2(:,2)';Alti_Lim*ones(1,q2+1)] ;
        points_2 = fnplt_1(cscvn(xyz_2)); % curve points
        
        x_aco_2 = points_2(1,:) ;
        y_aco_2 = points_2(2,:) ;
        z_aco_2 = points_2(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_2 = cp_reg_2(:,1:2) ;
        Nc_2 = 1 ;
        for j = 1:size(z_aco_2,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_2(j)) && x_aco_2(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_2(j))&& y_aco_2(j)<(YY(tt+1,1))
                            if z_aco_2(j)<ZZ(tt,t)
                                coll_points_2(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    
                                else
                                    if coll_points_2(k1-1)-coll_points_2(k1-2) ~= 1
                                        Nc_2 = Nc_2 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_2) ~=1
            
            data_col_2 = [x_aco_2(coll_points_2)' y_aco_2(coll_points_2)'] ;
            [~,centers_2] = kmeans(data_col_2,Nc_2) ;
            
            k1          = 1 ;
            cp_reg_update_2 = cp_reg_2(:,1:2) ;
            
            for j = 1:size(z_aco_2,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_2(j)) && x_aco_2(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_2(j))&& y_aco_2(j)<(YY(tt+1,1))
                                if z_aco_2(j)<ZZ(tt,t)
                                    coll_points_2(k1) = j ;
                                    distance_to_cc_2 = sqrt((centers_2(:,1)-x_aco_2(j)).^2+(centers_2(:,2)-y_aco_2(j)).^2) ;
                                    [~, order_dist_2] = sort(distance_to_cc_2) ;
                                    c_centers_2(k1) = order_dist_2(1) ;
                                    for j1 = j:length(z_aco_2)
                                        for i1=1:q2
                                            if x_aco_2(j1)==cp_reg_2(i1,1) && y_aco_2(j1)==cp_reg_2(i1,2)
                                                if i1==1
                                                    add_cp_2(k1) = q2 ;
                                                else
                                                    add_cp_2(k1) = i1-1 ;
                                                end
                                                
                                                temp_2 = cp_reg_2(add_cp_2(k1),1:2) ;
                                                for p1 = 1:q2+k1-1
                                                    if cp_reg_update_2(p1,:)==temp_2
                                                        cp_reg_temp_2 = zeros(q2+k1,2) ;
                                                        
                                                        cp_reg_temp_2 = [cp_reg_update_2(1:p1,1:2);centers_2(c_centers_2(k1),1) centers_2(c_centers_2(k1),2);cp_reg_update_2(p1+1:length(cp_reg_update_2),1:2)] ;
                                                        cp_reg_update_2 = cp_reg_temp_2 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_2(1,:) = cp_reg_update_2(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_2)-1
                if cp_reg_update_2(i+1,1)~=cp_reg_update_2(i,1) || cp_reg_update_2(i+1,2)~=cp_reg_update_2(i,2)
                    cp_reg_update_short_2(k1,:) = cp_reg_update_2(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_2)
                if kk>length(cp_reg_2)
                    add_2(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_2(i,1)==cp_reg_2(kk,1) && cp_reg_update_short_2(i,2)==cp_reg_2(kk,2)
                        kk = kk + 1 ;
                    else
                        add_2(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_2 = cp_reg_update_short_2(add_2,1) ;
            current_position_y_2 = cp_reg_update_short_2(add_2,2) ;            
        else
        end
        
        % the third curve
        cp_reg_3 = [temp_x_3 temp_y_3] ;
        cp_reg_loop_3 = [cp_reg_3;cp_reg_3(1,:)] ;
        
        xyz_3 = [cp_reg_loop_3(:,1)';cp_reg_loop_3(:,2)';Alti_Lim*ones(1,q3+1)] ;
        points_3 = fnplt_1(cscvn(xyz_3)); % curve points
        
        x_aco_3 = points_3(1,:) ;
        y_aco_3 = points_3(2,:) ;
        z_aco_3 = points_3(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_3 = cp_reg_3(:,1:2) ;
        Nc_3 = 1 ;
        for j = 1:size(z_aco_3,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_3(j)) && x_aco_3(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_3(j))&& y_aco_3(j)<(YY(tt+1,1))
                            if z_aco_3(j)<ZZ(tt,t)
                                coll_points_3(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    break
                                else
                                    if coll_points_3(k1-1)-coll_points_3(k1-2) ~= 1
                                        Nc_3 = Nc_3 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_3) ~=1
            
            data_col_3 = [x_aco_3(coll_points_3)' y_aco_3(coll_points_3)'] ;
            [~,centers_3] = kmeans(data_col_3,Nc_3) ;
            
            k1          = 1 ;
            cp_reg_update_3 = cp_reg_3(:,1:2) ;
            
            for j = 1:size(z_aco_3,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_3(j)) && x_aco_3(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_3(j))&& y_aco_3(j)<(YY(tt+1,1))
                                if z_aco_3(j)<ZZ(tt,t)
                                    coll_points_3(k1) = j ;
                                    distance_to_cc_3 = sqrt((centers_3(:,1)-x_aco_3(j)).^2+(centers_3(:,2)-y_aco_3(j)).^2) ;
                                    [~, order_dist_3] = sort(distance_to_cc_3) ;
                                    c_centers_3(k1) = order_dist_3(1) ;
                                    for j1 = j:length(z_aco_3)
                                        for i1=1:q3
                                            if x_aco_3(j1)==cp_reg_3(i1,1) && y_aco_3(j1)==cp_reg_3(i1,2)
                                                if i1==1
                                                    add_cp_3(k1) = q3 ;
                                                else
                                                    add_cp_3(k1) = i1-1 ;
                                                end
                                                
                                                temp_3 = cp_reg_3(add_cp_3(k1),1:2) ;
                                                for p1 = 1:q3+k1-1
                                                    if cp_reg_update_3(p1,:)==temp_3
                                                        cp_reg_temp_3 = zeros(q3+k1,2) ;
                                                        
                                                        cp_reg_temp_3 = [cp_reg_update_3(1:p1,1:2);centers_3(c_centers_3(k1),1) centers_3(c_centers_3(k1),2);cp_reg_update_3(p1+1:length(cp_reg_update_3),1:2)] ;
                                                        cp_reg_update_3 = cp_reg_temp_3 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_3(1,:) = cp_reg_update_3(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_3)-1
                if cp_reg_update_3(i+1,1)~=cp_reg_update_3(i,1) || cp_reg_update_3(i+1,2)~=cp_reg_update_3(i,2)
                    cp_reg_update_short_3(k1,:) = cp_reg_update_3(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_3)
                if kk>length(cp_reg_3)
                    add_3(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_3(i,1)==cp_reg_3(kk,1) && cp_reg_update_short_3(i,2)==cp_reg_3(kk,2)
                        kk = kk + 1 ;
                    else
                        add_3(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_3 = cp_reg_update_short_3(add_3,1) ;
            current_position_y_3 = cp_reg_update_short_3(add_3,2) ;            
        else
        end
        
        % the fourth curve
        
        cp_reg_4 = [temp_x_4 temp_y_4] ;
        cp_reg_loop_4 = [cp_reg_4;cp_reg_4(1,:)] ;
        
        xyz_4 = [cp_reg_loop_4(:,1)';cp_reg_loop_4(:,2)';Alti_Lim*ones(1,q4+1)] ;
        points_4 = fnplt_1(cscvn(xyz_4)); % curve points
        
        x_aco_4 = points_4(1,:) ;
        y_aco_4 = points_4(2,:) ;
        z_aco_4 = points_4(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_4 = cp_reg_4(:,1:2) ;
        Nc_4 = 1 ;
        for j = 1:size(z_aco_4,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_4(j)) && x_aco_4(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_4(j))&& y_aco_4(j)<(YY(tt+1,1))
                            if z_aco_4(j)<ZZ(tt,t)
                                coll_points_4(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    break
                                else
                                    if coll_points_4(k1-1)-coll_points_4(k1-2) ~= 1
                                        Nc_4 = Nc_4 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_4) ~=1
            
            data_col_4 = [x_aco_4(coll_points_4)' y_aco_4(coll_points_4)'] ;
            [~,centers_4] = kmeans(data_col_4,Nc_4) ;
            
            k1          = 1 ;
            cp_reg_update_4 = cp_reg_4(:,1:2) ;
            
            for j = 1:size(z_aco_4,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_4(j)) && x_aco_4(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_4(j))&& y_aco_4(j)<(YY(tt+1,1))
                                if z_aco_4(j)<ZZ(tt,t)
                                    coll_points_4(k1) = j ;
                                    distance_to_cc_4 = sqrt((centers_4(:,1)-x_aco_4(j)).^2+(centers_4(:,2)-y_aco_4(j)).^2) ;
                                    [~, order_dist_4] = sort(distance_to_cc_4) ;
                                    c_centers_4(k1) = order_dist_4(1) ;
                                    for j1 = j:length(z_aco_4)
                                        for i1=1:q4
                                            if x_aco_4(j1)==cp_reg_4(i1,1) && y_aco_4(j1)==cp_reg_4(i1,2)
                                                if i1==1
                                                    add_cp_4(k1) = q4 ;
                                                else
                                                    add_cp_4(k1) = i1-1 ;
                                                end
                                                
                                                temp_4 = cp_reg_4(add_cp_4(k1),1:2) ;
                                                for p1 = 1:q4+k1-1
                                                    if cp_reg_update_4(p1,:)==temp_4
                                                        cp_reg_temp_4 = zeros(q4+k1,2) ;
                                                        
                                                        cp_reg_temp_4 = [cp_reg_update_4(1:p1,1:2);centers_4(c_centers_4(k1),1) centers_4(c_centers_4(k1),2);cp_reg_update_4(p1+1:length(cp_reg_update_4),1:2)] ;
                                                        cp_reg_update_4 = cp_reg_temp_4 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_4(1,:) = cp_reg_update_4(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_4)-1
                if cp_reg_update_4(i+1,1)~=cp_reg_update_4(i,1) || cp_reg_update_4(i+1,2)~=cp_reg_update_4(i,2)
                    cp_reg_update_short_4(k1,:) = cp_reg_update_4(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_4)
                if kk>length(cp_reg_4)
                    add_4(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_4(i,1)==cp_reg_4(kk,1) && cp_reg_update_short_4(i,2)==cp_reg_4(kk,2)
                        kk = kk + 1 ;
                    else
                        add_4(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_4 = cp_reg_update_short_4(add_4,1) ;
            current_position_y_4 = cp_reg_update_short_4(add_4,2) ;            
        else
        end
        
        % the fifth curve
        cp_reg_5 = [temp_x_5 temp_y_5] ;
        cp_reg_loop_5 = [cp_reg_5;cp_reg_5(1,:)] ;
        
        xyz_5 = [cp_reg_loop_5(:,1)';cp_reg_loop_5(:,2)';Alti_Lim*ones(1,q5+1)] ;
        points_5 = fnplt_1(cscvn(xyz_5)); % curve points
        
        x_aco_5 = points_5(1,:) ;
        y_aco_5 = points_5(2,:) ;
        z_aco_5 = points_5(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_5 = cp_reg_5(:,1:2) ;
        Nc_5 = 1 ;
        for j = 1:size(z_aco_5,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_5(j)) && x_aco_5(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_5(j))&& y_aco_5(j)<(YY(tt+1,1))
                            if z_aco_5(j)<ZZ(tt,t)
                                coll_points_5(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    
                                else
                                    if coll_points_5(k1-1)-coll_points_5(k1-2) ~= 1
                                        Nc_5 = Nc_5 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_5) ~=1
            
            data_col_5 = [x_aco_5(coll_points_5)' y_aco_5(coll_points_5)'] ;
            [~,centers_5] = kmeans(data_col_5,Nc_5) ;
            
            k1          = 1 ;
            cp_reg_update_5 = cp_reg_5(:,1:2) ;
            
            for j = 1:size(z_aco_5,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_5(j)) && x_aco_5(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_5(j))&& y_aco_5(j)<(YY(tt+1,1))
                                if z_aco_5(j)<ZZ(tt,t)
                                    coll_points_5(k1) = j ;
                                    distance_to_cc_5 = sqrt((centers_5(:,1)-x_aco_5(j)).^2+(centers_5(:,2)-y_aco_5(j)).^2) ;
                                    [~, order_dist_5] = sort(distance_to_cc_5) ;
                                    c_centers_5(k1) = order_dist_5(1) ;
                                    for j1 = j:length(z_aco_5)
                                        for i1=1:q5
                                            if x_aco_5(j1)==cp_reg_5(i1,1) && y_aco_5(j1)==cp_reg_5(i1,2)
                                                if i1==1
                                                    add_cp_5(k1) = q5 ;
                                                else
                                                    add_cp_5(k1) = i1-1 ;
                                                end
                                                
                                                temp_5 = cp_reg_5(add_cp_5(k1),1:2) ;
                                                for p1 = 1:q5+k1-1
                                                    if cp_reg_update_5(p1,:)==temp_5
                                                        cp_reg_temp_5 = zeros(q5+k1,2) ;
                                                        
                                                        cp_reg_temp_5 = [cp_reg_update_5(1:p1,1:2);centers_5(c_centers_5(k1),1) centers_5(c_centers_5(k1),2);cp_reg_update_5(p1+1:length(cp_reg_update_5),1:2)] ;
                                                        cp_reg_update_5 = cp_reg_temp_5 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_5(1,:) = cp_reg_update_5(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_5)-1
                if cp_reg_update_5(i+1,1)~=cp_reg_update_5(i,1) || cp_reg_update_5(i+1,2)~=cp_reg_update_5(i,2)
                    cp_reg_update_short_5(k1,:) = cp_reg_update_5(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_5)
                if kk>length(cp_reg_5)
                    add_5(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_5(i,1)==cp_reg_5(kk,1) && cp_reg_update_short_5(i,2)==cp_reg_5(kk,2)
                        kk = kk + 1 ;
                    else
                        add_5(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_5 = cp_reg_update_short_5(add_5,1) ;
            current_position_y_5 = cp_reg_update_short_5(add_5,2) ;            
        else
        end
        
        % the sixth curve
        
        cp_reg_6 = [temp_x_6 temp_y_6] ;
        cp_reg_loop_6 = [cp_reg_6;cp_reg_6(1,:)] ;
        
        xyz_6 = [cp_reg_loop_6(:,1)';cp_reg_loop_6(:,2)';Alti_Lim*ones(1,q6+1)] ;
        points_6 = fnplt_1(cscvn(xyz_6)); % curve points
        
        x_aco_6 = points_6(1,:) ;
        y_aco_6 = points_6(2,:) ;
        z_aco_6 = points_6(3,:) ;
        
        k1          = 1 ;
        cp_reg_update_6 = cp_reg_6(:,1:2) ;
        Nc_6 = 1 ;
        for j = 1:size(z_aco_6,2)
            flag = 0 ;
            for t = 1:size(XX,2)-1
                if (XX(1,t)<x_aco_6(j)) && x_aco_6(j)<(XX(1,t+1))
                    for tt = 1:size(YY,2)-1
                        if (YY(tt,1)<y_aco_6(j))&& y_aco_6(j)<(YY(tt+1,1))
                            if z_aco_6(j)<ZZ(tt,t)
                                coll_points_6(k1) = j ;
                                k1 = k1 + 1 ;
                                
                                if k1-1==1
                                    
                                else
                                    if coll_points_6(k1-1)-coll_points_6(k1-2) ~= 1
                                        Nc_6 = Nc_6 +1 ;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        % collision point clusters
        if isempty(coll_points_6) ~=1
            
            data_col_6 = [x_aco_6(coll_points_6)' y_aco_6(coll_points_6)'] ;
            [~,centers_6] = kmeans(data_col_6,Nc_6) ;
            
            k1          = 1 ;
            cp_reg_update_6 = cp_reg_6(:,1:2) ;
            
            for j = 1:size(z_aco_6,2)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x_aco_6(j)) && x_aco_6(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y_aco_6(j))&& y_aco_6(j)<(YY(tt+1,1))
                                if z_aco_6(j)<ZZ(tt,t)
                                    coll_points_6(k1) = j ;
                                    distance_to_cc_6 = sqrt((centers_6(:,1)-x_aco_6(j)).^2+(centers_6(:,2)-y_aco_6(j)).^2) ;
                                    [~, order_dist_6] = sort(distance_to_cc_6) ;
                                    c_centers_6(k1) = order_dist_6(1) ;
                                    for j1 = j:length(z_aco_6)
                                        for i1=1:q6
                                            if x_aco_6(j1)==cp_reg_6(i1,1) && y_aco_6(j1)==cp_reg_6(i1,2)
                                                if i1==1
                                                    add_cp_6(k1) = q6 ;
                                                else
                                                    add_cp_6(k1) = i1-1 ;
                                                end
                                                
                                                temp_6 = cp_reg_6(add_cp_6(k1),1:2) ;
                                                for p1 = 1:q6+k1-1
                                                    if cp_reg_update_6(p1,:)==temp_6
                                                        cp_reg_temp_6 = zeros(q6+k1,2) ;
                                                        
                                                        cp_reg_temp_6 = [cp_reg_update_6(1:p1,1:2);centers_6(c_centers_6(k1),1) centers_6(c_centers_6(k1),2);cp_reg_update_6(p1+1:length(cp_reg_update_6),1:2)] ;
                                                        cp_reg_update_6 = cp_reg_temp_6 ;
                                                        break
                                                    end
                                                end
                                                
                                                k1 = k1 + 1 ;
                                                
                                                flag = 1 ;
                                            end
                                            if flag==1
                                                break
                                            end
                                        end
                                        if flag==1
                                            break
                                        end
                                    end
                                    if flag==1
                                        break
                                    end
                                end
                            end
                            if flag==1
                                break
                            end
                        end
                    end
                    if flag==1
                        break
                    end
                end
            end
            
            %   short cp reg updtae
            cp_reg_update_short_6(1,:) = cp_reg_update_6(1,:) ;
            k1 = 2 ;
            for i=1:length(cp_reg_update_6)-1
                if cp_reg_update_6(i+1,1)~=cp_reg_update_6(i,1) || cp_reg_update_6(i+1,2)~=cp_reg_update_6(i,2)
                    cp_reg_update_short_6(k1,:) = cp_reg_update_6(i+1,:);
                    k1 = k1 + 1 ;
                end
            end
            
            % added point determination
            k = 1 ; kk = 1;
            for i=1:length(cp_reg_update_short_6)
                if kk>length(cp_reg_6)
                    add_6(k) = i ; % added point numbers in cp_reg_update_short
                    k = k + 1 ;
                    kk = kk ;
                else
                    if cp_reg_update_short_6(i,1)==cp_reg_6(kk,1) && cp_reg_update_short_6(i,2)==cp_reg_6(kk,2)
                        kk = kk + 1 ;
                    else
                        add_6(k) = i ; % added point numbers in cp_reg_update_short
                        k = k + 1 ;
                        kk = kk ;
                    end
                end
            end
            % design variable vector
            current_position_x_6 = cp_reg_update_short_6(add_6,1) ;
            current_position_y_6 = cp_reg_update_short_6(add_6,2) ;            
        else
        end
        current_position_x_template = [current_position_x_1' current_position_x_2' current_position_x_3' current_position_x_4' current_position_x_5' current_position_x_6'] ;
        current_position_y_template = [current_position_y_1' current_position_y_2' current_position_y_3' current_position_y_4' current_position_y_5' current_position_y_6'] ;
        total_dim = length(current_position_x_template) ;
    for i = 1:n
        current_position_x(i,:) = current_position_x_template + current_position_x_template .* (0.5-rand(1,total_dim))/10 ;
        current_position_y(i,:) = current_position_y_template + current_position_y_template .* (0.5-rand(1,total_dim))/10 ;
    end
        current_position_z(1:n,:) = Alti_Lim * ones(n,total_dim) ;                
    end
    
    % INITIAL POPULATION PRODUCTION
    globl_best_position_x = zeros(n,total_dim) ;
    globl_best_position_y = zeros(n,total_dim) ;
    
    fitness = 0*ones(n,bird_step)   ;   % pre-initial fitness values
    
    %-----------------------------%
    %    initialize the parameter %
    %-----------------------------%
    
    R1 = rand(n,total_dim)                       ;
    R2 = rand(n,total_dim)                       ;
    
    %------------------------------------------------%
    % Initializing swarm and velocities and position %
    %------------------------------------------------%
    % design parameters
    %
    velocity_x            = current_position_x.*rand(n,total_dim)/10 ;
    local_best_position_x = current_position_x                                  ;
    velocity_y            = current_position_y.*rand(n,total_dim)/10 ;
    local_best_position_y = current_position_y                                  ;
    
    %-------------------------------------------%
    %     Evaluate initial population           %
    %-------------------------------------------%
    f_1 = zeros(1,n)             ;   % total distance to checkpoints
    f_2 = zeros(1,n)             ;   % the number of unvisited points
    f_3 = zeros(1,n)             ;   % curve length
    f_4 = zeros(1,n)             ;   % surface clearance
    Ff  = zeros(1,n)             ;   % total fitness value
    
    % curve points via B-spline "cscvn"
    for p=1:n
        % curve points via spline "cscvn"
        if n_uav ==2
            if isempty(cp_reg_update_short_1)~=1
                cp_reg_update_short_1x = cp_reg_update_short_1(:,1)' ;
                cp_reg_update_short_1y = cp_reg_update_short_1(:,2)' ;
                cp_reg_update_short_1z = Alti_Lim * ones(n,length(cp_reg_update_short_1x))     ;
                
                for i=1:length(add_1)
                    cp_reg_update_short_1x(add_1(i)) = current_position_x(p,i);
                    cp_reg_update_short_1y(add_1(i)) = current_position_y(p,i);
                end
                xyz_1 = [cp_reg_update_short_1x;cp_reg_update_short_1y;cp_reg_update_short_1z]; % control points
                xyz_1 = [xyz_1 xyz_1(:,1)] ;
                [points_1,~] = fnplt_1(cscvn(xyz_1)) ;     % curve points
            end
            if isempty(cp_reg_update_short_2)~=1
                cp_reg_update_short_2x = cp_reg_update_short_2(:,1)' ;
                cp_reg_update_short_2y = cp_reg_update_short_2(:,2)' ;
                cp_reg_update_short_2z = Alti_Lim * ones(n,length(cp_reg_update_short_2x))     ;
                
                for i=length(add_1)+1:length(add_1)+length(add_2)
                    j = i - length(add_1) ;
                    cp_reg_update_short_2x(add_2(j)) = current_position_x(p,i);
                    cp_reg_update_short_2y(add_2(j)) = current_position_y(p,i);
                end
                xyz_2 = [cp_reg_update_short_2x;cp_reg_update_short_2y;cp_reg_update_short_2z]; % control points
                xyz_2 = [xyz_2 xyz_2(:,1)] ;
                [points_2,~] = fnplt_1(cscvn(xyz_2)) ;     % curve points
            end
        elseif n_uav ==4
            if isempty(cp_reg_update_short_1)~=1
                cp_reg_update_short_1x = cp_reg_update_short_1(:,1)' ;
                cp_reg_update_short_1y = cp_reg_update_short_1(:,2)' ;
                cp_reg_update_short_1z = Alti_Lim * ones(n,length(cp_reg_update_short_1x))     ;
                
                for i=1:length(add_1)
                    cp_reg_update_short_1x(add_1(i)) = current_position_x(p,i);
                    cp_reg_update_short_1y(add_1(i)) = current_position_y(p,i);
                end
                xyz_1 = [cp_reg_update_short_1x;cp_reg_update_short_1y;cp_reg_update_short_1z]; % control points
                xyz_1 = [xyz_1 xyz_1(:,1)] ;
                [points_1,~] = fnplt_1(cscvn(xyz_1)) ;     % curve points
            end
            if isempty(cp_reg_update_short_2)~=1
                cp_reg_update_short_2x = cp_reg_update_short_2(:,1)' ;
                cp_reg_update_short_2y = cp_reg_update_short_2(:,2)' ;
                cp_reg_update_short_2z = Alti_Lim * ones(n,length(cp_reg_update_short_2x))     ;
                
                for i=length(add_1)+1:length(add_1)+length(add_2)
                    j = i - length(add_1) ;
                    cp_reg_update_short_2x(add_2(j)) = current_position_x(p,i);
                    cp_reg_update_short_2y(add_2(j)) = current_position_y(p,i);
                end
                xyz_2 = [cp_reg_update_short_2x;cp_reg_update_short_2y;cp_reg_update_short_2z]; % control points
                xyz_2 = [xyz_2 xyz_2(:,1)] ;
                [points_2,~] = fnplt_1(cscvn(xyz_2)) ;     % curve points
            end
            if isempty(cp_reg_update_short_3)~=1
                
                cp_reg_update_short_3x = cp_reg_update_short_3(:,1)' ;
                cp_reg_update_short_3y = cp_reg_update_short_3(:,2)' ;
                cp_reg_update_short_3z = Alti_Lim * ones(n,length(cp_reg_update_short_3x))     ;
                
                for i=length(add_1)+length(add_2)+1:length(add_1)+length(add_2)+length(add_3)
                    j = i - length(add_1)-length(add_2) ;
                    cp_reg_update_short_3x(add_3(j)) = current_position_x(p,i);
                    cp_reg_update_short_3y(add_3(j)) = current_position_y(p,i);
                end
                xyz_3 = [cp_reg_update_short_3x;cp_reg_update_short_3y;cp_reg_update_short_3z]; % control points
                xyz_3 = [xyz_3 xyz_3(:,1)] ;
                [points_3,~] = fnplt_1(cscvn(xyz_3)) ;     % curve points
            end
            if isempty(cp_reg_update_short_4)~=1
                cp_reg_update_short_4x = cp_reg_update_short_4(:,1)' ;
                cp_reg_update_short_4y = cp_reg_update_short_4(:,2)' ;
                cp_reg_update_short_4z = Alti_Lim * ones(n,length(cp_reg_update_short_4x))     ;
                
                for i=length(add_1)+length(add_2)+length(add_3)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)
                    j = i - length(add_1)-length(add_2)-length(add_3) ;
                    cp_reg_update_short_4x(add_4(j)) = current_position_x(p,i);
                    cp_reg_update_short_4y(add_4(j)) = current_position_y(p,i);
                end
                xyz_4 = [cp_reg_update_short_4x;cp_reg_update_short_4y;cp_reg_update_short_4z]; % control points
                xyz_4 = [xyz_4 xyz_4(:,1)] ;
                [points_4,~] = fnplt_1(cscvn(xyz_4)) ;     % curve points
            end
        else
            if isempty(cp_reg_update_short_1)~=1
                cp_reg_update_short_1x = cp_reg_update_short_1(:,1)' ;
                cp_reg_update_short_1y = cp_reg_update_short_1(:,2)' ;
                cp_reg_update_short_1z = Alti_Lim * ones(n,length(cp_reg_update_short_1x))     ;
                
                for i=1:length(add_1)
                    cp_reg_update_short_1x(add_1(i)) = current_position_x(p,i);
                    cp_reg_update_short_1y(add_1(i)) = current_position_y(p,i);
                end
                xyz_1 = [cp_reg_update_short_1x;cp_reg_update_short_1y;cp_reg_update_short_1z]; % control points
                xyz_1 = [xyz_1 xyz_1(:,1)] ;
                [points_1,~] = fnplt_1(cscvn(xyz_1)) ;     % curve points
            end
            if isempty(cp_reg_update_short_2)~=1
                cp_reg_update_short_2x = cp_reg_update_short_2(:,1)' ;
                cp_reg_update_short_2y = cp_reg_update_short_2(:,2)' ;
                cp_reg_update_short_2z = Alti_Lim * ones(n,length(cp_reg_update_short_2x))     ;
                
                for i=length(add_1)+1:length(add_1)+length(add_2)
                    j = i - length(add_1) ;
                    cp_reg_update_short_2x(add_2(j)) = current_position_x(p,i);
                    cp_reg_update_short_2y(add_2(j)) = current_position_y(p,i);
                end
                xyz_2 = [cp_reg_update_short_2x;cp_reg_update_short_2y;cp_reg_update_short_2z]; % control points
                xyz_2 = [xyz_2 xyz_2(:,1)] ;
                [points_2,~] = fnplt_1(cscvn(xyz_2)) ;     % curve points
            end
            if isempty(cp_reg_update_short_3)~=1
                
                cp_reg_update_short_3x = cp_reg_update_short_3(:,1)' ;
                cp_reg_update_short_3y = cp_reg_update_short_3(:,2)' ;
                cp_reg_update_short_3z = Alti_Lim * ones(n,length(cp_reg_update_short_3x))     ;
                
                for i=length(add_1)+length(add_2)+1:length(add_1)+length(add_2)+length(add_3)
                    j = i - length(add_1)-length(add_2) ;
                    cp_reg_update_short_3x(add_3(j)) = current_position_x(p,i);
                    cp_reg_update_short_3y(add_3(j)) = current_position_y(p,i);
                end
                xyz_3 = [cp_reg_update_short_3x;cp_reg_update_short_3y;cp_reg_update_short_3z]; % control points
                xyz_3 = [xyz_3 xyz_3(:,1)] ;
                [points_3,~] = fnplt_1(cscvn(xyz_3)) ;     % curve points
            end
            if isempty(cp_reg_update_short_4)~=1
                cp_reg_update_short_4x = cp_reg_update_short_4(:,1)' ;
                cp_reg_update_short_4y = cp_reg_update_short_4(:,2)' ;
                cp_reg_update_short_4z = Alti_Lim * ones(n,length(cp_reg_update_short_4x))     ;
                
                for i=length(add_1)+length(add_2)+length(add_3)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)
                    j = i - length(add_1)-length(add_2)-length(add_3) ;
                    cp_reg_update_short_4x(add_4(j)) = current_position_x(p,i);
                    cp_reg_update_short_4y(add_4(j)) = current_position_y(p,i);
                end
                xyz_4 = [cp_reg_update_short_4x;cp_reg_update_short_4y;cp_reg_update_short_4z]; % control points
                xyz_4 = [xyz_4 xyz_4(:,1)] ;
                [points_4,~] = fnplt_1(cscvn(xyz_4)) ;     % curve points
            end
            if isempty(cp_reg_update_short_5)~=1
                
                cp_reg_update_short_5x = cp_reg_update_short_5(:,1)' ;
                cp_reg_update_short_5y = cp_reg_update_short_5(:,2)' ;
                cp_reg_update_short_5z = Alti_Lim * ones(n,length(cp_reg_update_short_5x))     ;
                
                for i=length(add_1)+length(add_2)+length(add_3)+length(add_4)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)
                    j = i - length(add_1)-length(add_2)-length(add_3)-length(add_4);
                    cp_reg_update_short_5x(add_5(j)) = current_position_x(p,i);
                    cp_reg_update_short_5y(add_5(j)) = current_position_y(p,i);
                end
                xyz_5 = [cp_reg_update_short_5x;cp_reg_update_short_5y;cp_reg_update_short_5z]; % control points
                xyz_5 = [xyz_5 xyz_5(:,1)] ;
                [points_5,~] = fnplt_1(cscvn(xyz_5)) ;     % curve points
            end
            if isempty(cp_reg_update_short_6)~=1
                cp_reg_update_short_6x = cp_reg_update_short_6(:,1)' ;
                cp_reg_update_short_6y = cp_reg_update_short_6(:,2)' ;
                cp_reg_update_short_6z = Alti_Lim * ones(n,length(cp_reg_update_short_6x))     ;
                
                for i=length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)+length(add_6)
                    j = i - length(add_1)-length(add_2)-length(add_3)-length(add_4)-length(add_5);
                    cp_reg_update_short_6x(add_6(j)) = current_position_x(p,i);
                    cp_reg_update_short_6y(add_6(j)) = current_position_y(p,i);
                end
                xyz_6 = [cp_reg_update_short_6x;cp_reg_update_short_6y;cp_reg_update_short_6z]; % control points
                xyz_6 = [xyz_6 xyz_6(:,1)] ;
                [points_6,~] = fnplt_1(cscvn(xyz_6)) ;     % curve points
            end
            
        end
        % fitness function calculations
        
        % intersection points
        if n_uav==2
            [x12, y12] = polyxpoly(points_1(1,:),points_1(2,:),points_2(1,:),points_2(2,:));
            f_1(p) = size(x12,1) ;
        elseif n_uav==4
            [x12, y12] = polyxpoly(points_1(1,:),points_1(2,:),points_2(1,:),points_2(2,:));
            [x13, y13] = polyxpoly(points_1(1,:),points_1(2,:),points_3(1,:),points_3(2,:));
            [x14, y14] = polyxpoly(points_1(1,:),points_1(2,:),points_4(1,:),points_4(2,:));
            [x23, y23] = polyxpoly(points_2(1,:),points_2(2,:),points_3(1,:),points_3(2,:));
            [x24, y24] = polyxpoly(points_2(1,:),points_2(2,:),points_4(1,:),points_4(2,:));
            [x34, y34] = polyxpoly(points_3(1,:),points_3(2,:),points_4(1,:),points_4(2,:));
            f_1(p) = size(x12,1) + size(x13,1)+size(x14,1) + size(x23,1)+size(x24,1) + size(x34,1) ;
        else
            [x12, y12] = polyxpoly(points_1(1,:),points_1(2,:),points_2(1,:),points_2(2,:));
            [x13, y13] = polyxpoly(points_1(1,:),points_1(2,:),points_3(1,:),points_3(2,:));
            [x14, y14] = polyxpoly(points_1(1,:),points_1(2,:),points_4(1,:),points_4(2,:));
            [x15, y15] = polyxpoly(points_1(1,:),points_1(2,:),points_5(1,:),points_5(2,:));
            [x16, y16] = polyxpoly(points_1(1,:),points_1(2,:),points_6(1,:),points_6(2,:));
            [x23, y23] = polyxpoly(points_2(1,:),points_2(2,:),points_3(1,:),points_3(2,:));
            [x24, y24] = polyxpoly(points_2(1,:),points_2(2,:),points_4(1,:),points_4(2,:));
            [x25, y25] = polyxpoly(points_2(1,:),points_2(2,:),points_5(1,:),points_5(2,:));
            [x26, y26] = polyxpoly(points_2(1,:),points_2(2,:),points_6(1,:),points_6(2,:));
            [x34, y34] = polyxpoly(points_3(1,:),points_3(2,:),points_4(1,:),points_4(2,:));
            [x35, y35] = polyxpoly(points_3(1,:),points_3(2,:),points_5(1,:),points_5(2,:));
            [x36, y36] = polyxpoly(points_3(1,:),points_3(2,:),points_6(1,:),points_6(2,:));            
            [x45, y45] = polyxpoly(points_4(1,:),points_4(2,:),points_5(1,:),points_5(2,:));
            [x46, y46] = polyxpoly(points_4(1,:),points_4(2,:),points_6(1,:),points_6(2,:));
            [x56, y56] = polyxpoly(points_5(1,:),points_5(2,:),points_6(1,:),points_6(2,:));            
            
            f_1(p) = size(x12,1) + size(x13,1)+size(x14,1) + size(x15,1)+size(x16,1) + size(x23,1)+...
                size(x24,1) + size(x25,1)+size(x26,1) + size(x34,1)+size(x35,1) + size(x36,1)+...
                size(x45,1)+size(x46,1) + size(x56,1);            
        end
        
        if n_uav ==2
            
            % fitness for the distance to check points & the number of unvisited points
            x = points_1(1,:) ;
            y = points_1(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_1)   ;
            for j = 1:length(temp_x_1)
                d = sqrt((x-temp_x_1(j)).^2+(y-temp_y_1(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_1 = top_dist ;
            temp_points_1 = unvisited_points ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_2)   ;
            for j = 1:length(temp_x_2)
                d = sqrt((x-temp_x_2(j)).^2+(y-temp_y_2(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_2 = top_dist ;
            temp_points_2 = unvisited_points ;
            
%             f_1(p) = temp_dist_1 + temp_dist_2           ;
            f_2(p) = temp_points_1 + temp_points_2   ;
            
        elseif n_uav ==4
            % fitness for the distance to check points & the number of unvisited points
            x = points_1(1,:) ;
            y = points_1(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_1)   ;
            for j = 1:length(temp_x_1)
                d = sqrt((x-temp_x_1(j)).^2+(y-temp_y_1(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_1 = top_dist ;
            temp_points_1 = unvisited_points ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_2)   ;
            for j = 1:length(temp_x_2)
                d = sqrt((x-temp_x_2(j)).^2+(y-temp_y_2(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_2 = top_dist ;
            temp_points_2 = unvisited_points ;

            % fitness for the distance to check points & the number of unvisited points
            x = points_3(1,:) ;
            y = points_3(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_3)   ;
            for j = 1:length(temp_x_3)
                d = sqrt((x-temp_x_3(j)).^2+(y-temp_y_3(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_3 = top_dist ;
            temp_points_3 = unvisited_points ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_4)   ;
            for j = 1:length(temp_x_4)
                d = sqrt((x-temp_x_4(j)).^2+(y-temp_y_4(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_4 = top_dist ;
            temp_points_4 = unvisited_points ;
                        
%             f_1(p) = temp_dist_1 + temp_dist_2 + temp_dist_3 + temp_dist_4          ;
            f_2(p) = temp_points_1 + temp_points_2 + temp_points_3 + temp_points_4   ;
            
        else
            % fitness for the distance to check points & the number of unvisited points
            x = points_1(1,:) ;
            y = points_1(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_1)   ;
            for j = 1:length(temp_x_1)
                d = sqrt((x-temp_x_1(j)).^2+(y-temp_y_1(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_1 = top_dist ;
            temp_points_1 = unvisited_points ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_2)   ;
            for j = 1:length(temp_x_2)
                d = sqrt((x-temp_x_2(j)).^2+(y-temp_y_2(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_2 = top_dist ;
            temp_points_2 = unvisited_points ;

            x = points_3(1,:) ;
            y = points_3(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_3)   ;
            for j = 1:length(temp_x_3)
                d = sqrt((x-temp_x_3(j)).^2+(y-temp_y_3(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_3 = top_dist ;
            temp_points_3 = unvisited_points ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_4)   ;
            for j = 1:length(temp_x_4)
                d = sqrt((x-temp_x_4(j)).^2+(y-temp_y_4(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_4 = top_dist ;
            temp_points_4 = unvisited_points ;
            
            x = points_5(1,:) ;
            y = points_5(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_5)   ;
            for j = 1:length(temp_x_5)
                d = sqrt((x-temp_x_5(j)).^2+(y-temp_y_5(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_5 = top_dist ;
            temp_points_5 = unvisited_points ;
            
            x = points_6(1,:) ;
            y = points_6(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_6)   ;
            for j = 1:length(temp_x_6)
                d = sqrt((x-temp_x_6(j)).^2+(y-temp_y_6(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_6 = top_dist ;
            temp_points_6 = unvisited_points ;
            
%             f_1(p) = temp_dist_1 + temp_dist_2 + temp_dist_3 + temp_dist_4 + temp_dist_5 + temp_dist_6          ;
            f_2(p) = temp_points_1 + temp_points_2 + temp_points_3 + temp_points_4 + temp_points_5 + temp_points_6   ;
            
        end
        
        if n_uav ==2
            % curve length
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(1) = DD / vel ;

            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(2) = DD / vel ;

            
        f_3 (p) = max(revisit) ;
            
        elseif n_uav ==4
            % curve length
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(1) = DD / vel ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(2) = DD / vel ;
            
            x = points_3(1,:) ;
            y = points_3(2,:) ;
            z = points_3(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(3) = DD / vel ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
            z = points_4(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(4) = DD / vel ;
            
            f_3 (p) = max(revisit) ;
               else
            % curve length
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(1) = DD / vel ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(2) = DD / vel ;
            
            x = points_3(1,:) ;
            y = points_3(2,:) ;
            z = points_3(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(3) = DD / vel ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
            z = points_4(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(4) = DD / vel ;

            x = points_5(1,:) ;
            y = points_5(2,:) ;
            z = points_5(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(5) = DD / vel ;
            
            x = points_6(1,:) ;
            y = points_6(2,:) ;
            z = points_6(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(6) = DD / vel ;
            
            f_3 (p) = max(revisit) ;
            
        end
        
        if n_uav ==2
            
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_1 =  pun_clear ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_2 =  pun_clear ;
            
            f_4(p) = temp_pun_clear_1 + temp_pun_clear_2 ;
            
        elseif n_uav ==4
            
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_1 =  pun_clear ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_2 =  pun_clear ;
            
            x = points_3(1,:) ;
            y = points_3(2,:) ;
            z = points_3(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_3 =  pun_clear ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
            z = points_4(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_4 =  pun_clear ;
            
            f_4(p) = temp_pun_clear_1 + temp_pun_clear_2 + temp_pun_clear_3 + temp_pun_clear_4 ;
            
        else
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_1 =  pun_clear ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_2 =  pun_clear ;
            
            x = points_3(1,:) ;
            y = points_3(2,:) ;
            z = points_3(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_3 =  pun_clear ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
            z = points_4(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_4 =  pun_clear ;
            
            x = points_5(1,:) ;
            y = points_5(2,:) ;
            z = points_5(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_5 =  pun_clear ;
            
            x = points_6(1,:) ;
            y = points_6(2,:) ;
            z = points_6(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_6 =  pun_clear ;
            
            f_4(p) = temp_pun_clear_1 + temp_pun_clear_2 + temp_pun_clear_3 + temp_pun_clear_4 + temp_pun_clear_5 + temp_pun_clear_6 ;
            
        end
        
        % curve curvature
        
        % total value
        Ff(p) = a1 * f_1(p) + a2 * f_2(p)+ a3 *f_3(p)+ a4 * f_4(p);
    end
    
    current_fitness         = Ff                                    ;
    local_best_fitness      = current_fitness                       ;
    [global_best_fitness,g] = min(local_best_fitness)               ;
    
    for i = 1:n
        globl_best_position_x(i,:) = local_best_position_x(g,:)         ;
        globl_best_position_y(i,:) = local_best_position_y(g,:)         ;
    end
    
elit_fitness(1)=global_best_fitness;

%-------------------%
%  VELOCITY UPDATE  %
%-------------------%

velocity_x = w_i*velocity_x + c1*(R1.*(local_best_position_x-current_position_x))/dt + c2*(R2.*(globl_best_position_x-current_position_x)/dt) ;
velocity_y = w_i*velocity_y + c1*(R1.*(local_best_position_y-current_position_y))/dt + c2*(R2.*(globl_best_position_y-current_position_y)/dt) ;

%------------------%
%   SWARMUPDATE    %
%------------------%

current_position_x = current_position_x + velocity_x ;
current_position_y = current_position_y + velocity_y ;


%% Main Loop
iter = 0 ;        % Iterations counter
while  ( iter < bird_step )
    iter = iter + 1;
    % curve points via B-spline "cscvn"
    for p=1:n
        % curve points via spline "cscvn"
        if n_uav ==2
            if isempty(cp_reg_update_short_1)~=1
                for i=1:length(add_1)
                    cp_reg_update_short_1x(add_1(i)) = current_position_x(p,i);
                    cp_reg_update_short_1y(add_1(i)) = current_position_y(p,i);
                end
                xyz_1 = [cp_reg_update_short_1x;cp_reg_update_short_1y;cp_reg_update_short_1z]; % control points
                xyz_1 = [xyz_1 xyz_1(:,1)] ;
                [points_1,~] = fnplt_1(cscvn(xyz_1)) ;     % curve points
            end
            if isempty(cp_reg_update_short_2)~=1
                for i=length(add_1)+1:length(add_1)+length(add_2)
                    j = i - length(add_1) ;
                    cp_reg_update_short_2x(add_2(j)) = current_position_x(p,i);
                    cp_reg_update_short_2y(add_2(j)) = current_position_y(p,i);
                end
                xyz_2 = [cp_reg_update_short_2x;cp_reg_update_short_2y;cp_reg_update_short_2z]; % control points
                xyz_2 = [xyz_2 xyz_2(:,1)] ;
                [points_2,~] = fnplt_1(cscvn(xyz_2)) ;     % curve points
            end
        elseif n_uav ==4
            if isempty(cp_reg_update_short_1)~=1
                for i=1:length(add_1)
                    cp_reg_update_short_1x(add_1(i)) = current_position_x(p,i);
                    cp_reg_update_short_1y(add_1(i)) = current_position_y(p,i);
                end
                xyz_1 = [cp_reg_update_short_1x;cp_reg_update_short_1y;cp_reg_update_short_1z]; % control points
                xyz_1 = [xyz_1 xyz_1(:,1)] ;
                [points_1,~] = fnplt_1(cscvn(xyz_1)) ;     % curve points
            end
            if isempty(cp_reg_update_short_2)~=1
                for i=length(add_1)+1:length(add_1)+length(add_2)
                    j = i - length(add_1) ;
                    cp_reg_update_short_2x(add_2(j)) = current_position_x(p,i);
                    cp_reg_update_short_2y(add_2(j)) = current_position_y(p,i);
                end
                xyz_2 = [cp_reg_update_short_2x;cp_reg_update_short_2y;cp_reg_update_short_2z]; % control points
                xyz_2 = [xyz_2 xyz_2(:,1)] ;
                [points_2,~] = fnplt_1(cscvn(xyz_2)) ;     % curve points
            end
            if isempty(cp_reg_update_short_3)~=1
                for i=length(add_1)+length(add_2)+1:length(add_1)+length(add_2)+length(add_3)
                    j = i - length(add_1)-length(add_2) ;
                    cp_reg_update_short_3x(add_3(j)) = current_position_x(p,i);
                    cp_reg_update_short_3y(add_3(j)) = current_position_y(p,i);
                end
                xyz_3 = [cp_reg_update_short_3x;cp_reg_update_short_3y;cp_reg_update_short_3z]; % control points
                xyz_3 = [xyz_3 xyz_3(:,1)] ;
                [points_3,~] = fnplt_1(cscvn(xyz_3)) ;     % curve points
            end
            if isempty(cp_reg_update_short_4)~=1
                for i=length(add_1)+length(add_2)+length(add_3)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)
                    j = i - length(add_1)-length(add_2)-length(add_3) ;
                    cp_reg_update_short_4x(add_4(j)) = current_position_x(p,i);
                    cp_reg_update_short_4y(add_4(j)) = current_position_y(p,i);
                end
                xyz_4 = [cp_reg_update_short_4x;cp_reg_update_short_4y;cp_reg_update_short_4z]; % control points
                xyz_4 = [xyz_4 xyz_4(:,1)] ;
                [points_4,~] = fnplt_1(cscvn(xyz_4)) ;     % curve points
            end
        else
            if isempty(cp_reg_update_short_1)~=1
                
                for i=1:length(add_1)
                    cp_reg_update_short_1x(add_1(i)) = current_position_x(p,i);
                    cp_reg_update_short_1y(add_1(i)) = current_position_y(p,i);
                end
                xyz_1 = [cp_reg_update_short_1x;cp_reg_update_short_1y;cp_reg_update_short_1z]; % control points
                xyz_1 = [xyz_1 xyz_1(:,1)] ;
                [points_1,~] = fnplt_1(cscvn(xyz_1)) ;     % curve points
            end
            if isempty(cp_reg_update_short_2)~=1
                
                for i=length(add_1)+1:length(add_1)+length(add_2)
                    j = i - length(add_1) ;
                    cp_reg_update_short_2x(add_2(j)) = current_position_x(p,i);
                    cp_reg_update_short_2y(add_2(j)) = current_position_y(p,i);
                end
                xyz_2 = [cp_reg_update_short_2x;cp_reg_update_short_2y;cp_reg_update_short_2z]; % control points
                xyz_2 = [xyz_2 xyz_2(:,1)] ;
                [points_2,~] = fnplt_1(cscvn(xyz_2)) ;     % curve points
            end
            if isempty(cp_reg_update_short_3)~=1
                
                for i=length(add_1)+length(add_2)+1:length(add_1)+length(add_2)+length(add_3)
                    j = i - length(add_1)-length(add_2) ;
                    cp_reg_update_short_3x(add_3(j)) = current_position_x(p,i);
                    cp_reg_update_short_3y(add_3(j)) = current_position_y(p,i);
                end
                xyz_3 = [cp_reg_update_short_3x;cp_reg_update_short_3y;cp_reg_update_short_3z]; % control points
                xyz_3 = [xyz_3 xyz_3(:,1)] ;
                [points_3,~] = fnplt_1(cscvn(xyz_3)) ;     % curve points
            end
            if isempty(cp_reg_update_short_4)~=1
                
                for i=length(add_1)+length(add_2)+length(add_3)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)
                    j = i - length(add_1)-length(add_2)-length(add_3) ;
                    cp_reg_update_short_4x(add_4(j)) = current_position_x(p,i);
                    cp_reg_update_short_4y(add_4(j)) = current_position_y(p,i);
                end
                xyz_4 = [cp_reg_update_short_4x;cp_reg_update_short_4y;cp_reg_update_short_4z]; % control points
                xyz_4 = [xyz_4 xyz_4(:,1)] ;
                [points_4,~] = fnplt_1(cscvn(xyz_4)) ;     % curve points
            end
            if isempty(cp_reg_update_short_5)~=1
                
                for i=length(add_1)+length(add_2)+length(add_3)+length(add_4)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)
                    j = i - length(add_1)-length(add_2)-length(add_3)-length(add_4);
                    cp_reg_update_short_5x(add_5(j)) = current_position_x(p,i);
                    cp_reg_update_short_5y(add_5(j)) = current_position_y(p,i);
                end
                xyz_5 = [cp_reg_update_short_5x;cp_reg_update_short_5y;cp_reg_update_short_5z]; % control points
                xyz_5 = [xyz_5 xyz_5(:,1)] ;
                [points_5,~] = fnplt_1(cscvn(xyz_5)) ;     % curve points
            end
            if isempty(cp_reg_update_short_6)~=1
                
                for i=length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)+length(add_6)
                    j = i - length(add_1)-length(add_2)-length(add_3)-length(add_4)-length(add_5);
                    cp_reg_update_short_6x(add_6(j)) = current_position_x(p,i);
                    cp_reg_update_short_6y(add_6(j)) = current_position_y(p,i);
                end
                xyz_6 = [cp_reg_update_short_6x;cp_reg_update_short_6y;cp_reg_update_short_6z]; % control points
                xyz_6 = [xyz_6 xyz_6(:,1)] ;
                [points_6,~] = fnplt_1(cscvn(xyz_6)) ;     % curve points
            end
        end
        % fitness function calculations
        % intersection points
        if n_uav==2
            [x12, y12] = polyxpoly(points_1(1,:),points_1(2,:),points_2(1,:),points_2(2,:));
            f_1(p) = size(x12,1) ;
        elseif n_uav==4
            [x12, y12] = polyxpoly(points_1(1,:),points_1(2,:),points_2(1,:),points_2(2,:));
            [x13, y13] = polyxpoly(points_1(1,:),points_1(2,:),points_3(1,:),points_3(2,:));
            [x14, y14] = polyxpoly(points_1(1,:),points_1(2,:),points_4(1,:),points_4(2,:));
            [x23, y23] = polyxpoly(points_2(1,:),points_2(2,:),points_3(1,:),points_3(2,:));
            [x24, y24] = polyxpoly(points_2(1,:),points_2(2,:),points_4(1,:),points_4(2,:));
            [x34, y34] = polyxpoly(points_3(1,:),points_3(2,:),points_4(1,:),points_4(2,:));
            f_1(p) = size(x12,1) + size(x13,1)+size(x14,1) + size(x23,1)+size(x24,1) + size(x34,1) ;
        else
            [x12, y12] = polyxpoly(points_1(1,:),points_1(2,:),points_2(1,:),points_2(2,:));
            [x13, y13] = polyxpoly(points_1(1,:),points_1(2,:),points_3(1,:),points_3(2,:));
            [x14, y14] = polyxpoly(points_1(1,:),points_1(2,:),points_4(1,:),points_4(2,:));
            [x15, y15] = polyxpoly(points_1(1,:),points_1(2,:),points_5(1,:),points_5(2,:));
            [x16, y16] = polyxpoly(points_1(1,:),points_1(2,:),points_6(1,:),points_6(2,:));
            [x23, y23] = polyxpoly(points_2(1,:),points_2(2,:),points_3(1,:),points_3(2,:));
            [x24, y24] = polyxpoly(points_2(1,:),points_2(2,:),points_4(1,:),points_4(2,:));
            [x25, y25] = polyxpoly(points_2(1,:),points_2(2,:),points_5(1,:),points_5(2,:));
            [x26, y26] = polyxpoly(points_2(1,:),points_2(2,:),points_6(1,:),points_6(2,:));
            [x34, y34] = polyxpoly(points_3(1,:),points_3(2,:),points_4(1,:),points_4(2,:));
            [x35, y35] = polyxpoly(points_3(1,:),points_3(2,:),points_5(1,:),points_5(2,:));
            [x36, y36] = polyxpoly(points_3(1,:),points_3(2,:),points_6(1,:),points_6(2,:));            
            [x45, y45] = polyxpoly(points_4(1,:),points_4(2,:),points_5(1,:),points_5(2,:));
            [x46, y46] = polyxpoly(points_4(1,:),points_4(2,:),points_6(1,:),points_6(2,:));
            [x56, y56] = polyxpoly(points_5(1,:),points_5(2,:),points_6(1,:),points_6(2,:));            
            
            f_1(p) = size(x12,1) + size(x13,1)+size(x14,1) + size(x15,1)+size(x16,1) + size(x23,1)+...
                size(x24,1) + size(x25,1)+size(x26,1) + size(x34,1)+size(x35,1) + size(x36,1)+...
                size(x45,1)+size(x46,1) + size(x56,1);            
        end
        
        if n_uav ==2
            
            % fitness for the distance to check points & the number of unvisited points
            x = points_1(1,:) ;
            y = points_1(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_1)   ;
            for j = 1:length(temp_x_1)
                d = sqrt((x-temp_x_1(j)).^2+(y-temp_y_1(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_1 = top_dist ;
            temp_points_1 = unvisited_points ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_2)   ;
            for j = 1:length(temp_x_2)
                d = sqrt((x-temp_x_2(j)).^2+(y-temp_y_2(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_2 = top_dist ;
            temp_points_2 = unvisited_points ;
            
%             f_1(p) = temp_dist_1 + temp_dist_2           ;
            f_2(p) = temp_points_1 + temp_points_2   ;
            
        elseif n_uav ==4
            % fitness for the distance to check points & the number of unvisited points
            x = points_1(1,:) ;
            y = points_1(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_1)   ;
            for j = 1:length(temp_x_1)
                d = sqrt((x-temp_x_1(j)).^2+(y-temp_y_1(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_1 = top_dist ;
            temp_points_1 = unvisited_points ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_2)   ;
            for j = 1:length(temp_x_2)
                d = sqrt((x-temp_x_2(j)).^2+(y-temp_y_2(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_2 = top_dist ;
            temp_points_2 = unvisited_points ;
            
            % fitness for the distance to check points & the number of unvisited points
            x = points_3(1,:) ;
            y = points_3(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_3)   ;
            for j = 1:length(temp_x_3)
                d = sqrt((x-temp_x_3(j)).^2+(y-temp_y_3(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_3 = top_dist ;
            temp_points_3 = unvisited_points ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_4)   ;
            for j = 1:length(temp_x_4)
                d = sqrt((x-temp_x_4(j)).^2+(y-temp_y_4(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_4 = top_dist ;
            temp_points_4 = unvisited_points ;
            
%             f_1(p) = temp_dist_1 + temp_dist_2 + temp_dist_3 + temp_dist_4          ;
            f_2(p) = temp_points_1 + temp_points_2 + temp_points_3 + temp_points_4   ;
            
        else
            % fitness for the distance to check points & the number of unvisited points
            x = points_1(1,:) ;
            y = points_1(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_1)   ;
            for j = 1:length(temp_x_1)
                d = sqrt((x-temp_x_1(j)).^2+(y-temp_y_1(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_1 = top_dist ;
            temp_points_1 = unvisited_points ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_2)   ;
            for j = 1:length(temp_x_2)
                d = sqrt((x-temp_x_2(j)).^2+(y-temp_y_2(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_2 = top_dist ;
            temp_points_2 = unvisited_points ;
            
            x = points_3(1,:) ;
            y = points_3(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_3)   ;
            for j = 1:length(temp_x_3)
                d = sqrt((x-temp_x_3(j)).^2+(y-temp_y_3(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_3 = top_dist ;
            temp_points_3 = unvisited_points ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_4)   ;
            for j = 1:length(temp_x_4)
                d = sqrt((x-temp_x_4(j)).^2+(y-temp_y_4(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_4 = top_dist ;
            temp_points_4 = unvisited_points ;
            
            x = points_5(1,:) ;
            y = points_5(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_5)   ;
            for j = 1:length(temp_x_5)
                d = sqrt((x-temp_x_5(j)).^2+(y-temp_y_5(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_5 = top_dist ;
            temp_points_5 = unvisited_points ;
            
            x = points_6(1,:) ;
            y = points_6(2,:) ;
%             n_Steps     = length(x)         ;   % k -> Number of steps in each path
%             top_dist = 0                    ;
            unvisited_points = length(temp_x_6)   ;
            for j = 1:length(temp_x_6)
                d = sqrt((x-temp_x_6(j)).^2+(y-temp_y_6(j)).^2) ;
                dd(j) = min(d) ;
                if dd(j) <= s_Dia
                    unvisited_points = unvisited_points -1 ;
%                 else
%                     top_dist = top_dist + dd(j) ;
                end
            end
%             temp_dist_6 = top_dist ;
            temp_points_6 = unvisited_points ;
            
%             f_1(p) = temp_dist_1 + temp_dist_2 + temp_dist_3 + temp_dist_4 + temp_dist_5 + temp_dist_6          ;
            f_2(p) = temp_points_1 + temp_points_2 + temp_points_3 + temp_points_4 + temp_points_5 + temp_points_6   ;
            
        end
        
        if n_uav ==2
            % curve length
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(1) = DD / vel ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(2) = DD / vel ;
            
            f_3 (p) = max(revisit) ;
            
        elseif n_uav ==4
            % curve length
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(1) = DD / vel ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(2) = DD / vel ;
            
            x = points_3(1,:) ;
            y = points_3(2,:) ;
            z = points_3(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(3) = DD / vel ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
            z = points_4(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(4) = DD / vel ;
            
            f_3 (p) = max(revisit); 
        else
            % curve length
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(1) = DD / vel ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(2) = DD / vel ;
            
            x = points_3(1,:) ;
            y = points_3(2,:) ;
            z = points_3(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(3) = DD / vel ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
            z = points_4(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(4) = DD / vel ;
            
            x = points_5(1,:) ;
            y = points_5(2,:) ;
            z = points_5(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(5) = DD / vel ;
            
            x = points_6(1,:) ;
            y = points_6(2,:) ;
            z = points_6(3,:) ;
            
            n_Steps = length(x) ;
            
            Dx = (x(2:n_Steps) - x(1:n_Steps-1)).^2 ;
            Dy = (y(2:n_Steps) - y(1:n_Steps-1)).^2 ;
            Dz = (z(2:n_Steps) - z(1:n_Steps-1)).^2 ;
            DD = sum(sqrt(Dx + Dy + Dz)) ;
            revisit(6) = DD / vel ;
            
            f_3 (p) = max(revisit) ;
            
        end
        
        if n_uav ==2
            
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_1(p,1) =  pun_clear ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_2(p,1) =  pun_clear ;
            
            f_4(p) = temp_pun_clear_1(p,1) + temp_pun_clear_2(p,1) ;
            
        elseif n_uav ==4
            
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_1(p,1) =  pun_clear ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_2(p,1) =  pun_clear ;
            
            x = points_3(1,:) ;
            y = points_3(2,:) ;
            z = points_3(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_3(p,1) =  pun_clear ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
            z = points_4(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_4(p,1) =  pun_clear ;
            
            f_4(p) = temp_pun_clear_1(p,1) + temp_pun_clear_2(p,1)+ temp_pun_clear_3(p,1) + temp_pun_clear_4(p,1) ; ;
            
        else
            x = points_1(1,:) ;
            y = points_1(2,:) ;
            z = points_1(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_1(p,1) =  pun_clear ;
            
            x = points_2(1,:) ;
            y = points_2(2,:) ;
            z = points_2(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_2(p,1) =  pun_clear ;
            
            x = points_3(1,:) ;
            y = points_3(2,:) ;
            z = points_3(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_3(p,1) =  pun_clear ;
            
            x = points_4(1,:) ;
            y = points_4(2,:) ;
            z = points_4(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_4(p,1) =  pun_clear ;
            
            x = points_5(1,:) ;
            y = points_5(2,:) ;
            z = points_5(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_5(p,1) =  pun_clear ;
            
            x = points_6(1,:) ;
            y = points_6(2,:) ;
            z = points_6(3,:) ;
            
            % surface clearance
            pun_clear = 0;
            for j = 1:length(x)
                flag = 0 ;
                for t = 1:size(XX,2)-1
                    if (XX(1,t)<x(j)) && x(j)<(XX(1,t+1))
                        for tt = 1:size(YY,2)-1
                            if (YY(tt,1)<y(j))&& y(j)<(YY(tt+1,1))
                                if z(j)-ZZ(tt,t) < Clear
                                    pun_clear = pun_clear+1;
                                end
                                flag = 1 ;
                            end
                            if flag==1
                                break
                            end
                        end
                        if flag==1
                            break
                        end
                    end
                end
            end
            temp_pun_clear_6(p,1) =  pun_clear ;
            
            f_4(p) = temp_pun_clear_1(p,1) + temp_pun_clear_2(p,1) + temp_pun_clear_3(p,1) + temp_pun_clear_4(p,1) + temp_pun_clear_5(p,1) + temp_pun_clear_6(p,1) ;
            
        end
        
        
        % total value
        Ff(p) = a1 * f_1(p) + a2 * f_2(p)+ a3 *f_3(p)+ a4 * f_4(p);
    end
    
    % prediction strategy
    if n_uav==2
        [f4_best_1, b1] = min(temp_pun_clear_1) ;
        [f4_best_2, b2] = min(temp_pun_clear_2) ;
        
        if f4_best_1<temp_pun_clear_1_best
            if isempty(add_1)~=1
                best_path_1_x = current_position_x(b1,1:length(add_1)) ;
                best_path_1_y = current_position_y(b1,1:length(add_1)) ;
                temp_pun_clear_1_best = f4_best_1 ;
            else
                best_path_1_x = [] ;
                best_path_1_y = [] ;
            end
        end
        
        if f4_best_2 < temp_pun_clear_2_best
            if isempty(add_2)~=1
                best_path_2_x = current_position_x(b2,length(add_1)+1:length(add_1)+length(add_2)) ;
                best_path_2_y = current_position_y(b2,length(add_1)+1:length(add_1)+length(add_2)) ;
                temp_pun_clear_2_best = f4_best_2 ;
            else
                best_path_2_x = [] ;
                best_path_2_y = [] ;
            end
        end
        
        predict_2x = [best_path_1_x best_path_2_x] ;
        predict_2y = [best_path_1_y best_path_2_y] ;
        
    elseif n_uav==4
        
        [f4_best_1, b1] = min(temp_pun_clear_1) ;
        [f4_best_2, b2] = min(temp_pun_clear_2) ;
        [f4_best_3, b3] = min(temp_pun_clear_3) ;
        [f4_best_4, b4] = min(temp_pun_clear_4) ;
        
        if f4_best_1<temp_pun_clear_1_best
            if isempty(add_1)~=1
                best_path_1_x = current_position_x(b1,1:length(add_1)) ;
                best_path_1_y = current_position_y(b1,1:length(add_1)) ;
                temp_pun_clear_1_best = f4_best_1 ;
            else
                best_path_1_x = [] ;
                best_path_1_y = [] ;
            end
        end
        
        if f4_best_2 < temp_pun_clear_2_best
            if isempty(add_2)~=1
                best_path_2_x = current_position_x(b2,length(add_1)+1:length(add_1)+length(add_2)) ;
                best_path_2_y = current_position_y(b2,length(add_1)+1:length(add_1)+length(add_2)) ;
                temp_pun_clear_2_best = f4_best_2 ;
            else
                best_path_2_x = [] ;
                best_path_2_y = [] ;
            end
        end
        
        if f4_best_3 < temp_pun_clear_3_best
            if isempty(add_3)~=1
                best_path_3_x = current_position_x(b3,length(add_1)+length(add_2)+1:length(add_1)+length(add_2)+length(add_3)) ;
                best_path_3_y = current_position_y(b3,length(add_1)+length(add_2)+1:length(add_1)+length(add_2)+length(add_3)) ;
                temp_pun_clear_3_best = f4_best_3 ;
            else
                best_path_3_x = [] ;
                best_path_3_y = [] ;
            end
        end
        
        if f4_best_4 < temp_pun_clear_4_best
            if isempty(add_4)~=1
                best_path_4_x = current_position_x(b4,length(add_1)+length(add_2)+length(add_3)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)) ;
                best_path_4_y = current_position_y(b4,length(add_1)+length(add_2)+length(add_3)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)) ;
                temp_pun_clear_4_best = f4_best_4 ;
            else
                best_path_4_x = [] ;
                best_path_4_y = [] ;
            end
        end
        predict_4x = [best_path_1_x best_path_2_x best_path_3_x best_path_4_x] ;
        predict_4y = [best_path_1_y best_path_2_y best_path_3_y best_path_4_y] ;
    else
        [f4_best_1, b1] = min(temp_pun_clear_1) ;
        [f4_best_2, b2] = min(temp_pun_clear_2) ;
        [f4_best_3, b3] = min(temp_pun_clear_3) ;
        [f4_best_4, b4] = min(temp_pun_clear_4) ;
        [f4_best_5, b5] = min(temp_pun_clear_5) ;
        [f4_best_6, b6] = min(temp_pun_clear_6) ;
        
        if f4_best_1<temp_pun_clear_1_best
            if isempty(add_1)~=1
                best_path_1_x = current_position_x(b1,1:length(add_1)) ;
                best_path_1_y = current_position_y(b1,1:length(add_1)) ;
                temp_pun_clear_1_best = f4_best_1 ;
            else
                best_path_1_x = [] ;
                best_path_1_y = [] ;
            end
        end
        
        if f4_best_2 < temp_pun_clear_2_best
            if isempty(add_2)~=1
                best_path_2_x = current_position_x(b2,length(add_1)+1:length(add_1)+length(add_2)) ;
                best_path_2_y = current_position_y(b2,length(add_1)+1:length(add_1)+length(add_2)) ;
                temp_pun_clear_2_best = f4_best_2 ;
            else
                best_path_2_x = [] ;
                best_path_2_y = [] ;
            end
        end
        
        if f4_best_3 < temp_pun_clear_3_best
            if isempty(add_3)~=1
                best_path_3_x = current_position_x(b3,length(add_1)+length(add_2)+1:length(add_1)+length(add_2)+length(add_3)) ;
                best_path_3_y = current_position_y(b3,length(add_1)+length(add_2)+1:length(add_1)+length(add_2)+length(add_3)) ;
                temp_pun_clear_3_best = f4_best_3 ;
            else
                best_path_3_x = [] ;
                best_path_3_y = [] ;
            end
        end
        
        if f4_best_4 < temp_pun_clear_4_best
            if isempty(add_4)~=1
                best_path_4_x = current_position_x(b4,length(add_1)+length(add_2)+length(add_3)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)) ;
                best_path_4_y = current_position_y(b4,length(add_1)+length(add_2)+length(add_3)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)) ;
                temp_pun_clear_4_best = f4_best_4 ;
            else
                best_path_4_x = [] ;
                best_path_4_y = [] ;
            end
        end
        
        if f4_best_5 < temp_pun_clear_5_best
            if isempty(add_5)~=1
                best_path_5_x = current_position_x(b5,length(add_1)+length(add_2)+length(add_3)+length(add_4)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)) ;
                best_path_5_y = current_position_y(b5,length(add_1)+length(add_2)+length(add_3)+length(add_4)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)) ;
                temp_pun_clear_5_best = f4_best_5 ;
            else
                best_path_5_x = [] ;
                best_path_5_y = [] ;
            end
        end
        
        if f4_best_6 < temp_pun_clear_6_best
            if isempty(add_6)~=1
                best_path_6_x = current_position_x(b6,length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)+length(add_6)) ;
                best_path_6_y = current_position_y(b6,length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)+length(add_6)) ;
                temp_pun_clear_6_best = f4_best_6 ;
            else
                best_path_6_x = [] ;
                best_path_6_y = [] ;
            end
        end
        
        predict_6x = [best_path_1_x best_path_2_x best_path_3_x best_path_4_x best_path_5_x best_path_6_x] ;
        predict_6y = [best_path_1_y best_path_2_y best_path_3_y best_path_4_y best_path_5_y best_path_6_y] ;
    end
    
    current_fitness         = Ff                               ;
    [current_worst_fitness,worst] = max(Ff)       ;
    
    for i = 1 : n
        if current_fitness(i)       < local_best_fitness(i)
            local_best_fitness(i)    = current_fitness(i)        ;
            local_best_position_x(i,:) = current_position_x(i,:)     ;
            local_best_position_y(i,:) = current_position_y(i,:)     ;
        end
    end
    
    [current_global_best_fitness,g] = min(local_best_fitness)       ;
    F_1(iter) = f_1(g) ;
    F_2(iter) = f_2(g) ;
    F_3(iter) = f_3(g) ;
    F_4(iter) = f_4(g) ;
    
    if current_global_best_fitness < global_best_fitness
        global_best_fitness = current_global_best_fitness           ;
        for i=1:n
            globl_best_position_x(i,:) = local_best_position_x(g,:)     ;
            globl_best_position_y(i,:) = local_best_position_y(g,:)     ;
        end
    end
    
    best_fitness(iter) = global_best_fitness    ;
    elit_fitness(iter+1)= best_fitness(iter)    ;
    
    F(iter,:) = elit_fitness(iter+1)           ;
    
    R1 = rand(n,total_dim)                       ;
    R2 = rand(n,total_dim)                       ;
    
    %-------------------%
    %  VELOCITY UPDATE  %
    %-------------------%
    
    w_next = w_i -(w_i-w_e)/bird_step*iter  ;
    
    velocity_x = w_next*velocity_x + c1*(R1.*(local_best_position_x-current_position_x))/dt + c2*(R2.*(globl_best_position_x-current_position_x)/dt) ;
    velocity_y = w_next*velocity_y + c1*(R1.*(local_best_position_y-current_position_y))/dt + c2*(R2.*(globl_best_position_y-current_position_y)/dt) ;
    
    %------------------%
    %   SWARMUPDATE    %
    %------------------%
    
    current_position_x = current_position_x + velocity_x ;
    current_position_y = current_position_y + velocity_y ;
    
    % mutation application
    if rem(iter,fq)==0
        for i=1:size(current_position_x,2)
            for j=1:p
                current_position_x(j,i)=current_position_x(j,i)*(1+amp*(0.5-rand));
                current_position_y(j,i)=current_position_y(j,i)*(1+amp*(0.5-rand));
            end
        end
    end
    
    % prediction strategy
    if n_uav==2
        if isempty(predict_2x)~=1
            current_position_x(worst,:) = predict_2x ;
            current_position_y(worst,:) = predict_2y ;
        else
        end
    elseif n_uav==4
        if isempty(predict_4x)~=1
            current_position_x(worst,:) = predict_4x ;
            current_position_y(worst,:) = predict_4y ;
        else
        end
    else
        if isempty(predict_6x)~=1
            current_position_x(worst,:) = predict_6x ;
            current_position_y(worst,:) = predict_6y ;
        else
        end
    end
    
    clear f1 f2 f3 f4 n_Steps Dx Dy Dz DD revisit temp_pun_clear_1 temp_pun_clear_2...
        temp_pun_clear_3 temp_pun_clear_4 temp_pun_clear_5 temp_pun_clear_6...
        temp_points_1 temp_points_2 temp_points_3 temp_points_4 temp_points_5 temp_points_6...
        unvisited_points d dd x y xyz_1 xyz_2 xyz_3 xyz_4 xyz_5 xyz_6 temp_pun_clear_1 temp_pun_clear_2...
        temp_pun_clear_3 temp_pun_clear_4 temp_pun_clear_5 temp_pun_clear_6 predict_2x predict_2y predict_4x predict_4y...
        predict_5x predict_5y predict_6x predict_6y
    
end % end of while loop

if  min(F) < temp_fitness
    temp_fitness = min(F) ;
    save pso_5_2_best
end
General(run,:)=F ;
Ff_1(run,:) = F_1 ;
Ff_2(run,:) = F_2 ;
Ff_3(run,:) = F_3 ;
Ff_4(run,:) = F_4 ;

save pso_5_2
clear current_position_x  current_position_y current_position_z

end

clear
load pso_5_2_best.mat

% visualization of the best curve
% surface show
cla
xlabel('x');
ylabel('y');
% curve show
hold on
plot(tx,ty,'ko')
%
% curve points via spline "cscvn"
temp_globl_best_position_x = globl_best_position_x(1,:) ;
temp_globl_best_position_y = globl_best_position_y(1,:) ;

if n_uav ==2
    for i=1:length(add_1)
        cp_reg_update_short_1x(add_1(i)) = temp_globl_best_position_x(1,i);
        cp_reg_update_short_1y(add_1(i)) = temp_globl_best_position_y(1,i);
    end
    xyz_1 = [cp_reg_update_short_1x;cp_reg_update_short_1y;cp_reg_update_short_1z]; % control points
    xyz_1 = [xyz_1 xyz_1(:,1)] ;
    [points_1,~] = fnplt_1(cscvn(xyz_1)) ;     % curve points
    
    for i=length(add_1)+1:length(add_1)+length(add_2)
        j = i - length(add_1) ;
        cp_reg_update_short_2x(add_2(j)) = temp_globl_best_position_x(1,i);
        cp_reg_update_short_2y(add_2(j)) = temp_globl_best_position_y(1,i);
    end
    xyz_2 = [cp_reg_update_short_2x;cp_reg_update_short_2y;cp_reg_update_short_2z]; % control points
    xyz_2 = [xyz_2 xyz_2(:,1)] ;
    [points_2,~] = fnplt_1(cscvn(xyz_2)) ;     % curve points
    
    plot3(points_1(1,:),points_1(2,:),points_1(3,:))
    plot3(points_2(1,:),points_2(2,:),points_2(3,:))
elseif n_uav ==4
    if isempty(cp_reg_update_short_1)~=1
        for i=1:length(add_1)
            cp_reg_update_short_1x(add_1(i)) = temp_globl_best_position_x(1,i);
            cp_reg_update_short_1y(add_1(i)) = temp_globl_best_position_y(1,i);
        end
        xyz_1 = [cp_reg_update_short_1x;cp_reg_update_short_1y;cp_reg_update_short_1z]; % control points
        xyz_1 = [xyz_1 xyz_1(:,1)] ;
        [points_1,~] = fnplt_1(cscvn(xyz_1)) ;     % curve points
    end
    if isempty(cp_reg_update_short_2)~=1
        for i=length(add_1)+1:length(add_1)+length(add_2)
            j = i - length(add_1) ;
            cp_reg_update_short_2x(add_2(j)) = temp_globl_best_position_x(1,i);
            cp_reg_update_short_2y(add_2(j)) = temp_globl_best_position_y(1,i);
        end
        xyz_2 = [cp_reg_update_short_2x;cp_reg_update_short_2y;cp_reg_update_short_2z]; % control points
        xyz_2 = [xyz_2 xyz_2(:,1)] ;
        [points_2,~] = fnplt_1(cscvn(xyz_2)) ;     % curve points
    end
    if isempty(cp_reg_update_short_3)~=1
        for i=length(add_1)+length(add_2)+1:length(add_1)+length(add_2)+length(add_3)
            j = i - length(add_1)-length(add_2) ;
            cp_reg_update_short_3x(add_3(j)) = temp_globl_best_position_x(1,i);
            cp_reg_update_short_3y(add_3(j)) = temp_globl_best_position_y(1,i);
        end
        xyz_3 = [cp_reg_update_short_3x;cp_reg_update_short_3y;cp_reg_update_short_3z]; % control points
        xyz_3 = [xyz_3 xyz_3(:,1)] ;
        [points_3,~] = fnplt_1(cscvn(xyz_3)) ;     % curve points
    end
    if isempty(cp_reg_update_short_4)~=1
        for i=length(add_1)+length(add_2)+length(add_3)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)
            j = i - length(add_1)-length(add_2)-length(add_3) ;
            cp_reg_update_short_4x(add_4(j)) = temp_globl_best_position_x(1,i);
            cp_reg_update_short_4y(add_4(j)) = temp_globl_best_position_y(1,i);
        end
        xyz_4 = [cp_reg_update_short_4x;cp_reg_update_short_4y;cp_reg_update_short_4z]; % control points
        xyz_4 = [xyz_4 xyz_4(:,1)] ;
        [points_4,~] = fnplt_1(cscvn(xyz_4)) ;     % curve points
    end
    plot3(points_1(1,:),points_1(2,:),points_1(3,:))
    plot3(points_2(1,:),points_2(2,:),points_2(3,:))
    plot3(points_3(1,:),points_3(2,:),points_3(3,:))
    plot3(points_4(1,:),points_4(2,:),points_4(3,:))
else
    if isempty(cp_reg_update_short_1)~=1
        for i=1:length(add_1)
            cp_reg_update_short_1x(add_1(i)) = temp_globl_best_position_x(1,i);
            cp_reg_update_short_1y(add_1(i)) = temp_globl_best_position_y(1,i);
        end
        xyz_1 = [cp_reg_update_short_1x;cp_reg_update_short_1y;cp_reg_update_short_1z]; % control points
        xyz_1 = [xyz_1 xyz_1(:,1)] ;
        [points_1,~] = fnplt_1(cscvn(xyz_1)) ;     % curve points
    end
    if isempty(cp_reg_update_short_2)~=1
        for i=length(add_1)+1:length(add_1)+length(add_2)
            j = i - length(add_1) ;
            cp_reg_update_short_2x(add_2(j)) = temp_globl_best_position_x(1,i);
            cp_reg_update_short_2y(add_2(j)) = temp_globl_best_position_y(1,i);
        end
        xyz_2 = [cp_reg_update_short_2x;cp_reg_update_short_2y;cp_reg_update_short_2z]; % control points
        xyz_2 = [xyz_2 xyz_2(:,1)] ;
        [points_2,~] = fnplt_1(cscvn(xyz_2)) ;     % curve points
    end
    if isempty(cp_reg_update_short_3)~=1
        for i=length(add_1)+length(add_2)+1:length(add_1)+length(add_2)+length(add_3)
            j = i - length(add_1)-length(add_2) ;
            cp_reg_update_short_3x(add_3(j)) = temp_globl_best_position_x(1,i);
            cp_reg_update_short_3y(add_3(j)) = temp_globl_best_position_y(1,i);
        end
        xyz_3 = [cp_reg_update_short_3x;cp_reg_update_short_3y;cp_reg_update_short_3z]; % control points
        xyz_3 = [xyz_3 xyz_3(:,1)] ;
        [points_3,~] = fnplt_1(cscvn(xyz_3)) ;     % curve points
    end
    if isempty(cp_reg_update_short_4)~=1
        for i=length(add_1)+length(add_2)+length(add_3)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)
            j = i - length(add_1)-length(add_2)-length(add_3) ;
            cp_reg_update_short_4x(add_4(j)) = temp_globl_best_position_x(1,i);
            cp_reg_update_short_4y(add_4(j)) = temp_globl_best_position_y(1,i);
        end
        xyz_4 = [cp_reg_update_short_4x;cp_reg_update_short_4y;cp_reg_update_short_4z]; % control points
        xyz_4 = [xyz_4 xyz_4(:,1)] ;
        [points_4,~] = fnplt_1(cscvn(xyz_4)) ;     % curve points
    end
    if isempty(cp_reg_update_short_5)~=1
        
        for i=length(add_1)+length(add_2)+length(add_3)+length(add_4)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)
            j = i - length(add_1)-length(add_2)-length(add_3)-length(add_4);
            cp_reg_update_short_5x(add_5(j)) = temp_globl_best_position_x(1,i);
            cp_reg_update_short_5y(add_5(j)) = temp_globl_best_position_y(1,i);
        end
        xyz_5 = [cp_reg_update_short_5x;cp_reg_update_short_5y;cp_reg_update_short_5z]; % control points
        xyz_5 = [xyz_5 xyz_5(:,1)] ;
        [points_5,~] = fnplt_1(cscvn(xyz_5)) ;     % curve points
    end
    if isempty(cp_reg_update_short_6)~=1
        
        for i=length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)+1:length(add_1)+length(add_2)+length(add_3)+length(add_4)+length(add_5)+length(add_6)
            j = i - length(add_1)-length(add_2)-length(add_3)-length(add_4)-length(add_5);
            cp_reg_update_short_6x(add_6(j)) = temp_globl_best_position_x(1,i);
            cp_reg_update_short_6y(add_6(j)) = temp_globl_best_position_y(1,i);
        end
        xyz_6 = [cp_reg_update_short_6x;cp_reg_update_short_6y;cp_reg_update_short_6z]; % control points
        xyz_6 = [xyz_6 xyz_6(:,1)] ;
        [points_6,~] = fnplt_1(cscvn(xyz_6)) ;     % curve points
    end
    
    plot3(points_1(1,:),points_1(2,:),points_1(3,:))
    plot3(points_2(1,:),points_2(2,:),points_2(3,:))
    plot3(points_3(1,:),points_3(2,:),points_3(3,:))
    plot3(points_4(1,:),points_4(2,:),points_4(3,:))
    plot3(points_5(1,:),points_5(2,:),points_5(3,:))
    plot3(points_6(1,:),points_6(2,:),points_6(3,:))
end



for i = 1:length(tx)
    for j = 0:20:360
        dot_x = tx(i)+ s_Dia * cosd(j) ;
        dot_y = ty(i)+ s_Dia * sind(j) ;
        plot(dot_x,dot_y,'k+','MarkerSize',1)
        hold on
    end
end
axis equal

clear
load pso_5_2.mat

temp_fitness
mean(General(:,100))
max(General(:,100))

figure
plot(1:100,General,'k:')
hold on
plot(1:100,mean(General),'k')

figure
plot(1:100,Ff_1,'k:')
hold on
plot(1:100,mean(Ff_1),'k')

figure
plot(1:100,Ff_2,'k:')
hold on
plot(1:100,mean(Ff_2),'k')

figure
plot(1:100,Ff_3,'k:')
hold on
plot(1:100,mean(Ff_3),'k')

figure
plot(1:100,Ff_4,'k:')
hold on
plot(1:100,mean(Ff_4),'k')


