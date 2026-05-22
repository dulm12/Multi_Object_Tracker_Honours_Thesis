function [X_cv_pred, X_ca_pred, P_cv_pred, P_ca_pred, X_global_pred, P_global_pred, c_bar] = Helper_IMM_Prediction_Step(X_cv, P_cv, X_ca, P_ca, mu_prev, IMM_TPM, dt)
    % IMM prediction performs mixing and prediction for 2-model IMM 

    % A. Define 9 State Motion Model 
   
    % Model 1: CV 
    F_cv = eye(9); % 9 x 9 Identity matrix
    F_cv(1:3, 4:6) = eye(3) * dt; 
   
    % Model 2: CA 
    F_ca = eye(9); 
    F_ca(1:3, 4:6) = eye(3) * dt; 
    F_ca(1:3, 7:9) = eye(3) * (0.5 * dt^2); 
    F_ca(4:6, 7:9) = eye(3) * dt; 
    
    % CV assumes 0 acceleration. 
    q_acc_cv = 0.5; % m/s^2 
 
    % CA allows sharp turns/dives. High process noise 
    q_acc_ca = 20.0; % m/s^2
  
    % Per-axis Q block (3x3)
    % noise drives acceleration; position & velocity are coupled
    G = [0.5 * dt^2; dt; 1];  % how jolts in birds trajectories propagates into [p, v, a]
    Q_axis_cv = q_acc_cv^2 * (G * G');
    Q_axis_ca = q_acc_ca^2 * (G * G');

    % Full 9x9 Q (block diagonal, one block per axis x/y/z)
    Q_cv = blkdiag(Q_axis_cv, Q_axis_cv, Q_axis_cv);
    Q_ca = blkdiag(Q_axis_ca, Q_axis_ca, Q_axis_ca);

    % B. IMM Mixing Step 

    % Normalisation Constant 
    c_bar = IMM_TPM' * mu_prev; 

    % Mixing Probabilities 
    mu_mix = zeros(2,2); 
    mu_mix(1,1) = (IMM_TPM(1,1) * mu_prev(1)) / c_bar(1); % CV to CV
    mu_mix(2,1) = (IMM_TPM(2,1) * mu_prev(2)) / c_bar(1); % CA to CV
    mu_mix(1,2) = (IMM_TPM(1,2) * mu_prev(1)) / c_bar(2); % CV to CA
    mu_mix(2,2) = (IMM_TPM(2,2) * mu_prev(2)) / c_bar(2); % CA to CA

    % Mixed Initial States (X0)
    X_01 = mu_mix(1,1) * X_cv + mu_mix(2,1) * X_ca; % For CV 
    X_02 = mu_mix(1,2) * X_cv + mu_mix(2,2) * X_ca; % For CA 

    % Mixed Initial Covariances (P0) 
    diff_cv_1 = X_cv - X_01; 
    diff_ca_1 = X_ca - X_01; 
    P_01 = mu_mix(1,1) * (P_cv + diff_cv_1 * diff_cv_1') + ...
           mu_mix(2,1) * (P_ca + diff_ca_1 * diff_ca_1'); 

    diff_cv_2 = X_cv - X_02; 
    diff_ca_2 = X_ca - X_02; 
    P_02 = mu_mix(1,2) * (P_cv + diff_cv_2 * diff_cv_2') + ...
           mu_mix(2,2) * (P_ca + diff_ca_2 * diff_ca_2');

    % C. Model-Specific Predictions 
    X_cv_pred = F_cv * X_01; 
    P_cv_pred = F_cv * P_01 * F_cv' + Q_cv; 

    X_ca_pred = F_ca * X_02;
    P_ca_pred = F_ca * P_02 * F_ca' + Q_ca; 

    % D. Global State Prediction 

    X_global_pred = c_bar(1) * X_cv_pred + c_bar(2) * X_ca_pred; 

    diff_cv_pred = X_cv_pred - X_global_pred; 
    diff_ca_pred = X_ca_pred - X_global_pred; 

    P_global_pred = c_bar(1) * (P_cv_pred + diff_cv_pred * diff_cv_pred') + ...
                    c_bar(2) * (P_ca_pred + (diff_ca_pred * diff_ca_pred'));

end