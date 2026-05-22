function tracks = Run_Tracker_Predict_3_1(tracks, IMM_TPM, dt_pred)

    % 1. Predict all active tracks from t-1 to t (this exact moment)  
    for i = 1 : length(tracks)
       [X_cv_pred, X_ca_pred, P_cv_pred, P_ca_pred, X_global_pred, P_global_pred, c_bar] = Helper_IMM_Prediction_Step( ...
            tracks(i).X_cv, tracks(i).P_cv, ...
            tracks(i).X_ca, tracks(i).P_ca, ...
            tracks(i).mu, IMM_TPM, dt_pred); 
    
        tracks(i).X_cv = X_cv_pred; 
        tracks(i).P_cv = P_cv_pred; 
        tracks(i).X_ca = X_ca_pred; 
        tracks(i).P_ca = P_ca_pred; 
        tracks(i).X = X_global_pred; 
        tracks(i).P = P_global_pred; 
        tracks(i).c_bar = c_bar; 

        tracks(i).P = (tracks(i).P + tracks(i).P') / 2; % Ensure covariance symmetry 
    end

end
