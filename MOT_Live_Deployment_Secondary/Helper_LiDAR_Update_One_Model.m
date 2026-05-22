function [X_upd, P_upd] = Helper_LiDAR_Update_One_Model(X_pred, P_pred, Z_measured, H, R_LIDAR)
    Z_pred = H * X_pred;
    innovation = Z_measured - Z_pred;
    S = H * P_pred * H' + R_LIDAR;
    K = P_pred * H' / S;

    X_upd = X_pred + K * innovation;

    I = eye(size(P_pred));
    P_upd = (I - K*H) * P_pred * (I - K*H)' + K * R_LIDAR * K';
end