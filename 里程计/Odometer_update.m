%% Odometer.m
% 里程计仿真模型
% 功能：根据左右轮速度计算车辆位置和姿态

function [Odometer_est] = Odometer_update(V_L, V_R, Odometer_est)
    % 轮距 13.7cm
    Wheel_track = 13.7;
    % 车轮半径1.5cm
    Wheel_radius = 1.5;
    % 时间步长 1ms
    dt = 0.001;
    
    % 计算线速度 (cm/s)
    Odometer_est.V = (V_L + V_R) * pi / 40.0;
    
    % 计算角速度 (rad/s)
    Odometer_est.Omega = (V_R - V_L) * pi / 274.0;
    
    Odometer_est.dt = dt;
    
    % 中值积分法更新位置和角度
    Odometer_est.theta_mid = Odometer_est.theta + (Odometer_est.Omega * Odometer_est.dt) / 2;
    Odometer_est.x = Odometer_est.x + Odometer_est.V * cos(Odometer_est.theta_mid) * Odometer_est.dt;
    Odometer_est.y = Odometer_est.y + Odometer_est.V * sin(Odometer_est.theta_mid) * Odometer_est.dt;
    Odometer_est.theta = Odometer_est.theta + Odometer_est.Omega * Odometer_est.dt;
    
    % 角度归一化到 [-π, π]
    Odometer_est.theta = normalize_angle(Odometer_est.theta);
end

function [angle] = normalize_angle(angle)
    % 角度归一化到 [-π, π] 范围
    while (angle > pi)
        angle = angle - 2.0 * pi;
    end
    while (angle < -pi)
        angle = angle + 2.0 * pi;
    end
end