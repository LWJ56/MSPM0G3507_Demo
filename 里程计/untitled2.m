%% Simulation_example.m
% 里程计仿真示例
% 功能：演示如何使用Odometer_update函数进行轨迹仿真

% 清空工作区和关闭图形窗口
clear all;
close all;

% 初始化里程计状态
Odometer_est.x = 0;       % 初始x坐标 (cm)
Odometer_est.y = 0;       % 初始y坐标 (cm)
Odometer_est.theta = 0;   % 初始朝向 (rad)
Odometer_est.V = 0;       % 初始线速度 (cm/s)
Odometer_est.Omega = 0;   % 初始角速度 (rad/s)
Odometer_est.dt = 0.001;  % 时间步长 (s)

% 仿真参数设置
total_time = 10;          % 总仿真时间 (s)
dt = 0.001;              % 采样时间 (s)
steps = total_time / dt;  % 仿真步数

% 预分配轨迹存储数组
trajectory = zeros(steps, 3);  % [x, y, theta]

% 控制输入（示例：不同阶段设置不同的左右轮速度）
phase1_steps = floor(steps * 0.25);  % 第一阶段：直线前进
phase2_steps = floor(steps * 0.50);   % 第二阶段：右转
phase3_steps = steps;                % 第三阶段：直线后退

% 仿真循环
for i = 1:steps
    % 根据不同阶段设置左右轮速度
    if i <= phase1_steps
        % 阶段1：直线前进
        V_L = 30;  % 左轮速度 (rpm)
        V_R = 30;  % 右轮速度 (rpm)
    elseif i <= phase2_steps
        % 阶段2：右转
        V_L = 30;  % 左轮速度 (rpm)
        V_R = 15;  % 右轮速度 (rpm)
    else
        % 阶段3：直线后退
        V_L = -20; % 左轮速度 (rpm)
        V_R = -20; % 右轮速度 (rpm)
    end
    
    % 更新里程计状态
    Odometer_est = Odometer_update(V_L, V_R, Odometer_est);
    
    % 记录轨迹
    trajectory(i,:) = [Odometer_est.x, Odometer_est.y, Odometer_est.theta];
    
    % 每100步显示进度
    if mod(i, 1000) == 0
        fprintf('完成仿真步骤: %d / %d (%.1f%%)\n', i, steps, i/steps*100);
    end
end

% 绘制轨迹
figure('Position', [100, 100, 800, 600]);
plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('X (cm)');
ylabel('Y (cm)');
title('里程计轨迹仿真');
axis equal;

% 绘制起点和终点
hold on;
plot(trajectory(1,1), trajectory(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(trajectory(end,1), trajectory(end,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% 添加方向箭头
quiver_step = floor(steps/20);  % 每隔一定步数绘制一个方向箭头
for i = 1:quiver_step:steps
    quiver(trajectory(i,1), trajectory(i,2), 5*cos(trajectory(i,3)), 5*sin(trajectory(i,3)), ...
           0, 'k', 'LineWidth', 1);
end

% 显示最终位置和姿态
fprintf('\n仿真结束：\n');
fprintf('最终位置: (%.2f, %.2f) cm\n', Odometer_est.x, Odometer_est.y);
fprintf('最终朝向: %.2f 度\n', rad2deg(Odometer_est.theta));