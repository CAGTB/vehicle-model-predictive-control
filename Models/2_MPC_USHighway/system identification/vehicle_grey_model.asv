function [A, B, C, D, K, x0] = vehicle_grey_model(p, T, varargin)
    
    %获取模型参数
    tau = p(1); m = p(2); Iz = p(3); Lf = p(4); Lr = p(5); Cf = p(6); Cr = p(7); Vx = p(8);

    % 连续时间矩阵定义 (结构化)
    % 纵向部分 (Ax -> Vx)
    % dx1 = -1/tau*x1 + 1/tau*u1 (Vx)
    % dx2 = x1 (Position, 虽然不需要，但原模型里有)
    A1 = [-1/tau 0; 1 0];
    B1 = [1/tau; 0];
    C1 = [0 1]; % 仅输出 Vx

    % 横向部分 (delta -> Vy, gamma)
    A2 = [-2*(Cf+Cr)/m/Vx, -Vx-2*(Cf*Lf-Cr*Lr)/m/Vx;
          -2*(Cf*Lf-Cr*Lr)/Iz/Vx, -2*(Cf*Lf^2+Cr*Lr^2)/Iz/Vx];
    B2 = 2*Cf*[1/m; Lf/Iz];
    C2 = [1 0; 0 1]; % 输出 Vy 和 gamma

    % 组合
    A = [A1, zeros(2,2); zeros(2,2), A2];
    B = [B1, zeros(2,1); zeros(2,1), B2];
    
    % 输出对应关系: y = [Vx; Vy; gamma]
    C = [1, 0, 0, 0; 
         0, 0, 1, 0; 
         0, 0, 0, 1]; 
    
    D = zeros(3, 2);

    % K 和 x0 
    % K 为扰动增益矩阵（卡尔曼增益），若不考虑过程噪声辨识，设为 0
    K = zeros(4, 3); 
    
    % x0 为初始状态，
    x0 = zeros(4, 1); 
end