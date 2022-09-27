clear all;close all;
clc;
 
k = 0.1;    % look forward gain前视距离系数
Lfc = 1.0;  % look-ahead distance前视距离
Kp = 1.0 ;  % speed proportional gain速度P控制器系数
dt = 0.1  ; % 时间间隔，单位：s
L = 0.6  ;  % [m] wheel base of vehicle车辆轴距，单位：m
 

cx = [0.76, 1.52, 2.28, 3.04, 3.81, 4.57, 5.33, 5.44, 5.74, 6.66, 7.4, 7.6, 7.1, 6.9,6.5, 6, 5.6, 5.1];
cy = [0.1406933,0.5239,1.01,1.518,1.7990,1.726582,1.2278608,1.113062589,0.769004,-0.62, -1.8565, -3.5, -3.936, -4,-4.4, -4.7, -4.6, -4.2];
 
 
target_speed = 2/3.6;   %目标车速
T = 30;                 %最大模拟时间
lastIndex = length(cx); %最后索引

%初始坐标
x = 0; y = 0; 
yaw = 0; v = 1;         %初始偏航角  初始车速
time = 0;
Lf = k * v + Lfc;      %Ld预瞄距离
 
%主函数
figure
 target_ind = calc_target_index(x,y,cx,cy,Lf);
while T > time && lastIndex > target_ind               %仿真时间和索引（即轨迹点的个数）
   target_ind = calc_target_index(x,y,cx,cy,Lf);       %求解合适的索引
    ai = PIDcontrol(target_speed, v,Kp);               %求解加速度
    [delta, target_ind] = pure_pursuit_control(x,y,yaw,v,cx,cy,target_ind,k,Lfc,L,Lf); %求解前轮转角与索引
    [x,y,yaw,v] = update(x,y,yaw,v, ai, delta,dt,L);   %更新
    
    time = time + dt;                                   %时间更新
%   pause(0.1)   %pause(n) 暂停执行 n 秒，然后继续执行。必须启用暂停，此调用才能生效。
    plot(cx,cy,'bo',x,y,'r-*')
    drawnow  %drawnow 更新图窗并处理任何挂起的回调。如果您修改图形对象并且需要在屏幕上立即查看这次更新，请使用该命令。
    hold on
end
% plot(cx,cy,x,y,'*')
% hold on
 
%更新x  y  yaw  v
function [x, y, yaw, v] = update(x, y, yaw, v, a, delta, dt, L)
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    yaw = yaw + v / L * tan(delta) * dt;   % v / L * tan(delta) * dt = v / R * dt =w*dt 
    v = v + a * dt;
end
 
%PID控制
function [a] = PIDcontrol(target_v, current_v, Kp)  %由当前车速调控目标车速
a = Kp * (target_v - current_v);                    %加速度
end
 
%pure pursuit算法
function [delta, ind] = pure_pursuit_control(x,y,yaw,v,cx,cy,ind,k,Lfc,L,Lf)
% ind = calc_target_index(x,y,cx,cy,Lf);
 
if ind < length(cx)  %若目标点没有超过范围，去具体坐标赋予 tx，ty用作目标
    tx = cx(ind);
    ty = cy(ind);
else                %若超过了，把最后一个点赋给目标
    tx = cx(ind); 
    ty = cy(ind);
    ind = length(cx) - 1;
end
 
alpha = atan2((ty-y),(tx-x))-yaw;   %求航向角
 
if v < 0
    alpha = pi - alpha;
else
    alpha = alpha;
end
 
Lf = k * v + Lfc;    %前视距离的选取与速度有关，也与单位时间距离有关
delta = atan2(2*L * sin(alpha)/Lf,1)  ;         %必须用atan2()：四象限       atan()：二象限
end
 
%计算目标索引
function ind = calc_target_index(x,y, cx,cy,Lf)
N =  length(cx);
Distance = zeros(N,1);  %14x1
dx = zeros(N,1);
dy = zeros(N,1);
for i = 1:N
    dx(i) = x - cx(i);
end
 
for i = 1:N
    dy(i) = y - cy(i);
end
 
for i = 1:N
    Distance(i) = abs( sqrt(dx(i)^2 + dy(i)^2));  %离上一个点的距离
end
[~, location]= min(Distance);    %出最小的Distance所在的位置
                                 %[M,I] = min(___) 在上述语法基础上返回 A 中最小值在运算维度上的索引。
ind = location;
%     首先从目标点中找到一个离当前点最近的点
%     然后计算离这个点距离满足前视距离的下一个点
%     当两点之间的距离小于前视距离，需要累加几个点直至距离超过前视距离
%       
%     # search look ahead target point index
%     # 解读：从path point 接下来中找到 离当前点最接近于 前视距离的一个点
%     # 当路径中的下一个点离当前很远时，这里保证了目标点至少下移一个点，不会停留在原地
 
LL = 0;
 
    while Lf > LL && (ind + 1) < length(cx)
        dx = cx(ind + 1 )- cx(ind);
        dy = cx(ind + 1) - cx(ind);
        LL = LL + sqrt(dx^2 + dy^2);
        ind  = ind + 1;
    end
 
end
