% === Lab 3: Wireless Sensor Network Analysis ===
clc; clear; close all;

% ---- Parameters ----
L = 1;                  % 归一化环境大小
RC = 0.25 * L;          % 通信半径
RS = 0.2 * L;           % 传感半径

% ---- 读取数据 ----
data = readmatrix('D:\western uni\9047\lab3\Graph_251462513.txt');  % 用你自己的文件名替换
x = data(:,1);
y = data(:,2);
points = [x y];

% ---- 绘制节点和通信连接 ----
figure;
hold on; 
axis equal; %使x轴的刻度与y轴相等，不会被伸缩
xlim([0 1]); %x轴的显示范围是0-1
ylim([0 1]); %y轴的显示范围是0-1，这两句代码的意思使显示出来的图像不变形
title('Node Connectivity');
xlabel('x'); ylabel('y');

% 画节点
plot(x, y, 'bo', 'MarkerFaceColor', 'b');

% 画通信边
N = 25; %因为有25个点。这里要n就是25
for i = 1:N
    for j = i+1:N
        dist = norm(points(i,:) - points(j,:));%相当于勾股定理
        if dist <= RC
            plot([x(i) x(j)], [y(i) y(j)], 'r-');
        end
    end
end

% ---- 绘制覆盖区域 ----
figure;
hold on; 
axis equal;
title('Coverage Map');
xlim([0 1]); 
ylim([0 1]);

for i = 1:N
    theta = linspace(0, 2*pi, 100);
    fill(x(i) + RS*cos(theta), y(i) + RS*sin(theta), 'c', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end
plot(x, y, 'bo', 'MarkerFaceColor', 'b');
xlabel('x'); ylabel('y');

% ---- Voronoi 图和最大漏洞路径计算 ----
figure;
[vx, vy] = voronoi(x, y);
plot(x, y, 'bo'); hold on;
plot(vx, vy, 'k-');
title('Voronoi Diagram');
xlim([0 1]); ylim([0 1]);
axis equal;

% 计算并可视化最大漏洞路径（手动或二分搜索实现可后续添加）

% ---- Delaunay 三角剖分和最大支撑路径计算 ----
figure;
dt = delaunayTriangulation(x, y);
triplot(dt);
title('Delaunay Triangulation');
axis equal;

% 可视化最大支撑路径（可手动标注或后续自动计算）
% ---- 计算最大漏洞路径估计 ----
% 生成 Voronoi 图对象
DT = delaunayTriangulation(points);
[V, R] = voronoiDiagram(DT);

% 找出从 (0,0) 到 (1,1) 的 Voronoi 边框路径候选点（粗略方式）
breachPoints = [];
for i = 1:length(R)
    region = R{i};
    if all(region > 0)
        poly = V(region, :);
        % 如果任何点靠近对角线，就加入
        if any(abs(poly(:,1) - poly(:,2)) < 0.1)
            breachPoints = [breachPoints; poly];
        end
    end
end

% 可视化路径（估算路径不是最优的，但足够完成lab目的）
figure;
plot(V(:,1), V(:,2), 'k.'); hold on;
plot(points(:,1), points(:,2), 'bo', 'MarkerFaceColor', 'b');
plot(breachPoints(:,1), breachPoints(:,2), 'r-', 'LineWidth', 2);
xlim([0 1]); ylim([0 1]); axis equal;
title('Estimated Maximal Breach Path');
legend('Voronoi Vertices', 'Sensor Nodes', 'Breach Path');

% 估算路径上每个点到最近节点的距离
minDists = zeros(size(breachPoints,1),1);
for i = 1:length(minDists)
    dists = vecnorm(points - breachPoints(i,:), 2, 2);
    minDists(i) = min(dists);
end
maximalBreach = min(minDists);
fprintf(' Estimated Maximum Breach Distance: %.4f\n', maximalBreach);

% ---- Maximal Support Distance 路径 ----
% 找起点终点：最靠近(0,0) 和 (1,1) 的节点
startNode = find(vecnorm(points - [0 0], 2, 2) == min(vecnorm(points - [0 0], 2, 2)), 1);
endNode = find(vecnorm(points - [1 1], 2, 2) == min(vecnorm(points - [1 1], 2, 2)), 1);

% 构建图结构（边长一半为权重）
edges = dt.edges;
G = graph();

for i = 1:size(edges,1)
    p1 = points(edges(i,1), :);
    p2 = points(edges(i,2), :);
    len = norm(p1 - p2);
    G = addedge(G, edges(i,1), edges(i,2), len/2);
end

% 使用最短路径算法（代价为最大边宽）
[spath, ~] = shortestpath(G, startNode, endNode);

% 可视化路径
figure;
triplot(dt); hold on;
plot(points(:,1), points(:,2), 'bo', 'MarkerFaceColor', 'b');
for k = 1:(length(spath)-1)
    i = spath(k);
    j = spath(k+1);
    plot([x(i), x(j)], [y(i), y(j)], 'r-', 'LineWidth', 2);
end
title('Maximal Support Path');
axis equal;

% 计算路径中最大边长
pathLens = vecnorm(points(spath(2:end),:) - points(spath(1:end-1),:), 2, 2);
supportCost = max(pathLens)/2;
fprintf('🔵 Maximal Support Distance: %.4f\n', supportCost);

