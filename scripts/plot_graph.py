import matplotlib.pyplot as plt

# 从文件中读取节点和边的数据
nodes = []
edges = []
astar_path = []

with open(
    "/home/ahrs/workspace/nday/avp-prediction-decision-planning/data/graph.txt", "r"
) as file:
    for line in file:
        if "Vertex:" in line:
            coords = line.split("(")[1].split(")")[0].split(", ")
            nodes.append((float(coords[0]), float(coords[1])))
        elif "Connected to:" in line:
            coords = line.split("(")[1].split(")")[0].split(", ")
            edges.append((nodes[-1], (float(coords[0]), float(coords[1]))))
        elif line.strip().startswith("("):  # 检测A*路径坐标
            coords = line.strip().strip("()").split(", ")
            astar_path.append((float(coords[0]), float(coords[1])))

# 可视化
plt.figure(figsize=(12, 6))

# 绘制边
for edge in edges:
    x_values = [edge[0][0], edge[1][0]]
    y_values = [edge[0][1], edge[1][1]]
    plt.plot(x_values, y_values, color="gray", linewidth=1)

# 绘制节点
x_coords, y_coords = zip(*nodes)
plt.scatter(x_coords, y_coords, color="blue")

# 绘制A*路径
if astar_path:
    astar_x, astar_y = zip(*astar_path)
    plt.plot(astar_x, astar_y, color="red", linewidth=2, marker="o")

# 设置比例，保持x和y轴的比例一致
plt.gca().set_aspect("equal", adjustable="box")

plt.show()
