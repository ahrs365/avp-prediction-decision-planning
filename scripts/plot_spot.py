import matplotlib.pyplot as plt
import re


def parse_parking_spots(file_path):
    parking_spots = []
    with open(file_path, "r") as file:
        lines = file.readlines()
        for line in lines:
            if "Parking spot ID" in line:
                # 提取停车位ID和角点信息
                spot_id = int(re.search(r"Parking spot ID: (\d+)", line).group(1))
                corners = re.findall(r"Corner: \(([\d.]+), ([\d.]+)\)", line)
                corners = [(float(x), float(y)) for x, y in corners]
                parking_spots.append((spot_id, corners))
    return parking_spots


def plot_parking_spots(parking_spots, map_size):
    fig, ax = plt.subplots()
    for spot_id, corners in parking_spots:
        # 翻转Y坐标，使得原点在左下角
        flipped_corners = [(x, map_size[1] - y) for x, y in corners]
        polygon = plt.Polygon(flipped_corners, edgecolor="r", fill=None)
        ax.add_patch(polygon)
        # 添加停车位ID
        centroid_x = sum([corner[0] for corner in flipped_corners]) / len(
            flipped_corners
        )
        centroid_y = sum([corner[1] for corner in flipped_corners]) / len(
            flipped_corners
        )
        ax.text(
            centroid_x, centroid_y, str(spot_id), ha="center", va="center", fontsize=8
        )

    # 设置图形范围
    ax.set_xlim(0, map_size[0])
    ax.set_ylim(0, map_size[1])
    ax.set_aspect("equal")
    plt.show()


if __name__ == "__main__":
    # 读取并解析停车位数据
    parking_spots = parse_parking_spots("./data/spots.txt")
    map_size = (140, 80)  # 地图尺寸从文件中读取或在这里指定
    # 绘制停车位
    plot_parking_spots(parking_spots, map_size)
