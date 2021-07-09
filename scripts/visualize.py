import argparse
import json
import matplotlib.pyplot as plt

def get_x(point):
    return point[0]


def get_y(point):
    return point[1]


def visualize(problem_file_path, pose_file_path):
    with open(problem_file_path, 'r') as file:
        problem = json.load(file)

    # y軸を逆さまにする
    # Matplotlib-y軸が反転したグラフの描き方 | DATUM STUDIO株式会社 https://datumstudio.jp/blog/matplotlib-y%E8%BB%B8%E3%81%8C%E5%8F%8D%E8%BB%A2%E3%81%97%E3%81%9F%E3%82%B0%E3%83%A9%E3%83%95%E3%81%AE%E6%8F%8F%E3%81%8D%E6%96%B9/
    min_x = min(problem['hole'] + problem['figure']['vertices'], key=get_x)[0]
    max_x = max(problem['hole'] + problem['figure']['vertices'], key=get_x)[0]
    min_y = min(problem['hole'] + problem['figure']['vertices'], key=get_y)[1]
    max_y = max(problem['hole'] + problem['figure']['vertices'], key=get_y)[1]
    plt.axes().set_ylim([max_y + 1, min_y - 1])

    # holeを描画する
    for index in range(0, len(problem['hole'])):
        src = problem['hole'][index]
        dst = problem['hole'][(index + 1) % len(problem['hole'])]
        # plt.plot() はx座標の列とy座標の列を受け取るので転置する
        src, dst = [src[0], dst[0]], [src[1], dst[1]]
        plt.plot(src, dst, color='black')

    # figureを描画する
    for edge in problem['figure']['edges']:
        src = problem['figure']['vertices'][edge[0]]
        dst = problem['figure']['vertices'][edge[1]]
        src, dst = [src[0], dst[0]], [src[1], dst[1]]
        plt.plot(src, dst, color='red')

    # poseを描画する
    if pose_file_path:
        with open(pose_file_path, 'r') as file:
            pose = json.load(file)
        for edge in pose['vertices']:
            src = problem['figure']['vertices'][edge[0]]
            dst = problem['figure']['vertices'][edge[1]]
            plt.plot(src, dst, color='blue')

    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Brain Wall Visualizer')
    parser.add_argument('--problem', action='store', type=str,
                        required=True, help='Problem file path.')
    parser.add_argument('--pose', action='store',
                        type=str, help='Pose file path.')
    args = parser.parse_args()

    visualize(args.problem, args.pose)


if __name__ == '__main__':
    main()
