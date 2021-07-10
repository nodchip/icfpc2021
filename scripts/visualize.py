#!/usr/bin/python3

import argparse
import json
import matplotlib.pyplot as plt
import os


def get_x(point):
    return point[0]


def get_y(point):
    return point[1]


def visualize(problem_file_path, pose_file_path, output_file_path):
    '''一つの問題を視覚化する。

    problem_file_path [必須] 問題ファイルパス。
    pose_file_path [任意] ポーズ (解答) ファイルパス。指定した場合、ポーズも描画する。
    output_file_path [任意] 出力画像ファイルパス。指定しない場合、ウィンドウに出力する。指定した場合、画像ファイルに出力する。
    '''
    with open(problem_file_path, 'r') as file:
        problem = json.load(file)

    # y軸を逆さまにする
    # Matplotlib-y軸が反転したグラフの描き方 | DATUM STUDIO株式会社 https://datumstudio.jp/blog/matplotlib-y%E8%BB%B8%E3%81%8C%E5%8F%8D%E8%BB%A2%E3%81%97%E3%81%9F%E3%82%B0%E3%83%A9%E3%83%95%E3%81%AE%E6%8F%8F%E3%81%8D%E6%96%B9/
    min_x = min(problem['hole'] + problem['figure']['vertices'], key=get_x)[0]
    max_x = max(problem['hole'] + problem['figure']['vertices'], key=get_x)[0]
    min_y = min(problem['hole'] + problem['figure']['vertices'], key=get_y)[1]
    max_y = max(problem['hole'] + problem['figure']['vertices'], key=get_y)[1]
    axes = plt.axes()
    axes.set_ylim([max_y + 1, min_y - 1])

    # 縦横比を1:1にする
    axes.set_aspect('equal')

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
        color = 'gray' if pose_file_path else 'red'
        plt.plot(*zip(src, dst), color=color)

    # poseを描画する
    if pose_file_path:
        epsilon = problem['epsilon']
        with open(pose_file_path, 'r') as file:
            pose = json.load(file)
        for edge in problem['figure']['edges']:
            src = problem['figure']['vertices'][edge[0]]
            dst = problem['figure']['vertices'][edge[1]]
            d0 = sum((s - d) ** 2 for s, d in zip(src, dst))
            src = pose['vertices'][edge[0]]
            dst = pose['vertices'][edge[1]]
            d1 = sum((s - d) ** 2 for s, d in zip(src, dst))
            color = 'blue'
            satisfied = 10**6 * abs(d1 - d0) <= d0 * epsilon
            if not satisfied:
                color = 'red' if d1 > d0 else 'cyan'
            plt.plot(*zip(src, dst), color=color)

    if output_file_path:
        plt.savefig(output_file_path)
    else:
        plt.show()

    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Brain Wall Visualizer')
    parser.add_argument('--problem', action='store',
                        type=str, help='Problem file path. Visualize one file if specified.')
    parser.add_argument('--pose', action='store',
                        type=str, help='Pose file path.')
    parser.add_argument('--problems', action='store', type=str,
                        help='Problem folder path. Use to convert json files to image files.')
    parser.add_argument('--output_image_directory_path', action='store', type=str,
                        help='Problem folder path. Use to convert json files to image files.')
    args = parser.parse_args()

    if args.problem:
        visualize(args.problem, args.pose, None)

    if args.problems:
        for problem_file_name in os.listdir(args.problems):
            print(problem_file_name)
            problem_file_path = os.path.join(args.problems, problem_file_name)
            output_file_name = os.path.splitext(problem_file_name)[0] + ".png"
            output_file_path = os.path.join(
                args.output_image_directory_path, output_file_name)
            visualize(problem_file_path, None, output_file_path)


if __name__ == '__main__':
    main()
