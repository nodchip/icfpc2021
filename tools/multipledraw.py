import argparse
import os
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, Normalize

# An utility for saving muptiple-draw data as an image.
#
# e.g.
# [state.json]
# {
#   "human" : [[0, 5], [1, 5], [2, 0], [2, 1], [2, 2], [2, 3], [2, 4], [2, 5], [3, 2], [3, 3], [3, 4], [3, 5], [3, 6], [4, 0], [4, 1], [4, 2], [4, 3], [4, 4], [4, 5], [5, 5], [6, 5]],
#   "alien" : [[0, 3], [0, 4], [0, 6], [1, 1], [1, 2], [1, 3], [1, 4], [1, 5], [2, 1], [2, 5], [2, 6], [3, 0], [3, 1], [3, 4], [3, 5], [4, 1], [4, 5], [4, 6], [5, 1], [5, 2], [5, 3], [5, 4], [5, 5], [6, 3], [6, 4], [6, 6]],
#   "universe" : [[[0, 5], [1, 5], [2, 0], [2, 1], [2, 2], [2, 3], [2, 4], [2, 5], [3, 2], [3, 3], [3, 4], [3, 5], [3, 6], [4, 0], [4, 1], [4, 2], [4, 3], [4, 4], [4, 5], [5, 5], [6, 5]],
#               [[1, 7], [1, 8], [1, 10], [2, 5], [2, 6], [2, 7], [2, 8], [2, 9], [3, 5], [3, 9], [3, 10], [4, 4], [4, 5], [4, 8], [4, 9], [5, 5], [5, 9], [5, 10], [6, 5], [6, 6], [6, 7], [6, 8], [6, 9], [7, 7], [7, 8], [7, 10]],
#               [[4, 4], [4, 5], [5, 3], [5, 6], [6, 1], [6, 3], [6, 4], [6, 7], [7, 1], [7, 3], [7, 5], [7, 7], [8, 1], [8, 4], [8, 5], [8, 7], [9, 2], [9, 5], [10, 3], [10, 4]]]
# }
#
# $ python multipledraw.py state.json
# -> saved as 'human.png', 'alien.png' and 'universe.png', respectively.
# Use 'python multipledraw.py -h' for other options.


def get_list_dim(l, dims=1):
    if isinstance(l, list):
        for v in l:
            max_dims = max(dims, get_list_dim(v, dims + 1))
        return max_dims
    else:
        return 2


def get_colormap(max_colors, idx, max_alpha=0.4):
    w = np.linspace(0, 1, max_colors)[idx]
    if w < 0.5:
        wr = (0.5 - w) / 0.5
        wg = 1 - wr
        wb = 0
    else:
        wr = 0
        wg = (1.0 - w) / 0.5
        wb = 1 - wg

    vals = np.ones((256, 4))
    vals[:, 0] = np.linspace(1, wr, 256)
    vals[:, 1] = np.linspace(1, wg, 256)
    vals[:, 2] = np.linspace(1, wb, 256)
    vals[:, 3] = np.linspace(0.0, max_alpha, 256)
    return ListedColormap(vals)


def multiple_draw(json_path, img_size=-1, save_dir="", show=False):
    json_load = json.load(open(json_path))
    for save_name, value in json_load.items():
        fig = plt.figure()
        dims = get_list_dim(value)

        # single image
        if dims <= 2:
            arr = np.array(value)
            min_x, min_y = arr.min(axis=0)
            if min_x < 0 or min_y < 0:
                arr -= arr.min(axis=0)
            draw_size = img_size
            if draw_size <= 0:
                max_x, max_y = arr.max(axis=0)
                draw_size = max(6, max(max_x, max_y) + 1)

            plt_arr = np.zeros((draw_size, draw_size))
            for i in range(arr.shape[0]):
                y = arr[i, 1]
                x = arr[i, 0]
                if x < 0 or x >= draw_size or y < 0 or y >= draw_size:
                    continue
                plt_arr[y, x] += 1

            ax = fig.add_subplot(111)
            ax.tick_params(labelbottom=False,
                           labelleft=False,
                           labelright=False,
                           labeltop=False,
                           bottom=False,
                           left=False)
            ax.imshow(plt_arr, cmap=get_colormap(1, 0))
        # multiple images
        elif dims == 3:
            subarrs = []
            min_x = 0
            min_y = 0
            max_x = 0
            max_y = 0
            for v in value:
                subarrs.append(np.array(v))
                min_x_v, min_y_v = subarrs[-1].min(axis=0)
                min_x = min(min_x, min_x_v)
                min_y = min(min_y, min_y_v)
                max_x_v, max_y_v = subarrs[-1].max(axis=0)
                max_x = max(max_x, max_x_v)
                max_y = max(max_y, max_y_v)

            if min_x < 0 or min_y < 0:
                for subarr in subarrs:
                    subarr -= np.array([min_x, min_y])
                max_x -= min_x
                max_y -= min_y
            if img_size <= 0:
                draw_size = max(6, max(max_x, max_y) + 1)
            else:
                draw_size = img_size

            ax = fig.add_subplot(111)

            for j in range(len(value)):
                subarr = subarrs[j]
                plt_arr = np.zeros((draw_size, draw_size))
                for i in range(subarr.shape[0]):
                    y = subarr[i, 1]
                    x = subarr[i, 0]
                    if x < 0 or x >= draw_size or y < 0 or y >= draw_size:
                        continue
                    plt_arr[y, x] += 1
                ax.tick_params(labelbottom=False,
                               labelleft=False,
                               labelright=False,
                               labeltop=False,
                               bottom=False,
                               left=False)
                ax.imshow(plt_arr, cmap=get_colormap(len(value), j),
                          alpha=1.0, norm=Normalize(vmin=0, vmax=1))

        plt.savefig(os.path.join(save_dir, save_name))
        if show:
            plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Saves images from a list of muptiple-draw data.")
    parser.add_argument(
        "json_path", help="JSON file for drawing.")
    parser.add_argument(
        "--img_size",
        default=-
        1,
        type=int,
        help="Width of the image to be drawn. --img_size=16 -> saved as 16x16 pixels. If not specified, it will be adjusted automatically.")
    parser.add_argument(
        "--save_dir",
        default="",
        help="Directory where the files are saved.")
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open a figure window directly.")

    args = parser.parse_args()
    multiple_draw(
        args.json_path,
        args.img_size,
        args.save_dir,
        args.show)
