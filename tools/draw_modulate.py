import argparse
import os
import glob
import re
import numpy as np
import matplotlib.pyplot as plt
from modulate import demodulate_readable

# An utility for saving a distorted list as an image.
# $ python draw_modulate.py 111101001011110100110001011110100110010011110100110011011110110000101100001111101100001011000111111011000010110010111110110001001011110110001001100010111101100010011001001111011000100110011011110110001101100001111101100011011000111111011000110110010111110110010001011110110010001100010111101100100011001001111011001000110011011110110010101100001111101100101011000111111011001010110010111110110011001011110110011001100010111101100110011001001111011001100110011000
# -> saved as 'state_000.png'
# Use 'python draw_modulate.py -h' for other options.


def draw_picture(bit, img_size=-1, save_dir="", save_name="", show=False):
    value = demodulate_readable(bit)
    arr = np.array(value)
    if img_size <= 0:
        arr -= arr.min(axis=0)
        max_x, max_y = arr.max(axis=0)
        img_size = max(6, max(max_y, max_x) + 1)
    plt_arr = np.zeros((img_size, img_size))
    for i in range(arr.shape[0]):
        x = arr[i, 0]
        y = arr[i, 1]
        if x < 0 or x >= img_size or y < 0 or y >= img_size:
            continue
        plt_arr[y, x] += 1

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.tick_params(labelbottom=False,
                   labelleft=False,
                   labelright=False,
                   labeltop=False,
                   bottom=False,
                   left=False)
    ax.imshow(plt_arr, "Greys")
    if save_name == "":
        base_name = "state"
        exist_files = glob.glob(
            os.path.join(
                save_dir,
                "{}_*.png".format(base_name)))
        if len(exist_files) == 0:
            file_idx = 0
        else:
            latest_file = sorted(exist_files)[-1]
            file_idx = int(re.split("[_]", latest_file[:-4])[-1])
            file_idx += 1
        save_name = "{}_{:03}".format(base_name, file_idx)

    plt.savefig(os.path.join(save_dir, save_name))
    if show:
        plt.show()


def test_checker_board():
    checker_bit = "111101001011110100110001011110100110010011110100110011011110110000101100001111101100001011000111111011000010110010111110110001001011110110001001100010111101100010011001001111011000100110011011110110001101100001111101100011011000111111011000110110010111110110010001011110110010001100010111101100100011001001111011001000110011011110110010101100001111101100101011000111111011001010110010111110110011001011110110011001100010111101100110011001001111011001100110011000"
    draw_picture(checker_bit)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Saves an image from a list of modulated coordinates.")
    parser.add_argument(
        "bit", help="Modulated binary representation. e.g. 11110...")
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
        "--save_name",
        default="",
        help="File name. --save_name=draw001 -> saved as draw001.png. If you don't specify it, it is automatically saved with a sequential number.")
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open a figure window directly.")

    args = parser.parse_args()
    draw_picture(
        args.bit,
        args.img_size,
        args.save_dir,
        args.save_name,
        args.show)
