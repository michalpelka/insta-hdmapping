#!/usr/bin/env python3
"""
Generate and display a ChArUco board sized to fill a 4K screen exactly,
for fisheye calibration / reprojection verification.

Usage:
    python show_charuco_4k.py                     # generate + fullscreen display
    python show_charuco_4k.py --save-only          # just write the PNG
    python show_charuco_4k.py --cols 12 --rows 7   # custom board layout
    python show_charuco_4k.py --list-dicts         # list available ArUco dicts

Requires: opencv-contrib-python >= 4.7 (for cv2.aruco.CharucoBoard.generateImage)
    pip install opencv-contrib-python --break-system-packages
"""

import argparse
import sys

import cv2
import numpy as np

DICT_NAMES = {
    "4X4_50": cv2.aruco.DICT_4X4_50,
    "5X5_100": cv2.aruco.DICT_5X5_100,
    "6X6_250": cv2.aruco.DICT_6X6_250,
    "7X7_1000": cv2.aruco.DICT_7X7_1000,
    "APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def build_board(cols, rows, dict_name, screen_w, screen_h, margin_px):
    """Build a ChArUco board sized so squares are as large as possible
    while fitting the given pixel canvas, then render it at exactly
    that resolution (so 1 image pixel == 1 screen pixel, no rescale)."""

    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_NAMES[dict_name])

    usable_w = screen_w - 2 * margin_px
    usable_h = screen_h - 2 * margin_px
    square_px = min(usable_w // cols, usable_h // rows)
    marker_px = int(square_px * 0.7)  # marker_length < square_length, required by API

    # square_length / marker_length are in "arbitrary units" (mm if you like);
    # we keep them numerically equal to pixel size so 1 unit = 1 px at render time.
    board = cv2.aruco.CharucoBoard(
        (cols, rows),
        squareLength=float(square_px),
        markerLength=float(marker_px),
        dictionary=aruco_dict,
    )

    board_w_px = cols * square_px
    board_h_px = rows * square_px
    img_size = (board_w_px, board_h_px)

    board_img = board.generateImage(img_size, marginSize=0, borderBits=1)

    # Composite onto full white 4K canvas, centered, with quiet margin.
    canvas = np.full((screen_h, screen_w), 255, dtype=np.uint8)
    x0 = (screen_w - board_w_px) // 2
    y0 = (screen_h - board_h_px) // 2
    canvas[y0:y0 + board_h_px, x0:x0 + board_w_px] = board_img

    return board, canvas, square_px, marker_px


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cols", type=int, default=14, help="chessboard squares, columns (default 14)")
    ap.add_argument("--rows", type=int, default=8, help="chessboard squares, rows (default 8)")
    ap.add_argument("--dict", dest="dict_name", default="5X5_100", choices=DICT_NAMES.keys(),
                     help="ArUco dictionary (default 5X5_100 — good balance of ID count vs detectability)")
    ap.add_argument("--width", type=int, default=3840, help="screen width px (default 3840)")
    ap.add_argument("--height", type=int, default=2160, help="screen height px (default 2160)")
    ap.add_argument("--margin", type=int, default=40, help="quiet white margin around board, px (default 40)")
    ap.add_argument("--out", default="/mnt/user-data/outputs/charuco_4k.png", help="output PNG path")
    ap.add_argument("--save-only", action="store_true", help="don't open a display window, just save PNG")
    ap.add_argument("--list-dicts", action="store_true", help="list available dictionaries and exit")
    args = ap.parse_args()

    if args.list_dicts:
        for name in DICT_NAMES:
            print(name)
        return

    board, canvas, square_px, marker_px = build_board(
        args.cols, args.rows, args.dict_name, args.width, args.height, args.margin
    )

    cv2.imwrite(args.out, canvas)
    print(f"Saved {args.out}  ({args.width}x{args.height}, {args.cols}x{args.rows} squares, "
          f"square={square_px}px, marker={marker_px}px, dict={args.dict_name})")
    print("IMPORTANT: on the TV, force pixel-exact scaling (e.g. 'PC mode' / 'Just Scan' / "
          "'1:1 pixel mapping') or the known square size will be wrong due to overscan.")

    # Save calibration metadata alongside the image so the capture/analysis
    # script can reconstruct object points without re-deriving square size.
    meta_path = args.out.rsplit(".", 1)[0] + "_meta.txt"
    with open(meta_path, "w") as f:
        f.write(f"cols={args.cols}\nrows={args.rows}\ndict={args.dict_name}\n"
                f"square_px={square_px}\nmarker_px={marker_px}\n"
                f"screen_w={args.width}\nscreen_h={args.height}\nmargin={args.margin}\n")
    print(f"Saved {meta_path}")

    if not args.save_only:
        window = "charuco_4k"
        cv2.namedWindow(window, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow(window, canvas)
        print("Press any key in the display window (or ESC) to close.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
