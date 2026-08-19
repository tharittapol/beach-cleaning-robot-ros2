#!/usr/bin/env python3
import math
import os
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node


@dataclass
class RectArea:
    origin_x: float = 0.0
    origin_y: float = 0.0
    width: float = 20.0
    height: float = 15.0
    yaw: float = 0.0


def rot(x: float, y: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw); s = math.sin(yaw)
    return (c*x - s*y, s*x + c*y)


def inside_rot_rect(px: float, py: float, area: RectArea) -> bool:
    dx = px - area.origin_x
    dy = py - area.origin_y
    lx, ly = rot(dx, dy, -area.yaw)
    return (0.0 <= lx <= area.width) and (0.0 <= ly <= area.height)


class GenerateKeepoutMask(Node):
    def __init__(self):
        super().__init__('generate_keepout_mask')

        self.declare_parameter('output_dir', 'config')
        self.declare_parameter('mask_basename', 'keepout_mask')

        self.declare_parameter('area.origin_x', 0.0)
        self.declare_parameter('area.origin_y', 0.0)
        self.declare_parameter('area.width', 20.0)
        self.declare_parameter('area.height', 15.0)
        self.declare_parameter('area.yaw', 0.0)

        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('outer_margin', 5.0)

        self.output_dir = str(self.get_parameter('output_dir').value)
        self.mask_basename = str(self.get_parameter('mask_basename').value)

        self.area = RectArea(
            origin_x=float(self.get_parameter('area.origin_x').value),
            origin_y=float(self.get_parameter('area.origin_y').value),
            width=float(self.get_parameter('area.width').value),
            height=float(self.get_parameter('area.height').value),
            yaw=float(self.get_parameter('area.yaw').value),
        )
        self.res = float(self.get_parameter('resolution').value)
        self.outer = float(self.get_parameter('outer_margin').value)

        self._run_once()

    def _run_once(self):
        os.makedirs(self.output_dir, exist_ok=True)
        pgm_path = os.path.join(self.output_dir, f'{self.mask_basename}.pgm')
        yaml_path = os.path.join(self.output_dir, f'{self.mask_basename}.yaml')

        # bounds from rotated corners
        corners_local = [(0,0), (self.area.width,0), (self.area.width,self.area.height), (0,self.area.height)]
        corners_map = []
        for lx, ly in corners_local:
            rx, ry = rot(lx, ly, self.area.yaw)
            corners_map.append((self.area.origin_x + rx, self.area.origin_y + ry))
        xs = [p[0] for p in corners_map]
        ys = [p[1] for p in corners_map]
        minx = min(xs) - self.outer
        maxx = max(xs) + self.outer
        miny = min(ys) - self.outer
        maxy = max(ys) + self.outer

        w_px = int(math.ceil((maxx - minx) / self.res))
        h_px = int(math.ceil((maxy - miny) / self.res))

        # start as keepout (black=0), then paint inside boundary as free (white=254)
        img = np.zeros((h_px, w_px), dtype=np.uint8)
        for iy in range(h_px):
            y = miny + (iy + 0.5) * self.res
            for ix in range(w_px):
                x = minx + (ix + 0.5) * self.res
                if inside_rot_rect(x, y, self.area):
                    img[h_px - 1 - iy, ix] = 254

        self._write_pgm(pgm_path, img)

        # origin yaw MUST be 0 for nav2 maps
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(f"""image: {os.path.basename(pgm_path)}
                    resolution: {self.res}
                    origin: [{minx}, {miny}, 0.0]
                    negate: 0
                    occupied_thresh: 0.65
                    free_thresh: 0.196
                    mode: trinary
                    """)

        self.get_logger().info(f'Generated mask:\n  {pgm_path}\n  {yaml_path}\n  size={w_px}x{h_px}px res={self.res}m/px')

    def _write_pgm(self, path: str, img: np.ndarray):
        h, w = img.shape
        header = f'P5\n{w} {h}\n255\n'.encode('ascii')
        with open(path, 'wb') as f:
            f.write(header)
            f.write(img.tobytes())


def main():
    rclpy.init()
    node = GenerateKeepoutMask()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
