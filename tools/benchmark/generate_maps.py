"""从 Gazebo world 的静态 box 几何生成 Nav2 静态栅格地图（PGM + YAML）。

Nav2 对照臂需要每个场景的先验地图，而仓库里只有 detour_test 手工地图。
这里直接解析 SDF 里的 static box（墙体/静态障碍），按 0.05m 栅格化，
保证地图与 world 几何一致；动态障碍（如 obstacle_course 的箱子）不进入地图，
正好用来检验局部避障能力。
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

FREE = 254
OCCUPIED = 0


def _floats(text, default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
    if not text:
        return default
    return tuple(float(v) for v in text.split())


def load_static_boxes(world_path):
    """解析 world，返回 [(cx, cy, yaw, sx, sy), ...]（只含 static box）。"""
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    if world is None:
        raise ValueError(f'{world_path} 中没有 <world> 元素')

    boxes = []
    for model in world.findall('model'):
        static = (model.findtext('static') or '').strip().lower() == 'true'
        if not static:
            continue
        size_el = model.find('.//collision/geometry/box/size')
        if size_el is None:
            continue  # 地面是 plane，不是 box
        sx, sy, _ = _floats(size_el.text, default=(0.0, 0.0, 0.0))
        if sx <= 0.0 or sy <= 0.0:
            continue
        cx, cy, _, _, _, yaw = _floats(model.findtext('pose'))
        boxes.append((cx, cy, yaw, sx, sy))
    return boxes


def _corners(box):
    cx, cy, yaw, sx, sy = box
    cos_a, sin_a = math.cos(yaw), math.sin(yaw)
    result = []
    for dx, dy in ((-sx / 2, -sy / 2), (-sx / 2, sy / 2),
                   (sx / 2, sy / 2), (sx / 2, -sy / 2)):
        result.append((cx + dx * cos_a - dy * sin_a,
                       cy + dx * sin_a + dy * cos_a))
    return result


def _inside(box, x, y):
    cx, cy, yaw, sx, sy = box
    dx, dy = x - cx, y - cy
    cos_a, sin_a = math.cos(-yaw), math.sin(-yaw)
    lx = dx * cos_a - dy * sin_a
    ly = dx * sin_a + dy * cos_a
    return abs(lx) <= sx / 2.0 and abs(ly) <= sy / 2.0


def rasterize(boxes, resolution=0.05, margin=0.5):
    """把 boxes 栅格化，返回 (pixels, origin_x, origin_y, width, height)。

    pixels 按 PGM 行序（第 0 行是世界坐标 y 最大的一行）。
    """
    if not boxes:
        raise ValueError('没有可用于建图的静态 box')
    xs = [p[0] for b in boxes for p in _corners(b)]
    ys = [p[1] for b in boxes for p in _corners(b)]
    xmin, xmax = min(xs) - margin, max(xs) + margin
    ymin, ymax = min(ys) - margin, max(ys) + margin

    width = max(1, int(math.ceil((xmax - xmin) / resolution)))
    height = max(1, int(math.ceil((ymax - ymin) / resolution)))

    pixels = bytearray([FREE]) * (width * height)
    for iy in range(height):
        y = ymin + (height - 1 - iy + 0.5) * resolution
        for ix in range(width):
            x = xmin + (ix + 0.5) * resolution
            if any(_inside(b, x, y) for b in boxes):
                pixels[iy * width + ix] = OCCUPIED
    return bytes(pixels), xmin, ymin, width, height


def write_map(pixels, origin_x, origin_y, width, height, out_prefix,
              resolution=0.05):
    """写出 <out_prefix>.pgm 和 <out_prefix>.yaml，返回 yaml 路径。"""
    os.makedirs(os.path.dirname(os.path.abspath(out_prefix)), exist_ok=True)
    pgm_path = out_prefix + '.pgm'
    yaml_path = out_prefix + '.yaml'
    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    with open(pgm_path, 'wb') as f:
        f.write(header)
        f.write(pixels)
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(f'# 由 tools/benchmark/generate_maps.py 从 world 自动生成\n')
        f.write(f'image: {os.path.basename(pgm_path)}\n')
        f.write(f'resolution: {resolution}\n')
        f.write(f'origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')
    return yaml_path


def generate_for_world(world_path, out_dir, resolution=0.05, margin=0.5):
    """world → 地图，返回 yaml 绝对路径。"""
    boxes = load_static_boxes(world_path)
    pixels, ox, oy, w, h = rasterize(boxes, resolution, margin)
    stem = os.path.splitext(os.path.basename(world_path))[0]
    prefix = os.path.join(out_dir, f'{stem}_map')
    return write_map(pixels, ox, oy, w, h, prefix, resolution)


def main():
    parser = argparse.ArgumentParser(description='由 Gazebo world 生成 Nav2 静态地图')
    parser.add_argument(
        '--worlds-dir',
        default=os.path.join(
            os.path.dirname(__file__), '..', '..',
            'src', 'robot_description', 'worlds'),
        help='world 文件目录')
    parser.add_argument(
        '--out', default=os.path.join(os.path.dirname(__file__), 'maps'),
        help='输出目录')
    parser.add_argument('--world', action='append', default=None,
                        help='只处理指定 world 文件名（可多次指定）')
    args = parser.parse_args()

    names = args.world or [
        f for f in sorted(os.listdir(args.worlds_dir)) if f.endswith('.world')]
    for name in names:
        path = os.path.join(args.worlds_dir, name)
        yaml_path = generate_for_world(path, args.out)
        print(f'{name} -> {yaml_path}')


if __name__ == '__main__':
    main()
