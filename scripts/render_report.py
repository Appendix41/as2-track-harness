#!/usr/bin/env python3
"""Render one saved shadow run into a single PNG comparison figure.

Requires Pillow (install with ``pip install -e '.[vision]'``). The input must be
a run directory under this repository's ``runs/``; nothing is written outside
it and no camera, API or robot contact is made.
"""
import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'


def font(size):
    try:
        return ImageFont.truetype(FONT, size)
    except OSError:
        return ImageFont.load_default()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory')
    parser.add_argument('--output', default='comparison.png')
    args = parser.parse_args()
    run = Path(args.run_directory).resolve()
    if not run.is_relative_to(ROOT / 'runs'):
        raise SystemExit('invalid output directory')
    state = json.loads((run / 'latest.json').read_text())
    canvas = Image.new('RGB', (1440, 820), '#111827')
    draw = ImageDraw.Draw(canvas)
    big, small = font(22), font(17)
    draw.text((30, 20), 'VLX + slow planner | SHADOW ONLY | Robot commands: 0',
              font=big, fill='white')
    draw.text((30, 64), 'Captured request image + model image-space route',
              font=small, fill='#cbd5e1')
    request_id = state.get('request_id', '')
    image_path = run / ('request_' + request_id + '.jpg')
    request_image = None
    if request_id.isalnum() and image_path.is_file():
        try:
            request_image = Image.open(image_path).convert('RGB').resize((640, 480))
        except (OSError, ValueError):
            request_image = None
    if request_image is not None:
        canvas.paste(request_image, (30, 110))
        if state.get('slow_frame') == 'image_uv':
            points = [(30 + int(p[0] * 640), 110 + int(p[1] * 480))
                      for p in state.get('slow_raw', [])]
            if len(points) > 1:
                draw.line(points, fill='#fbbf24', width=5)
            for index, (x, y) in enumerate(points):
                draw.ellipse((x - 6, y - 6, x + 6, y + 6), fill='#fbbf24')
                draw.text((x + 8, y - 12), str(index + 1), font=small, fill='#fbbf24')
    else:
        draw.text((40, 160), 'No readable request image for this run',
                  font=big, fill='#fbbf24')
    draw.text((740, 64), 'Current local frame | VLX blue, recommendation green',
              font=small, fill='#cbd5e1')
    draw.rectangle((730, 110, 1400, 590), fill='#0b1322', outline='#475569')
    extent = max([1.] + [abs(v) for key in ('fast', 'slow_aligned', 'recommended')
                         for point in state.get(key, []) for v in point[:2]]) * 1.2
    scale = 350 / extent
    ox, oy = 1065, 550
    for key, color in (('fast', '#60a5fa'), ('slow_aligned', '#c084fc'),
                       ('recommended', '#34d399')):
        points = [(ox - p[1] * scale, oy - p[0] * scale) for p in state.get(key, [])]
        if len(points) > 1:
            draw.line(points, fill=color, width=4)
    draw.ellipse((ox - 5, oy - 5, ox + 5, oy + 5), fill='white')
    lines = [
        'Result: ' + str(state.get('reason')),
        'Plan age: ' + str(state.get('plan_age_s')) + ' s | Local loop: '
        + str(round(state.get('loop_hz', 0), 2)) + ' Hz | Source: '
        + str(round(state.get('source_hz', 0), 2)) + ' Hz',
        'Image-space navigation points require calibrated geometry before metric fusion.',
        'This figure does not establish target selection accuracy, collision avoidance '
        'or navigation success.',
    ]
    for index, line in enumerate(lines):
        draw.text((30, 620 + 38 * index), line, font=small,
                  fill='#fbbf24' if index == 0 else '#cbd5e1')
    output = run / args.output
    canvas.save(output)
    print(json.dumps({'output': str(output), 'request_id': request_id}))


if __name__ == '__main__':
    main()
