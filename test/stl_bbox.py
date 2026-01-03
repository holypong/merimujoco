#!/usr/bin/env python3
import struct
import sys
from pathlib import Path


def parse_stl(path):
    p = Path(path)
    if not p.exists():
        print(f"MISSING: {path}")
        return None
    with p.open('rb') as f:
        header = f.read(80)
        f.seek(0)
        data = f.read()
    # Try ASCII
    try:
        text = data.decode('utf-8')
        if text.lstrip().startswith('solid') and 'facet' in text:
            verts = []
            for line in text.splitlines():
                line = line.strip()
                if line.startswith('vertex'):
                    parts = line.split()
                    if len(parts) >= 4:
                        verts.append(tuple(float(x) for x in parts[1:4]))
            if verts:
                return verts
    except Exception:
        pass
    # Binary
    if len(data) < 84:
        return []
    # number of triangles at bytes 80..84
    ntri = struct.unpack('<I', data[80:84])[0]
    verts = []
    offset = 84
    rec_size = 50
    for i in range(ntri):
        if offset + rec_size > len(data):
            break
        rec = data[offset:offset+rec_size]
        vals = struct.unpack('<12fH', rec)
        # vals: normal (3), v1(3), v2(3), v3(3), attr
        v1 = vals[3:6]
        v2 = vals[6:9]
        v3 = vals[9:12]
        verts.extend([v1, v2, v3])
        offset += rec_size
    return verts


def bbox(verts):
    import math
    if not verts:
        return None
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


if __name__ == '__main__':
    files = [
        '/home/hori/mujoco/urdf/l_foot_large.stl',
        '/home/hori/mujoco/urdf/r_foot_large.stl'
    ]
    for f in files:
        verts = parse_stl(f)
        if verts is None:
            print(f"{f}: NOT FOUND")
            continue
        if len(verts) == 0:
            print(f"{f}: no vertices parsed")
            continue
        bb = bbox(verts)
        cnt = len(verts)
        centroid = (sum(v[0] for v in verts)/cnt, sum(v[1] for v in verts)/cnt, sum(v[2] for v in verts)/cnt)
        print(f"{f}: vertices={cnt}, centroid={centroid}")
        print(f"  min={bb[0]}, max={bb[1]}")
        # print first 3 verts for sanity
        print(f"  sample verts: {verts[0]}, {verts[1]}, {verts[2]}")
