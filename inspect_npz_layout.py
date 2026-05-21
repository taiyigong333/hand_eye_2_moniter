#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import numpy as np
import cv2


def safe_shape(x):
    try:
        return tuple(x.shape)
    except Exception:
        return None


def summarize_array(name, arr):
    info = {
        'key': name,
        'dtype': str(getattr(arr, 'dtype', type(arr))),
        'shape': safe_shape(arr),
    }
    if isinstance(arr, np.ndarray):
        info['ndim'] = arr.ndim
        if arr.dtype != object and arr.size > 0:
            try:
                info['min'] = float(np.min(arr))
                info['max'] = float(np.max(arr))
            except Exception:
                pass
    return info


def is_image_like(a):
    if not isinstance(a, np.ndarray):
        return False
    if a.dtype == object:
        return False
    if a.ndim == 2:
        return True
    if a.ndim == 3 and a.shape[-1] in (1, 3, 4):
        return True
    return False


def save_image(path, arr):
    arr = np.asarray(arr)
    if arr.dtype != np.uint8:
        arr = arr.astype(np.float32)
        mn, mx = float(arr.min()), float(arr.max())
        if mx > mn:
            arr = (255.0 * (arr - mn) / (mx - mn)).clip(0, 255).astype(np.uint8)
        else:
            arr = np.zeros_like(arr, dtype=np.uint8)
    if arr.ndim == 3 and arr.shape[-1] == 3:
        cv2.imwrite(str(path), cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
    else:
        cv2.imwrite(str(path), arr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz_path')
    ap.add_argument('--index', type=int, default=0)
    ap.add_argument('--out_dir', default='./npz_inspect')
    args = ap.parse_args()

    npz = np.load(args.npz_path, allow_pickle=True)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print('=== keys ===')
    for k in npz.files:
        arr = npz[k]
        print(json.dumps(summarize_array(k, arr), ensure_ascii=False))

    print('\n=== candidates: top-level image-like arrays ===')
    top_candidates = []
    for k in npz.files:
        arr = npz[k]
        if is_image_like(arr):
            top_candidates.append((k, arr))
        elif isinstance(arr, np.ndarray) and arr.dtype != object and arr.ndim >= 3:
            # maybe batched images, like (N,H,W,C) or (N,2,H,W,C)
            print(f'{k}: possible batched data shape={arr.shape} dtype={arr.dtype}')

    if top_candidates:
        for k, arr in top_candidates:
            print(f'{k}: image-like shape={arr.shape} dtype={arr.dtype}')
            save_image(out_dir / f'{k}_top.png', arr)

    # Inspect object arrays/dicts deeply
    print('\n=== object-array / nested inspection ===')
    for k in npz.files:
        arr = npz[k]
        if not (isinstance(arr, np.ndarray) and arr.dtype == object and arr.size > 0):
            continue
        print(f'[{k}] object array shape={arr.shape}')
        try:
            first = arr.flat[args.index].item() if hasattr(arr.flat[args.index], 'item') else arr.flat[args.index]
        except Exception:
            first = arr.flat[0]
        print(' first element type:', type(first))
        if isinstance(first, dict):
            print(' dict keys:', list(first.keys()))
            for kk, vv in first.items():
                if isinstance(vv, np.ndarray):
                    print('  ', kk, 'shape=', vv.shape, 'dtype=', vv.dtype)
                    if is_image_like(vv):
                        save_image(out_dir / f'{k}_{kk}.png', vv)
                else:
                    print('  ', kk, 'type=', type(vv), 'value_preview=', str(vv)[:120])
        else:
            print(' value preview:', str(first)[:300])

    # Try common batched layouts and save sample 0 previews
    print('\n=== sample image probes ===')
    for k in npz.files:
        arr = npz[k]
        if not isinstance(arr, np.ndarray) or arr.dtype == object:
            continue
        shp = arr.shape
        try:
            if arr.ndim == 4 and shp[-1] in (1,3,4):
                # (N,H,W,C)
                sample = arr[args.index]
                save_image(out_dir / f'{k}_sample{args.index}.png', sample)
                print(f'{k}: saved as (N,H,W,C) sample -> {sample.shape}')
            elif arr.ndim == 5 and shp[-1] in (1,3,4):
                # (N,2,H,W,C) maybe two cameras
                for c in range(min(shp[1], 4)):
                    sample = arr[args.index, c]
                    save_image(out_dir / f'{k}_sample{args.index}_cam{c}.png', sample)
                print(f'{k}: saved as (N,Cam,H,W,C) sample -> {arr[args.index].shape}')
            elif arr.ndim == 4 and shp[1] in (1,2,3,4):
                # (N,C,H,W)
                sample = arr[args.index]
                for c in range(min(sample.shape[0], 4)):
                    save_image(out_dir / f'{k}_sample{args.index}_ch{c}.png', sample[c])
                print(f'{k}: saved as (N,C,H,W) channels -> {sample.shape}')
            elif arr.ndim == 3:
                # maybe one grayscale image or (N,H,W)
                if args.index < shp[0]:
                    sample = arr[args.index]
                    if isinstance(sample, np.ndarray) and sample.ndim == 2:
                        save_image(out_dir / f'{k}_sample{args.index}.png', sample)
                        print(f'{k}: saved as (N,H,W) sample -> {sample.shape}')
        except Exception as e:
            print(f'{k}: probe failed: {e}')

    print(f'\nSaved previews to: {out_dir}')


if __name__ == '__main__':
    main()
