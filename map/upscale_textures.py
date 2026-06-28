import argparse
from pathlib import Path
import urllib.request

import cv2


def main():
    base_dir = Path(__file__).resolve().parent  # map/
    default_tile = base_dir / "LoD3-HH_Area4_2024_10_10" / "6734"

    parser = argparse.ArgumentParser(description="Upscale CityGML texture images with Real-ESRGAN")
    parser.add_argument("--tile", default=str(default_tile),
                        help="Path to tile directory containing images/")
    parser.add_argument("--scale", type=int, default=4, choices=[2, 3, 4], help="Upscale factor")
    parser.add_argument("--out", default="images_upscaled_x4", help="Output folder name inside tile")
    parser.add_argument("--ext", default=".png", help="Output image extension (.png or .jpg)")
    args = parser.parse_args()

    tile_dir = Path(args.tile)
    in_dir = tile_dir / "images"
    out_dir = tile_dir / args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    # Use OpenCV DNN superres with EDSR models (lightweight and reliable)
    model_dir = tile_dir / "weights"
    model_dir.mkdir(parents=True, exist_ok=True)
    if args.scale == 4:
        model_file = model_dir / "EDSR_x4.pb"
        url = "https://github.com/Saafke/EDSR_Tensorflow/raw/master/models/EDSR_x4.pb"
        model_name = "edsr"
    elif args.scale == 3:
        model_file = model_dir / "EDSR_x3.pb"
        url = "https://github.com/Saafke/EDSR_Tensorflow/raw/master/models/EDSR_x3.pb"
        model_name = "edsr"
    else:
        model_file = model_dir / "EDSR_x2.pb"
        url = "https://github.com/Saafke/EDSR_Tensorflow/raw/master/models/EDSR_x2.pb"
        model_name = "edsr"

    if not model_file.exists():
        print(f"Downloading model: {model_file.name}")
        urllib.request.urlretrieve(url, model_file)

    sr = cv2.dnn_superres.DnnSuperResImpl_create()
    sr.readModel(str(model_file))
    sr.setModel(model_name, args.scale)

    exts = {".jpg", ".jpeg", ".png"}
    images = [p for p in in_dir.rglob("*") if p.suffix.lower() in exts]
    if not images:
        raise SystemExit(f"No images found in {in_dir}")

    for img_path in images:
        rel = img_path.relative_to(in_dir)
        out_path = (out_dir / rel).with_suffix(args.ext)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if img is None:
            print(f"Skip unreadable: {img_path}")
            continue

        output = sr.upsample(img)
        cv2.imwrite(str(out_path), output)
        print(f"Upscaled: {img_path.name} -> {out_path}")

    print(f"Done. Upscaled images in: {out_dir}")


if __name__ == "__main__":
    main()
