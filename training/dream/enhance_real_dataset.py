import cv2
import numpy as np
import shutil
from pathlib import Path
from tqdm import tqdm

INPUT_DIR = Path("dream/dream_data/real")
OUTPUT_DIR = Path("dream/dream_data/real_enhanced")

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}

DENOISE_STRENGTH = 3
CLAHE_CLIP_LIMIT = 2.5
SHARPEN_AMOUNT = 1.5
GAMMA_VALUE = 0.75
JPEG_QUALITY = 95

def apply_gray_world_white_balance(bgr_img):
    img = bgr_img.astype(np.float32)
    avg_b, avg_g, avg_r = np.mean(img[:,:,0]), np.mean(img[:,:,1]), np.mean(img[:,:,2])
    avg_gray = (avg_b + avg_g + avg_r) / 3.0
    img[:,:,0] *= avg_gray / (avg_b + 1e-6)
    img[:,:,1] *= avg_gray / (avg_g + 1e-6)
    img[:,:,2] *= avg_gray / (avg_r + 1e-6)
    return np.clip(img, 0, 255).astype(np.uint8)

def apply_gamma_correction(bgr_img, gamma=1.0):
    inv_gamma = 1.0 / gamma
    table = np.array([(i / 255.0) ** inv_gamma * 255 for i in range(256)]).astype("uint8")
    return cv2.LUT(bgr_img, table)

def enhance_image(bgr_img):
    img = apply_gray_world_white_balance(bgr_img)
    img = apply_gamma_correction(img, gamma=GAMMA_VALUE)
    img = cv2.fastNlMeansDenoisingColored(img, None, h=DENOISE_STRENGTH, hColor=DENOISE_STRENGTH, templateWindowSize=7, searchWindowSize=21)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=(8,8))
    img = cv2.cvtColor(cv2.merge((clahe.apply(l), a, b)), cv2.COLOR_LAB2BGR)
    blurred = cv2.GaussianBlur(img, (0,0), sigmaX=1.0)
    return np.clip(cv2.addWeighted(img, 1.0+SHARPEN_AMOUNT, blurred, -SHARPEN_AMOUNT, 0), 0, 255).astype(np.uint8)

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Copie les fichiers settings
    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = INPUT_DIR / f
        if src.exists():
            shutil.copy2(str(src), str(OUTPUT_DIR / f))
            print(f"✅ Copié : {f}")

    image_paths = [p for p in INPUT_DIR.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS]
    print(f"Images trouvées : {len(image_paths)}")

    for img_path in tqdm(list(image_paths)[:5], desc="Amélioration"):
        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if img is None:
            continue
        enhanced = enhance_image(img)
        save_path = OUTPUT_DIR / img_path.relative_to(INPUT_DIR)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(save_path), enhanced)

        # Copie le JSON correspondant
        json_src = INPUT_DIR / img_path.relative_to(INPUT_DIR).with_suffix('').with_suffix('.json')
        if json_src.exists():
            json_dst = OUTPUT_DIR / json_src.relative_to(INPUT_DIR)
            shutil.copy2(str(json_src), str(json_dst))

    print(f"✅ Terminé ! Sauvegardé dans : {OUTPUT_DIR.resolve()}")

if __name__ == "__main__":
    main()