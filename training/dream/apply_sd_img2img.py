#!/usr/bin/env python3
import os
import argparse
import shutil
from PIL import Image
import torch
from tqdm import tqdm
from diffusers import StableDiffusionImg2ImgPipeline

PROMPT = "white robot arm on light gray background, clean studio lighting, product photo, high quality, realistic"
NEGATIVE_PROMPT = "cartoon, synthetic, cgi, blurry, low quality, painting, simulation, gazebo, colorful background"

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data",        required=True)
    parser.add_argument("--output",      required=True)
    parser.add_argument("--max-samples", type=int, default=10000)
    parser.add_argument("--strength",    type=float, default=0.25)
    parser.add_argument("--guidance",    type=float, default=7.5)
    parser.add_argument("--steps",       type=int, default=20)
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = os.path.join(args.data, f)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(args.output, f))
            print(f"✅ Copié : {f}")

    print("Chargement SD img2img...")
    pipe = StableDiffusionImg2ImgPipeline.from_pretrained(
        "runwayml/stable-diffusion-v1-5",
        torch_dtype=torch.float16,
        safety_checker=None,
    ).to("cuda")
    pipe.enable_attention_slicing()
    print("✅ Pipeline SD chargé !")

    files = sorted([
        f for f in os.listdir(args.data)
        if f.endswith(".rgb.png") and not f.startswith("_")
    ])[:args.max_samples]

    errors = 0
    for fname in tqdm(files, desc="Applying SD img2img"):
        stem = fname.replace(".rgb.png", "")
        json_src = os.path.join(args.data, stem + ".json")
        json_dst = os.path.join(args.output, stem + ".json")
        if os.path.exists(json_src):
            shutil.copy2(json_src, json_dst)

        img_src = os.path.join(args.data, fname)
        img_dst = os.path.join(args.output, fname)

        try:
            init_image = Image.open(img_src).convert("RGB").resize((512, 512))
            result = pipe(
                prompt=PROMPT,
                negative_prompt=NEGATIVE_PROMPT,
                image=init_image,
                strength=args.strength,
                guidance_scale=args.guidance,
                num_inference_steps=args.steps,
            ).images[0]
            result = result.resize((640, 480))
            result.save(img_dst)
        except Exception as e:
            errors += 1
            shutil.copy2(img_src, img_dst)

    print(f"\n✅ TERMINÉ ! Traduit: {len(files)-errors} | Échecs: {errors}")

if __name__ == "__main__":
    main()