#!/usr/bin/env python3
"""Apply ControlNet Canny + IP-Adapter on synthetic Gazebo images.
Usage:
  python dream/apply_controlnet.py \
    --data dream/dream_data/synthetic \
    --output dream/dream_data/synth_real_style_sd \
    --ref-image dream/mycobot_reference.jpg \
    --max-samples 10000
"""
import os
import argparse
import shutil
import cv2
import numpy as np
from PIL import Image
import torch
from tqdm import tqdm
from diffusers import StableDiffusionControlNetPipeline, ControlNetModel, UniPCMultistepScheduler
from diffusers.models.attention_processor import AttnProcessor2_0

PROMPT = "Ultra-realistic robotics laboratory, modern office environment, " \
"collaborative robot arm mounted on a white desk in the foreground, " \
"compact white 6-axis robotic manipulator with smooth glossy plastic casing," \
" industrial cobot base fixed to the table, black cables connected to the robot, " \
"computer workstation on the left side, engineer sitting and working on a laptop, " \
"focused posture, server racks and networking equipment in the background, " \
"large bright window creating strong natural sunlight, " \
"realistic shadows cast across gray patterned floor tiles, " \
"green high stool near the window," \
" emergency stop button on the desk, clean research lab, " \
"robotics startup workspace, engineering office, " \
"photorealistic industrial scene, " \
"highly detailed mechanical components," \
" realistic materials, cinematic lighting, " \
"global illumination, physically based rendering, " \
"ultra sharp focus, 8k, HDR, " \
"professional industrial photography, " \
"natural colors, " \
"depth realism, " \
"accurate reflections, " \
"documentary style, engineering environment, high dynamic range, realistic perspective, no distortion, production robotics laboratory."

NEGATIVE_PROMPT = "cartoon, anime, illustration, painting, sketch, low quality, blurry, noisy, oversaturated colors, \
distorted robot arm, extra robotic arms, floating objects, duplicated objects, deformed cables, incorrect perspective, \
unrealistic lighting, low resolution, watermark, text, logo, cropped robot, missing workstation, \
missing engineer, bad anatomy, warped desk, unrealistic shadows, \
CGI artifacts, melted geometry, fisheye distortion, motion blur, overexposed highlights, underexposed scene"

def get_canny(image_pil, low=100, high=200):
    img_np = np.array(image_pil.convert("RGB"))
    gray = cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, low, high)
    edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
    return Image.fromarray(edges_rgb)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data",            required=True)
    parser.add_argument("--output",          required=True)
    parser.add_argument("--ref-images", required=True, nargs="+", help="1 ou plusieurs photos réelles du mycobot")
    parser.add_argument("--max-samples",     type=int,   default=10000)
    parser.add_argument("--guidance",        type=float, default=5.0)   # réduit vs 7.5
    parser.add_argument("--steps",           type=int,   default=25)
    parser.add_argument("--ip-scale",        type=float, default=0.6,   help="Force IP-Adapter (0.4-0.8)")
    parser.add_argument("--cnet-scale",      type=float, default=1.2,   help="Force ControlNet Canny (1.0-1.5)")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = os.path.join(args.data, f)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(args.output, f))
            print(f"✅ Copié : {f}")

   
    # Charger et fusionner les images de référence en une seule
    ref_list = [Image.open(p).convert("RGB").resize((512, 512)) for p in args.ref_images]
    ref_array = np.mean([np.array(img).astype(float) for img in ref_list], axis=0).astype(np.uint8)
    ref_images = Image.fromarray(ref_array)
    print(f"✅ {len(ref_list)} image(s) fusionnées en une référence : {args.ref_images}")

    print("Chargement ControlNet...")
    controlnet = ControlNetModel.from_pretrained(
        "/tmp/controlnet-canny",
        torch_dtype=torch.float16
    )
    pipe = StableDiffusionControlNetPipeline.from_pretrained(
        "runwayml/stable-diffusion-v1-5",
        controlnet=controlnet,
        torch_dtype=torch.float16,
        safety_checker=None,
    ).to("cuda")
    pipe.scheduler = UniPCMultistepScheduler.from_config(pipe.scheduler.config)
    # Reset attention processors AVANT IP-Adapter (évite le conflit SlicedAttnProcessor)
    pipe.unet.set_attn_processor(AttnProcessor2_0())
    
    # Charger IP-Adapter
    print("Chargement IP-Adapter...")
    pipe.load_ip_adapter("h94/IP-Adapter", subfolder="models", weight_name="ip-adapter_sd15.bin")
    pipe.set_ip_adapter_scale(args.ip_scale)
    print(f"✅ Pipeline chargé ! (IP-Adapter scale={args.ip_scale}, ControlNet scale={args.cnet_scale})")
    # Charger LoRA (style bureau réel)
    pipe.load_lora_weights("dream/lora_mycobot")
    pipe.fuse_lora(lora_scale=0.8)
    print("✅ LoRA chargé !")
    torch.cuda.empty_cache() 

    files = sorted([
        f for f in os.listdir(args.data)
        if f.endswith(".rgb.png") and not f.startswith("_")
    ])[:args.max_samples]

    print(f"\nTraduction de {len(files)} images synth → style réel ControlNet+IP-Adapter...")
    errors = 0
    for fname in tqdm(files, desc="Applying ControlNet+IP-Adapter"):
        stem = fname.replace(".rgb.png", "")
        json_src = os.path.join(args.data, stem + ".json")
        json_dst = os.path.join(args.output, stem + ".json")
        if os.path.exists(json_src):
            shutil.copy2(json_src, json_dst)

        img_src = os.path.join(args.data, fname)
        img_dst = os.path.join(args.output, fname)
        try:
            init_image = Image.open(img_src).convert("RGB").resize((512, 512))
            canny_image = get_canny(init_image)

            result = pipe(
                prompt=PROMPT,
                negative_prompt=NEGATIVE_PROMPT,
                image=canny_image,
                ip_adapter_image=ref_images,
                num_inference_steps=args.steps,
                guidance_scale=args.guidance,
                controlnet_conditioning_scale=args.cnet_scale,
            ).images[0]

            result = result.resize((640, 480))
            result.save(img_dst)
        except Exception as e:
            print(f"Erreur {fname}: {e}")
            errors += 1
            shutil.copy2(img_src, img_dst)

    print(f"\n✅ TERMINÉ !")
    print(f"  Images traduites : {len(files) - errors}")
    print(f"  Échecs           : {errors}")
    print(f"  Output           : {args.output}")

if __name__ == "__main__":
    main()