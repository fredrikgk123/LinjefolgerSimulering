from PIL import Image
import numpy as np

img = Image.open('/Users/fredrikkarlsaune/Desktop/Studie/2. Semester/AIS1104AUTO/Fase3/Simulering/assets/suzuka.png').convert('L')
arr = np.array(img)

with open('/Users/fredrikkarlsaune/Desktop/Studie/2. Semester/AIS1104AUTO/Fase3/Simulering/image_info.txt', 'w') as f:
    f.write(f"Shape: {arr.shape}\n")
    f.write(f"Min: {arr.min()}, Max: {arr.max()}, Mean: {round(float(arr.mean()), 1)}\n")
    f.write(f"Pixels == 0: {int((arr == 0).sum())}\n")
    f.write(f"Pixels == 255: {int((arr == 255).sum())}\n")
    f.write(f"Pixels < 50: {int((arr < 50).sum())}\n")
    f.write(f"Pixels < 128: {int((arr < 128).sum())}\n")
    f.write(f"Pixels > 200: {int((arr > 200).sum())}\n")
    f.write(f"Total pixels: {arr.size}\n")
    # Show histogram buckets
    hist, _ = np.histogram(arr, bins=8, range=(0,256))
    for i, h in enumerate(hist):
        f.write(f"  [{i*32}-{(i+1)*32}]: {h}\n")

