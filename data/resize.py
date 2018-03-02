import PIL
from PIL import Image

img = Image.open('data/images/00000000.png')
img = img.resize((320,240))
img.save('resized_img.png')