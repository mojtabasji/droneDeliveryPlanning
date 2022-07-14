from PIL import Image , ImageDraw
import json

images =[]

f = open('picpath.json', 'r')
path = json.load(f)

colors = [(204, 0, 0), (204, 102, 0), (204, 204, 0), (102, 204, 0),
          (0, 204, 0), (0, 204, 102), (0, 204, 204), (0, 102, 204),
          (0, 0, 204), (102, 0, 204), (204, 0, 204), (204, 0, 102),
          (96, 96, 96)]

for i in range(0,len(path[0]), int(len(path[0]) / 100) ):
    print(i)
    im = Image.open('./images/tehran.png')
    draw = ImageDraw.Draw(im)
    for j in range(len(path)):
        point2pixcel_x = path[j][i][0]
        point2pixcel_y = path[j][i][1]
        clr = (0, 0, 0)
        if j != 0:
            clr = colors[int(j % 13)]
        draw.rounded_rectangle([point2pixcel_x-5, point2pixcel_y-5, point2pixcel_x+5, point2pixcel_y+5],
                                radius=1, fill=clr)
    images.append(im)



images[0].save('./images/tehran_imagedraw.gif',
               save_all=True, append_images=images[1:], optimize=True, duration=1000, loop=20)