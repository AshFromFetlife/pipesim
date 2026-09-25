from pathlib import Path
from PIL import Image, ImageDraw

files = list(Path('sources/tubeclamp').glob('*/drawing-0*'))
sheet = Image.new('RGB', (2400, 650*((len(files)+3)//4)), 'white')
for i, path in enumerate(files):
    pic = Image.open(path).convert('RGB')
    print(path, pic.size)
    pic.thumbnail((590,610))
    sheet.paste(pic, ((i%4)*600, (i//4)*650))
    ImageDraw.Draw(sheet).text(((i%4)*600+10, (i//4)*650+620), path.parent.name, fill='black')
sheet.save('sources/tubeclamp/contact.jpg')
