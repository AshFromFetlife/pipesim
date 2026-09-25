"""Collect the supplied Tubeclamp gallery's creation images and make review sheets."""
from html.parser import HTMLParser
from html import unescape
from urllib.parse import urljoin
from pathlib import Path
from io import BytesIO
import concurrent.futures
import json
from PIL import Image,ImageDraw
from scripts.fetch_sources import fetch

root=Path('sources/inspiration')
raw=(root/'page.html').read_text(encoding='utf-8') if (root/'page.html').exists() else fetch('https://www.tubeclamp.com.au/pages/premade-kits').decode('utf-8')
class Gallery(HTMLParser):
    def __init__(self): super().__init__(); self.records=[]; self.section=False
    def handle_starttag(self,tag,attrs):
        a=dict(attrs)
        if tag=='img':
            url=a.get('data-src') or a.get('src')
            if url and not url.startswith('data:'):
                self.records.append({'url':urljoin('https://www.tubeclamp.com.au',unescape(url)),'alt':a.get('alt','')})
g=Gallery(); g.feed(raw)
seen=set(); records=[]
for rec in g.records:
    if rec['url'] in seen: continue
    seen.add(rec['url']); records.append(rec)
print(json.dumps(records,indent=2))
def get(item):
    i,record=item
    try:
        picture=Image.open(BytesIO(fetch(record['url']))).convert('RGB')
        if min(picture.size)<150: return None
        picture.thumbnail((410,280))
        tile=Image.new('RGB',(430,320),'white'); tile.paste(picture,((430-picture.width)//2,0))
        caption=(record['alt'] or record['url'].split('/')[-1])[:58]
        ImageDraw.Draw(tile).text((8,290),str(i)+' '+caption,fill='black')
        record['path']=f'creation-{i}.jpg'; picture.save(root/record['path'])
        return record,tile
    except Exception as exc: print(i,str(exc)); return None
with concurrent.futures.ThreadPoolExecutor(max_workers=5) as pool:
    fetched=[v for v in pool.map(get,enumerate(records)) if v]
for i in range(0,len(fetched),16):
    batch=fetched[i:i+16]; sheet=Image.new('RGB',(1720,320*((len(batch)+3)//4)),'#dddddd')
    for j,(_,tile) in enumerate(batch): sheet.paste(tile,((j%4)*430,(j//4)*320))
    sheet.save(root/f'gallery-{i//16}.jpg')
(root/'gallery-sources.json').write_text(json.dumps([r for r,_ in fetched],indent=2),encoding='utf-8')
print('Gallery images:',len(fetched))
