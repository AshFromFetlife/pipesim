"""Refresh public product metadata and dimension drawings, without deriving dimensions by guessing.

Downloads are intentionally separate from the offline runtime. The checked-in library
contains manually reviewed dimension transcriptions, never a product-title parser.
"""
from __future__ import annotations
import argparse
import concurrent.futures
import hashlib
import json
import re
import urllib.request
from pathlib import Path
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]

def fetch(url):
    req = urllib.request.Request(url, headers={"User-Agent": "PipeSim/0.1 (public CAD metadata research)"})
    with urllib.request.urlopen(req, timeout=40) as response:
        return response.read()

def product(handle):
    directory = ROOT / "sources" / "tubeclamp" / handle
    directory.mkdir(parents=True, exist_ok=True)
    data = json.loads(fetch(f"https://www.tubeclamp.com.au/products/{handle}.js"))
    images = ["https:" + u if u.startswith("//") else u for u in data["images"]]
    record = {"title": data["title"], "url": f"https://www.tubeclamp.com.au/products/{handle}",
              "retrieved": datetime.now(timezone.utc).isoformat(), "images": images,
              "variants": [{k: v.get(k) for k in ("title", "sku", "weight", "price")} for v in data["variants"]]}
    (directory / "source.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    drawings = [u for u in images if re.search(r"technical|diagram|dimension|spec|drawing", u, re.I)]
    if not drawings:
        drawings = images[1:3]
    for i, url in enumerate(drawings):
        raw = fetch(url)
        path = directory / f"drawing-{i}{Path(url.split('?')[0]).suffix}"
        path.write_bytes(raw)
        record.setdefault("downloads", []).append({"path": path.name, "url": url, "sha256": hashlib.sha256(raw).hexdigest()})
    (directory / "source.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    return handle, data["title"], len(drawings)

def references():
    from PIL import Image, ImageDraw
    directory = ROOT / "sources" / "inspiration"
    directory.mkdir(parents=True, exist_ok=True)
    raw = fetch("https://www.tubeclamp.com.au/pages/premade-kits").decode()
    urls = list(dict.fromkeys(re.findall(r'(?:https:)?//cdn\.shopify\.com/[^"<>\s]+', raw)))
    urls = ["https:" + u if u.startswith("//") else u for u in urls]
    urls = [u.replace("&amp;", "&") for u in urls if re.search(r"\.(jpg|png|webp)", u, re.I)]
    records = []
    tiles = []
    for i, url in enumerate(urls):
        try:
            path = directory / f"kit-{i}.jpg"
            from io import BytesIO
            pic = Image.open(BytesIO(fetch(url))).convert("RGB")
            if pic.width < 250 or pic.height < 200:
                continue
            pic.save(path)
            pic.thumbnail((290, 240))
            tile = Image.new("RGB", (310, 270), "white")
            tile.paste(pic, ((310-pic.width)//2, 5))
            ImageDraw.Draw(tile).text((10, 248), str(i), fill="black")
            tiles.append(tile)
            records.append({"path": path.name, "url": url})
        except Exception as exc:
            print(f"Reference {i}: {exc}")
    for page in range((len(tiles)+19)//20):
        batch = tiles[page*20:(page+1)*20]
        sheet = Image.new("RGB", (1550, 270*((len(batch)+4)//5)), "#dddddd")
        for j,tile in enumerate(batch):
            sheet.paste(tile, ((j%5)*310, (j//5)*270))
        sheet.save(directory / f"contact-{page}.jpg")
    (directory / "sources.json").write_text(json.dumps(records, indent=2), encoding="utf-8")
    return len(records)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("handles", nargs="*", default=["tc101", "tc104", "tc116", "tc125", "tc128", "tc131", "tc132", "tc134", "tc136", "tc138", "tc140", "tc148", "tc161", "tc173", "tc173m", "tc173f"])
    parser.add_argument("--references", action="store_true")
    args = parser.parse_args()
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as pool:
        tasks = {pool.submit(product, h): h for h in args.handles}
        for task in concurrent.futures.as_completed(tasks):
            try:
                print(task.result())
            except Exception as exc:
                print(tasks[task], type(exc).__name__, str(exc))
    if args.references:
        print("Reference images:", references())
