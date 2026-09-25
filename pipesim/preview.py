"""Bounded, server-local preparation for interactive movement.

Cached assemblies are read-only snapshots. Keys include the complete document
and its directory; file stamps invalidate library/mesh edits and replacements.
No preview changes the authored document. Only release materializes an edit.
"""
from collections import OrderedDict
import hashlib
import json
from pathlib import Path
import secrets
from threading import RLock

from .document import DATA, Library


def _key(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _sources(doc, base):
    return tuple(sorted((DATA/'libraries').glob('*.yaml')))+tuple(Path(base)/ref for ref in doc.get('libraries', []))


def _stamp(paths):
    values=[]
    for path in paths:
        resolved=path.resolve(); stat=resolved.stat()
        values.append((str(path), str(resolved), stat.st_mtime_ns, stat.st_ctime_ns, stat.st_size, stat.st_ino))
    return tuple(values)


def _put(cache, key, value, limit):
    cache[key]=value; cache.move_to_end(key)
    while len(cache)>limit: cache.popitem(last=False)
    return value


class PreparedAssembly:
    def __init__(self, assembly):
        self.assembly=assembly
        self.sources=_sources(assembly.doc, assembly.base)
        self.meshes=tuple(sorted({p.base/shape['file'] for p in assembly.parts.values()
                                  for shape in p.shapes if shape['type']=='mesh'}))
        self.stamp=_stamp(self.sources+self.meshes)
        self.mechanisms=OrderedDict(); self.results=OrderedDict(); self.lock=RLock()

    def current(self):
        try:
            return self.sources==_sources(self.assembly.doc, self.assembly.base) and self.stamp==_stamp(self.sources+self.meshes)
        except OSError: return False

    def mechanism(self, selected):
        from .posing import Mechanism
        with self.lock:
            if selected in self.mechanisms:
                self.mechanisms.move_to_end(selected)
                return self.mechanisms[selected]
            return _put(self.mechanisms, selected, Mechanism(self.assembly, selected), 16)

    @staticmethod
    def movement_key(data):
        return _key({k:data.get(k) for k in ('selected', 'object', 'target', 'mode')})

    def remember(self, data, result):
        token=secrets.token_urlsafe(16)
        with self.lock: _put(self.results, token, (self.movement_key(data), result), 8)
        return {**result, 'preview_id':token}

    def recalled(self, data):
        with self.lock:
            item=self.results.get(data.get('preview_id'))
            return item[1] if item and item[0]==self.movement_key(data) else None


class PreviewCache:
    def __init__(self):
        self.libraries=OrderedDict(); self.assemblies=OrderedDict(); self.lock=RLock()

    def library(self, doc, base):
        stamp=_stamp(_sources(doc, base))
        # Assembly.from_doc takes its own copy, so overrides cannot pollute the
        # cached source library or another tab's resolved parts.
        with self.lock:
            if stamp in self.libraries:
                self.libraries.move_to_end(stamp)
                return self.libraries[stamp]
            return _put(self.libraries, stamp, Library.load(doc.get('libraries', []), base), 4)

    def prepared(self, doc, base, build):
        key=(str(Path(base).resolve()), _key(doc))
        with self.lock:
            existing=self.assemblies.get(key)
            if existing and existing.current():
                self.assemblies.move_to_end(key)
                return existing
            self.assemblies.pop(key, None)
            return _put(self.assemblies, key, PreparedAssembly(build()), 8)

    def remember(self, assembly):
        # Resolving/opening a scene already paid for its schema, expansion and
        # asset checks. Reuse that work on the very first drag frame.
        return self.prepared(assembly.doc, assembly.base, lambda:assembly)
