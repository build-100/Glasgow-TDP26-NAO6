#!/usr/bin/env python3
"""Verify and restore the published lossless gzip parts without overwriting a bag."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

ROOT=Path(__file__).resolve().parents[1]
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--manifest',type=Path,default=ROOT/'evidence/20261110_213922/recording.json')
    args=parser.parse_args();manifest=args.manifest.resolve();data=json.loads(manifest.read_text())
    folder=manifest.parent
    names=[data['bag_filename']]+[p['file'] for p in data['compressed_parts']]
    if any(Path(n).name!=n or '/' in n or '\\' in n for n in names):
        raise ValueError('Only local filenames are permitted in a recording manifest.')
    target=folder/data['bag_filename']
    if target.exists():
        if target.stat().st_size==data['bag_bytes'] and sha(target)==data['bag_sha256']:
            print('Already verified: '+str(target));return
        raise RuntimeError('Existing bag differs; it was not overwritten: '+str(target))
    with tempfile.TemporaryFile(dir=folder) as packed:
        for part in data['compressed_parts']:
            p=folder/part['file']
            if not p.is_file() or p.stat().st_size!=part['bytes'] or sha(p)!=part['sha256']:
                raise RuntimeError('Missing or corrupt part: '+str(p))
            with p.open('rb') as f:shutil.copyfileobj(f,packed,1024*1024)
        packed.seek(0)
        with tempfile.NamedTemporaryFile(dir=folder,prefix='.restore-',suffix='.tmp',delete=False) as out:
            temporary=Path(out.name)
            try:
                with gzip.GzipFile(fileobj=packed,mode='rb') as stream:
                    shutil.copyfileobj(stream,out,1024*1024)
            except BaseException:
                out.close();temporary.unlink();raise
    try:
        if temporary.stat().st_size!=data['bag_bytes'] or sha(temporary)!=data['bag_sha256']:
            raise RuntimeError('Decompressed bag checksum mismatch.')
        if target.exists():raise RuntimeError('A bag appeared during restoration; it was not overwritten.')
        temporary.rename(target)
    finally:
        if temporary.exists():temporary.unlink()
    print('Restored and SHA256 verified: '+str(target))
if __name__=='__main__':main()
