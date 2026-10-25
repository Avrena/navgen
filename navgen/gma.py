"""Read-only GMA v1-v3 file tables and payloads (Facepunch/gmad AddonReader.h)."""
from pathlib import Path
import struct
import zlib


def content_path(name):
    name=name.lower().replace('\\','/')
    if not name or name.startswith('/') or ':' in name or '..' in name.split('/'):
        raise ValueError(f'Invalid content path: {name!r}')
    return name


class GMA:
    def __init__(self,path):
        self.path=Path(path);self.entries={}
        size=self.path.stat().st_size
        with self.path.open('rb') as f:
            def read(fmt):
                n=struct.calcsize(fmt);data=f.read(n)
                if len(data)!=n:raise ValueError(f'Truncated GMA table: {self.path}')
                return struct.unpack(fmt,data)
            def cstr():
                value=bytearray()
                while len(value)<1048576:
                    c=f.read(1)
                    if c==b'\0':return bytes(value)
                    if not c:raise ValueError(f'Truncated GMA string: {self.path}')
                    value.extend(c)
                raise ValueError(f'Oversized GMA string: {self.path}')
            if f.read(4)!=b'GMAD':raise ValueError(f'Unsupported GMA signature: {self.path}')
            version,=read('<B')
            if not 1<=version<=3:raise ValueError(f'Unsupported GMA version {version}: {self.path}')
            read('<QQ')
            if version>1:
                while cstr():pass
            cstr();cstr();cstr();read('<i')
            offset=0
            while read('<I')[0]:
                name=content_path(cstr().decode('utf-8'))
                length,crc=read('<qI')
                if length<0 or offset+length>size or name in self.entries:
                    raise ValueError(f'Invalid GMA entry {name}: {self.path}')
                self.entries[name]=(offset,length,crc);offset+=length
            self.data_offset=f.tell()
            if self.data_offset+offset>size:raise ValueError(f'Truncated GMA payload: {self.path}')

    def read(self,name):
        offset,length,crc=self.entries[content_path(name)]
        with self.path.open('rb') as f:
            f.seek(self.data_offset+offset);data=f.read(length)
        if len(data)!=length or zlib.crc32(data)!=crc:
            raise ValueError(f'GMA payload/CRC mismatch for {name}: {self.path}')
        return data
