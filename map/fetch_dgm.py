"""Read selected members of an official DGM ZIP using HTTP byte ranges."""
import argparse
import fnmatch
import io
from pathlib import Path
from urllib.request import Request, urlopen
import zipfile

DEFAULT_URL = "https://www.daten-hamburg.de/opendata/fernerkundung_hoehenmodelle/dgm/dgm1_hh_2022-04-30.zip"


class RemoteZipFile(io.RawIOBase):
    def __init__(self, url):
        with urlopen(Request(url, method="HEAD"), timeout=60) as response:
            self.url = response.url
            self.size = int(response.headers["Content-Length"])
            self.etag = response.headers.get("ETag")
        self.position = 0

    def seekable(self):
        return True

    def tell(self):
        return self.position

    def seek(self, offset, whence=0):
        self.position = (0 if whence == 0 else self.position if whence == 1 else self.size) + offset
        if self.position < 0:
            raise ValueError("Negative seek")
        return self.position

    def read(self, size=-1):
        size = min(self.size - self.position, size if size >= 0 else self.size)
        if size <= 0:
            return b""
        if size > 256 * 1024 * 1024:
            raise ValueError("Member exceeds 256 MB request limit; download the archive explicitly instead")
        start, end = self.position, self.position + size - 1
        headers = {"Range": "bytes={}-{}".format(start, end), "Accept-Encoding": "identity"}
        if self.etag:
            headers["If-Match"] = self.etag
        with urlopen(Request(self.url, headers=headers), timeout=120) as response:
            expected = "bytes {}-{}/{}".format(start, end, self.size)
            if response.status != 206 or response.headers.get("Content-Range") != expected:
                raise IOError("Server did not honor byte range; refusing whole-archive download")
            data = response.read(size)
        if len(data) != size:
            raise IOError("Incomplete ZIP range")
        self.position += size
        return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--pattern", action="append", default=[])
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "dgm")
    args = parser.parse_args()
    with RemoteZipFile(args.url) as remote, zipfile.ZipFile(remote) as archive:
        members = [m for m in archive.infolist() if not m.is_dir()]
        if not args.pattern:
            for member in members:
                print(member.filename, member.file_size)
            return
        matches = [m for m in members if any(fnmatch.fnmatch(m.filename, p) for p in args.pattern)]
        if not matches:
            raise SystemExit("No matching ZIP members")
        args.output.mkdir(parents=True, exist_ok=True)
        for member in matches:
            target = args.output / Path(member.filename).name
            if target.exists():
                raise SystemExit("Already exists: {}".format(target))
            data = archive.read(member)  # zipfile validates decompression and CRC.
            target.write_bytes(data)
            print("Downloaded", target, len(data), "bytes")


if __name__ == "__main__":
    main()
