import hashlib
from pathlib import Path


def source_fingerprints(*filenames):
    root = Path(__file__).resolve().parent.parent
    paths = [Path(__file__), *(Path(filename) for filename in filenames)]
    return {path.resolve().relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths}
