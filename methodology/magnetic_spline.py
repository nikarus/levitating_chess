import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager, ExitStack
import ctypes
import hashlib
import json
from pathlib import Path
import platform
import subprocess

import numpy as np
import threadpoolctl

from model import Fixed
from methodology import source_fingerprints


@contextmanager
def spline_workers(evaluator, workers):
    if not isinstance(workers, int) or workers < 1:
        raise ValueError('Native spline worker count must be a positive integer')
    if workers == 1:
        yield evaluator
    else:
        with ThreadPoolExecutor(max_workers=workers) as executor:
            yield lambda points: np.concatenate(list(executor.map(evaluator, np.array_split(points, workers))))


@contextmanager
def native_evaluation(atlas, library_path):
    if atlas.evaluator_identity['method'] != 'scipy':
        raise ValueError('Native evaluation requires an unmodified SciPy atlas evaluator')
    if not isinstance(Fixed.native_spline_blas_threads, int) or Fixed.native_spline_blas_threads < 1:
        raise ValueError('Native BLAS thread count must be a positive integer')
    native = TensorSplineEvaluator(atlas.interpolator, library_path)
    with ExitStack() as resources:
        resources.enter_context(threadpoolctl.threadpool_limits(limits=Fixed.native_spline_blas_threads, user_api='blas'))
        evaluator = resources.enter_context(spline_workers(native, Fixed.native_spline_workers))
        libraries = [{key: info[key] for key in ('internal_api', 'prefix', 'version', 'num_threads', 'threading_layer', 'architecture')}
            for info in threadpoolctl.threadpool_info() if info['user_api'] == 'blas']
        libraries.sort(key=lambda entry: json.dumps(entry, sort_keys=True))
        identity = dict(method='native', build=native.build, workers=Fixed.native_spline_workers,
            blas_threads=Fixed.native_spline_blas_threads, blas_libraries=libraries, threadpoolctl=threadpoolctl.__version__)
        for name, value in dict(interpolator=evaluator, evaluator_identity=identity,
                evaluator_sources=source_fingerprints(__file__, Path(__file__).with_suffix('.c'))).items():
            resources.callback(setattr, atlas, name, getattr(atlas, name))
            setattr(atlas, name, value)
        yield identity


def build(output):
    if output.exists():
        raise ValueError('Preserve existing native spline build evidence')
    if platform.system() != 'Linux':
        raise ValueError('The native spline build is currently verified only on Linux')
    output.mkdir(parents=True)
    source = Path(__file__).with_suffix('.c')
    library = output / 'magnetic_spline.so'
    command = ['cc', '-std=c99', '-O3', '-fPIC', '-shared', '-ffp-contract=off', str(source), '-o', str(library)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    metadata = dict(source_sha256=source_fingerprints(__file__, source), command=command,
        compiler=subprocess.check_output(['cc', '--version'], text=True), platform=platform.platform(),
        machine=platform.machine(), library_sha256=hashlib.sha256(library.read_bytes()).hexdigest())
    library.with_suffix('.json').write_text(json.dumps(metadata, indent=Fixed.report_precision) + '\n')
    return library


class TensorSplineEvaluator:
    def __init__(self, spline, library_path):
        self.build = json.loads(library_path.with_suffix('.json').read_text())
        if (self.build['source_sha256'] != source_fingerprints(__file__, Path(__file__).with_suffix('.c'))
                or self.build['library_sha256'] != hashlib.sha256(library_path.read_bytes()).hexdigest()
                or self.build['machine'] != platform.machine() or platform.system() != 'Linux'):
            raise ValueError('Native spline build source, binary or architecture differs')
        self.c = np.asarray(spline.c)
        self.k = np.ascontiguousarray(spline.k, dtype=np.int64)
        self.t = tuple(np.ascontiguousarray(t, dtype=float) for t in spline.t)
        if (self.c.ndim != 4 or self.c.shape[-1] != 3 or self.c.dtype != np.float64
                or not np.isfinite(self.c).all() or self.k.shape != (3,) or len(self.t) != 3
                or any(k not in Fixed.atlas_spline_orders for k in self.k)
                or any(s <= 0 or s % self.c.itemsize for s in self.c.strides)):
            raise ValueError('Native spline requires real three-axis vector coefficients and supported orders')
        for axis, (knots, degree) in enumerate(zip(self.t, self.k)):
            if (knots.ndim != 1 or not np.isfinite(knots).all() or np.any(np.diff(knots) < 0)
                    or len(knots) != self.c.shape[axis] + degree + 1 or len(knots) < 2 * degree + 2
                    or knots[degree] >= knots[-degree - 1]):
                raise ValueError('Native spline knots and coefficients disagree')
        self.strides = np.asarray(self.c.strides, dtype=np.int64) // self.c.itemsize
        self.lengths = np.asarray([len(t) for t in self.t], dtype=np.int64)
        self.lower = np.array([t[k] for t, k in zip(self.t, self.k)])
        self.upper = np.array([t[-k - 1] for t, k in zip(self.t, self.k)])
        self.library = ctypes.CDLL(str(library_path.resolve()))
        self.evaluate = self.library.evaluate_spline
        real = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        integer = np.ctypeslib.ndpointer(dtype=np.int64, flags='C_CONTIGUOUS')
        self.evaluate.argtypes = [ctypes.c_int64, real, ctypes.c_void_p, integer, integer, integer, real, real, real, real]
        self.evaluate.restype = None

    def __call__(self, points):
        points = np.ascontiguousarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('Native spline requires finite three-dimensional points')
        if np.any(points < self.lower) or np.any(points > self.upper):
            raise ValueError('Native spline query is outside its knot domain')
        result = np.empty((len(points), 3))
        self.evaluate(len(points), points, self.c.ctypes.data, self.strides, self.k, self.lengths, *self.t, result)
        if not np.isfinite(result).all():
            raise ValueError('Native spline returned nonfinite values')
        return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Build the explicit native spline candidate; no production backend changes.')
    parser.add_argument('--output', type=Path, required=True)
    print(build(parser.parse_args().output))
