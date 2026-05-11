import sys


# Block open3d.ml's eager import chain — it pulls sklearn → scipy and trips
# a numpy ABI mismatch in this container. The calibrator only needs
# open3d.{geometry,io,utility,visualization}.
# TODO: remove once docker setup pins compatible numpy/scipy/open3d versions.
class _Open3DMLBlocker:
    def find_module(self, name, path=None):
        if name == "open3d.ml" or name.startswith("open3d._ml3d"):
            return self
        return None

    def load_module(self, name):
        m = sys.modules.get(name) or type(sys)(name)
        m.__path__ = []
        sys.modules[name] = m
        return m


sys.meta_path.insert(0, _Open3DMLBlocker())
