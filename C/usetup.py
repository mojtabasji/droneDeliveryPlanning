from distutils.core import setup, Extension
uav_tjp_module = Extension('_uav_tjp',
   sources=['uav_tjp_wrap.cxx','uav_tjp.cpp','point.cpp'],
)
setup (name = 'uav_tjp',
   version = '0.1',
   author = "SWIG Docs",
   description = """Simple swig uav_tjp from docs""",
   ext_modules = [uav_tjp_module],
   py_modules = ["uav_tjp"],
)