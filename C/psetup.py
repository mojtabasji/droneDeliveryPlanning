from distutils.core import setup, Extension
point_module = Extension('_point',
   sources=['point_wrap.cxx','point.cpp'],
)
setup (name = 'point',
   version = '0.1',
   author = "SWIG Docs",
   description = """Simple swig point from docs""",
   ext_modules = [point_module],
   py_modules = ["point"],
)