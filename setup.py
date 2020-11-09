""" Generic setup.py to compile each .pyx module. Build using
python cython_setup.py build_ext --inplace
"""

from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize


def compile_cython():
    extensions = cythonize([Extension(
            "cythonKinematics",
            sources=["src/Kinematics.pyx"],
            extra_compile_args=["-march=native", "-ffast-math", "-O3"])],
            compiler_directives={'language_level':"3"})

    setup(
        ext_modules=extensions,
    )


if __name__ == '__main__':
    compile_cython()