from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize


def compile_cython():
    extensions = cythonize([Extension(
            "Kinematics",
            sources=["src/Kinematics.pyx"],
            extra_compile_args=["-march=native", "-ffast-math", "-O3"])],
            compiler_directives={'language_level':"3"})

    setup(
        ext_modules=extensions,
    )


if __name__ == '__main__':
    compile_cython()