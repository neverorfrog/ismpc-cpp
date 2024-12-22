import os
import subprocess
from pathlib import Path
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        super().__init__(name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
    user_options = build_ext.user_options + [
        ('cmake-args=', None, 'Additional arguments for CMake')
    ]

    def initialize_options(self):
        super().initialize_options()
        self.cmake_args = None

    def finalize_options(self):
        super().finalize_options()
        self.cmake_args = self.cmake_args.split() if self.cmake_args else []

    def build_extension(self, ext: CMakeExtension) -> None:
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()

        cfg = "Release" if not self.debug else "Debug"
        build_temp = Path.cwd() / "build"
        build_temp.mkdir(parents=True, exist_ok=True)

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
            f"-DBUILD_PYTHON_BINDINGS=ON",
        ] + self.cmake_args

        build_args = ["--config", cfg]

        if not extdir.exists():
            extdir.mkdir(parents=True)

        subprocess.check_call(["cmake", ext.sourcedir] + cmake_args, cwd=build_temp)
        subprocess.check_call(["cmake", "--build", ".", "--target", ext.name] + build_args, cwd=build_temp)

setup(
    name="ismpc_py",
    version="0.1",
    author="Flavio Maiorana",
    author_email="97flavio.maiorana@gmail.com",
    description="Cose",
    long_description="Altre cose",
    ext_modules=[CMakeExtension("ismpc_py", sourcedir=".")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
    extras_require={"test": ["pytest>=6.0"]},
    python_requires=">=3.8",
    package_data={"bindings": ["ismpc_py.pyi"]},
    package_dir={"": "."},
    packages=["bindings"],
)
