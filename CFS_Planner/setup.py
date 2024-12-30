from setuptools import setup
from setuptools.command.install import install
import os
import shutil

class InstallCommand(install):
    def run(self):
        # Run the standard install command
        install.run(self)
        
        # Now copy the already-built .so file to the installation directory
        build_dir = os.path.join(os.path.dirname(__file__), "build")
        so_file = [f for f in os.listdir(build_dir) if f.endswith(".so")][0]
        target_dir = os.path.join(self.install_lib, so_file)
        shutil.copy(os.path.join(build_dir, so_file), target_dir)

setup(
    name="cfspy",
    version="0.1",
    cmdclass={'install': InstallCommand},
    py_modules=["cfspy"],  # This tells setuptools that there is a module named cfspy
)
