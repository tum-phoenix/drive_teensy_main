# The build system in PlatformIO is based on SCon
import os
import subprocess
import shutil

# create SCon environment
env = Environment()

# remember current working directory
cwd = os.getcwd()

# dsdl compiler
dsdlc = os.path.join(cwd, 'libuavcan', 'libuavcan', 'dsdl_compiler', 'libuavcan_dsdlc')

# Update submodules
print("Update git submodules...")
subprocess.call(['git', 'submodule', 'update', '--init', '--recursive'])

# Collect all paths that contain .uavcan files
dirs = list()
dirs.append(os.path.join(cwd, 'libuavcan', 'dsdl', 'uavcan'))
dirs.append(os.path.join(cwd, '..', 'phoenix_msgs'))

print("Running DSDL Compiler")
# Run the DSDL compiler on all previously defined directories and put the
# output into the message_header directory
subprocess.call( ['python', dsdlc] + dirs + ['-Omessage_header'])

print("Done setting up build environment")