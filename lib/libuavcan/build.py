# The build system in PlatformIO is based on SCon
import os
import subprocess

# create SCon environment
env = Environment()

# message header directory
headerdir = "./message_header"

# remember current working directory
cwd = os.getcwd()

# dsdl compiler
dsdlc = os.path.join(cwd, 'libuavcan', 'libuavcan', 'dsdl_compiler', 'libuavcan_dsdlc')

# Update submodules
subprocess.call(['git', 'submodule', 'update', '--init', '--recursive'])

# delete old message headers and folders
print("Delete old message header *.hpp files.")
for root, dirs, files in os.walk(headerdir, topdown=True):
    for name in files:
        if name.endswith(".hpp"):
            os.remove(os.path.join(root, name))


# Collect all paths that contain .uavcan files
dirs = list()
dirs.append(os.path.join(cwd, 'libuavcan', 'dsdl', 'uavcan'))
dirs.append(os.path.join(cwd, '..', 'phoenix_msgs'))
print("Collect *.uavcan files in: " + str(dirs))

# Run the DSDL compiler on all previously defined directories and put the
# output into the headerdir directory
print("Compile *.uavcan to message header *.hpp files.")
subprocess.call( ['python', dsdlc] + dirs + ['-O'] + [headerdir])

print("Done setting up build environment")