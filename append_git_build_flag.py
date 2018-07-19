import subprocess
Import("env")

git_commit = subprocess.check_output(['git', 'rev-parse', 'HEAD']).strip() 
git_commit = git_commit[0:8]
git_flag = str("-D__GIT_HASH__=0x") + str(git_commit)

print("Uploading with following git commit ID: ", git_commit)

env.Append(
  BUILD_FLAGS=[
      git_flag
  ]
)