from ament_index_python.packages import get_package_prefix
import subprocess

# Typically use this to copy an entire install space
def copy_install_space(user, machine, local_install_space, remote_install_space):
    _copy_dir(user, machine, local_install_space, remote_install_space)

# Probably don't want to use this unless you intentionally want to only use a single
# package, and that package's rosdeps will be fulfilled on the remote system
def copy_single_package_install(user, machine, package_name, remote_install_space):
    local_package_install = get_package_prefix(package_name) + '/'
    remote_package_install = remote_install_space + '/' + package_name + '/'

    # Copy package to remote install directory
    _copy_dir(user, machine, local_package_install, remote_package_install)

def _copy_dir(user, machine, local_dir, remote_dir):
    if user == '' or machine == '':
        raise Exception("'user' and 'machine' arguments must be provided.")
    
    user_machine = user + '@' + machine

    # Create remote directory on machine
    subprocess.run(
        ['ssh ' + user_machine + ' "mkdir -p ' + remote_dir + '"'],
        shell=True,
    )

    # Copy files to remote directory
    subprocess.run(
        ['rsync -r ' + local_dir + ' ' + user_machine + ':' + remote_dir],
        shell=True
    )