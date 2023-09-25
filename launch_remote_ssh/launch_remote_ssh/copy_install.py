from ament_index_python.packages import get_package_prefix
import subprocess

def copy_single_package_install(user, machine, package_name, remote_install_space):
    if user == '' or machine == '':
        raise Exception("'user' and 'machine' arguments must be provided.")
    
    user_machine = user + '@' + machine

    local_package_install = get_package_prefix(package_name) + '/'
    remote_package_install = remote_install_space + '/' + package_name + '/'

    # Create install space on remote machine
    subprocess.run(
        ['ssh ' + user_machine + ' "mkdir -p ' + remote_package_install + '"'],
        shell=True,
    )

    # Copy files to install space
    subprocess.run(
        ['rsync -r ' + local_package_install + ' ' + user_machine + ':' + remote_package_install],
        shell=True
    )
