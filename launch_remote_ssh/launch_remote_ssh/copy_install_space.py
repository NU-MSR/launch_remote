from ament_index_python.packages import get_package_prefix
import subprocess

def copy_install_space(user, machine, package_name, remote_install_space):
    if user == '' or machine == '':
        raise Exception("'user' and 'machine' arguments must be provided.")
    
    user_machine = user + '@' + machine

    package_install_space = get_package_prefix(package_name) + '/'

    # Create install space on remote machine
    subprocess.run(
        ['ssh ' + user_machine + ' "mkdir -p ' + remote_install_space + '"'],
        shell=True,
    )

    # Copy files to install space
    subprocess.run(
        ['rsync -r ' + package_install_space + ' ' + user_machine + ':' + remote_install_space],
        shell=True
    )
