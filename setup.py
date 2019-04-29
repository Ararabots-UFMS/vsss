#!/usr/bin/python
import os
import subprocess as sb
#echo "source ~/ararabots/devel/setup.bash" >> ~/.bashrc

if __name__=="__main__":
    # Gets the path to file
    path_to_file = os.path.dirname(os.path.abspath(__file__))
    # Read from our bash
    with open(os.path.expanduser('~')+'/.bashrc','r+') as file:
        bash_file = file.readlines()

    # This guys are:
    line_number = 0  # the line which the script is located
    found = False  # If he is in fact in our bashrc
    ros_found = False  # There is ROS in this machine?

    while (line_number < len(bash_file)) and not found:
        # Find the line of our header
        if bash_file[line_number] == "#======= ARARABOT ROS BASH SCRIPT =============\n":
            print('Script found at line :'+str(line_number))
            found = True  # Found'ya at line_number
        line_number += 1

    # Find ROS so we can put him on our bash
    if os.path.exists("/opt/ros/"):  # ROS IS INSTALLED
        ros_version = sb.check_output("ls", cwd="/opt/ros/").strip("\n")
        path_to_ros = "/opt/ros/"+ros_version+"/"
        print("ROS Version : " + str.capitalize(ros_version))
        ros_found = True
        installation_bash = "source /opt/ros/"+ros_version+"/setup.bash"
    else:
        print("Ros is NOT installed, but setup will continue")
        installation_bash = ""

    # If Ros was found, lets build our package
    if ros_found:
        print("Building Ros Workspace at: " + path_to_file)
        try:
            sb.check_output("catkin_make")

            print("Making Install...")
            sb.check_output("catkin_make install", shell=True)
            package_bash = "source "+path_to_file + "/devel/setup.bash"

        except OSError:
            print("Something went wrong when building the package, please check.")
            package_bash = ""
    else:
        package_bash = ""

    # The enviroment variable to use in python script files
    ros_arara_root = path_to_file + "/src/verysmall/"

    if found:
        print("Updating script in bash.")
        bash_file[line_number] = "export ROS_ARARA_ROOT="+ros_arara_root + "\n"
        bash_file[line_number+1] = installation_bash + "\n"
        bash_file[line_number+2] = package_bash+ "\n"
        bash_file[line_number+3] = "source "+ros_arara_root+"src/parameters/rosmaster.bash"+ "\n"
        bash_file[line_number+4] = "export ROS_PACKAGE_PATH="+path_to_file + "/src/:"+ path_to_ros+"share/"+"\n"
        bash_file[line_number+5] = "export PYTHONPATH=$PYTHONPATH:"+ros_arara_root+"src/\n"
    else:
        print("Creating sources in bash")
        bash_file.append("\n#======= ARARABOT ROS BASH SCRIPT =============\n" +
                         "export ROS_ARARA_ROOT="+ros_arara_root + "\n" +
                         installation_bash + "\n" +
                         package_bash+ "\n" +
                         "source "+ros_arara_root+"src/parameters/rosmaster.bash"+ "\n" +
                         "export ROS_PACKAGE_PATH="+path_to_file + "/src/:"+ path_to_ros+"share/"+"\n" +
                         "export PYTHONPATH=$PYTHONPATH:"+ros_arara_root+"src/\n"+
                         "#======== END OF ARARABOT SCRIPT ==============\n")

    with open(os.path.expanduser('~')+'/.bashrc','w+') as file:
       file.writelines(bash_file)