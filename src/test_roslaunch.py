import roslaunch
from rosparam import upload_params
from yaml import load

def main():
    file1_package = 'find_frontier'
    file1_executable = 'objmap_to_image_converter'
    node1 = roslaunch.core.Node(file1_package, file1_executable, name = 'objmap_to_image_converter0', args=0, output='screen', required = False)
    f = open('/home/morin/catkin_ws/src/find_frontier/param/global_costmap.yaml', 'r')
    yaml_file = load(f)
    f.close()
    upload_params('/', yaml_file)
    launch1 = roslaunch.scriptapi.ROSLaunch()
    launch1.start()
    process = launch1.launch(node1)
    launch1.spin()
    print("bye")

if __name__ == '__main__':
    main()
