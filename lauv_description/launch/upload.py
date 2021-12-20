from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration as Lc

# from launch.actions import ExecuteProcess
# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

import os
import pathlib
import xacro


def launch_setup(context, *args, **kwargs): 
    debug = Lc('debug').perform(context)
    x = Lc('x').perform(context)
    y = Lc('y').perform(context)
    z = Lc('z').perform(context)
    roll = Lc('roll').perform(context)
    pitch = Lc('pitch').perform(context)
    yaw = Lc('yaw').perform(context)
    use_geodetic = Lc('use_geodetic').perform(context)
    latitude = Lc('latitude').perform(context)
    longitude = Lc('longitude').perform(context)
    depth = Lc('depth').perform(context)
    latitude_ref = Lc('latitude_ref').perform(context)
    longitude_ref = Lc('longitude_ref').perform(context)
    altitude_ref = Lc('altitude_ref').perform(context)
    mode = Lc('mode').perform(context)
    namespace = Lc('namespace').perform(context)
    gazebo_namespace = Lc('gazebo_namespace').perform(context)
    reference_frame = Lc('reference_frame').perform(context)
    
    # Generate the urdf file
    xacro_file = os.path.join(
        get_package_share_directory('lauv_description'),
        'robots',
        mode + '.xacro'
    )

    if not pathlib.Path(xacro_file).exists():
        exc = 'xacro file ' + xacro_file + ' does not exist'
        raise Exception(exc)

    mappings = {'debug': debug, 
                'namespace': namespace, 
                'inertial_reference_frame': reference_frame,
                }    

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings=mappings)

    # Generate the urdf file
    urdf_file = os.path.join(
        get_package_share_directory('lauv_description'),
        'robots/generated',
        'robot_description.urdf'
        )
    with open(urdf_file, 'w') as file_out:
        file_out.write(doc.toxml())

    # Run a python script to the send a service call to gazebo_ros to spawn a URDF robot 
    args = []

    if use_geodetic == True:
        if gazebo_namespace == "":
            args = ('-urdf -latitude %s -longitude %s -depth %s -latitude_ref %s -longitude_ref %s -altitude_ref %s -R %s -P %s -Y %s -entity %s -param %s'
                %(latitude, longitude, depth, latitude_ref, longitude_ref, altitude_ref, roll, pitch, yaw, namespace, '/' + namespace + '/robot_description')).split()
        else:
            args = ('-gazebo_namespace %s -urdf -latitude %s -longitude %s -depth %s -latitude_ref %s -longitude_ref %s -altitude_ref %s -R %s -P %s -Y %s -entity %s -param %s'
                %(gazebo_namespace, latitude, longitude, depth, latitude_ref, longitude_ref, altitude_ref, roll, pitch, yaw, namespace, '/' + namespace + '/robot_description')).split()
    else:
        if gazebo_namespace == "":
            args = ('-x %s -y %s -z %s -R %s -P %s -Y %s -entity %s -topic robot_description' 
                %(x, y, z, roll, pitch, yaw, namespace)).split()
        else:
            args = ('-gazebo_namespace %s -x %s -y %s -z %s -R %s -P %s -Y %s -entity %s -topic robot_description' 
                %(gazebo_namespace, x, y, z, roll, pitch, yaw, namespace)).split()

    spawn_model = Node(
        name = 'urdf_spawner',
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=args
    )

    # A joint state publisher plugin already is started with the model, no need to use the default joint state publisher

    # Publish robot model for ROS
    args = {'robot_description': doc.toxml(), 'namespace': namespace}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        respawn='true',
        parameters=[args]
    )

    # Publish state and tf for in relation to the world frame
    message_to_tf_launch = os.path.join(
        get_package_share_directory('uuv_assistants'),
        'launch',
        'message_to_tf.launch'
    )

    if not pathlib.Path(message_to_tf_launch).exists():
        exc = 'Launch file ' + message_to_tf_launch + ' does not exist'
        raise Exception(exc)

    args = [('namespace', namespace), 
            ('world_frame', reference_frame), 
            ('child_frame_id', '/' + namespace + '/base_link'), 
           ]

    message_to_tf_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(message_to_tf_launch), launch_arguments=args)

    group = GroupAction([
        spawn_model,
        robot_state_publisher,
    ])
    
    return [group, message_to_tf_launch]



def generate_launch_description():
    return LaunchDescription([
        # Debug flag
        DeclareLaunchArgument('debug', default_value='0'),

        # Vehicle's initial pose
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_geodetic', default_value='false'),
        DeclareLaunchArgument('latitude', default_value='0'),
        DeclareLaunchArgument('longitude', default_value='0'),
        DeclareLaunchArgument('depth', default_value='0'),
        DeclareLaunchArgument('latitude_ref', default_value='0'),
        DeclareLaunchArgument('longitude_ref', default_value='0'),
        DeclareLaunchArgument('altitude_ref', default_value='0'),
        DeclareLaunchArgument('mode', default_value='default'),
        DeclareLaunchArgument('namespace', default_value='lauv'),
        DeclareLaunchArgument('gazebo_namespace', default_value='gazebo'),
        DeclareLaunchArgument('reference_frame', default_value='world'),

        OpaqueFunction(function = launch_setup)
    ])
