import dynamic_reconfigure.client

rospy.init_node('myconfig_py', anonymous=True)

client = dynamic_reconfigure.client.Client(takeoff_node)

params = { 'my_string_parameter' : 'value', 'my_int_parameter' : 5 }
config = client.update_configuration(params)