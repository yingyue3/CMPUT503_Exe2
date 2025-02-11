import rosbag
import os
bag = rosbag.Bag('./packages/my_package/move.bag')
_vehicle_name = os.environ['VEHICLE_NAME']   
for topic, msg, t in bag.read_messages(topics=[f"/{_vehicle_name}/wheels_driver_node/wheels_cmd"]):
    print(msg)
bag.close()

# import bagpy
# from bagpy import bagreader

# b = bagreader('move.bag')

# # get the list of topics
# print(b.topic_table)

# # get all the messages of type velocity
# velmsgs   = b.vel_data()
# veldf = pd.read_csv(velmsgs[0])
# plt.plot(veldf['Time'], veldf['linear.x'])

# # quickly plot velocities
# b.plot_vel(save_fig=True)

# # you can animate a timeseries data
# bagpy.animate_timeseries(veldf['Time'], veldf['linear.x'], title='Velocity Timeseries Plot')
