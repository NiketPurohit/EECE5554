import bagpy
import pandas as pd
from matplotlib import pyplot as plt
import numpy as np

from bagpy import bagreader

# b = bagreader('../Data/closed_stationary.bag')

# velmsgs = b.message_by_topic('/gps')
# veldf = pd.read_csv(velmsgs)

# north = 4689472.08
# east = 328034.74 

# error_east_occluded ,error_north_occluded = np.sqrt(np.mean(((veldf['UTM_easting']) - east) ** 2)), np.sqrt(np.mean(((veldf['UTM_northing']) - north) ** 2))
# print("Occluded")
# print(error_east_occluded, error_north_occluded)
# print()
# print()
# print('---------------------------------------------')
# plt.title("Occluded easting error")
# plt.hist(veldf["UTM_easting"]- east)
# plt.show()
# plt.title("Occluded northing error")
# plt.hist(veldf["UTM_northing"]- north)
# plt.show()

# b = bagreader('../Data/open_stationary.bag')

# velmsgs = b.message_by_topic('/gps')
# veldf = pd.read_csv(velmsgs)

# north = 4689648.23
# east = 327969.49

# error_east_open ,error_north_open = np.sqrt(np.mean(((veldf['UTM_easting']) - east) ** 2)), np.sqrt(np.mean(((veldf['UTM_northing']) - north) ** 2))
# print("Open")
# print(error_east_open, error_north_open)
# print()
# print()
# print('------------------------------------------')


# plt.title("Open easting error")
# plt.hist(veldf["UTM_easting"]- east)
# plt.show()
# plt.title("Open northing error")
# plt.hist(veldf["UTM_northing"]- north)
# plt.show()

  
# # Set title
# ax.set_title("error histogram")
  
# # adding labels
# ax.set_xlabel('error')
  
# # Make some labels.
# rects = ax.patches
# labels = ["error_east_occluded", "error_east_open", "error_north_occluded", "error_north_open"]
  
# for rect, label in zip(rects, labels):
#     height = rect.get_height()
#     ax.text(rect.get_x() + rect.get_width() / 2, height+0.01, label,
#             ha='center', va='bottom')
  
# Show plot
plt.show()




# # scatter plot for open stationary area
# b = bagreader('../Data/open_stationary.bag')

# velmsgs = b.message_by_topic('/gps')
# veldf = pd.read_csv(velmsgs)
# # print(veldf)
# plt.scatter(veldf['UTM_easting'] - veldf['UTM_easting'][0], veldf['UTM_northing'] - veldf['UTM_northing'][0])

# # quickly plot velocities
# b.plot_vel(save_fig=True)


# scatter plot for closed stationary data
# b = bagreader('../Data/closed_stationary.bag')

# velmsgs = b.message_by_topic('/gps')
# veldf = pd.read_csv(velmsgs)
# # print(veldf)
# plt.scatter(veldf['UTM_easting'] - veldf['UTM_easting'][0], veldf['UTM_northing'] - veldf['UTM_northing'][0])

# # quickly plot velocities
# b.plot_vel(save_fig=True)

# scatter plot for moving data
b = bagreader('../Data/moving.bag')

velmsgs = b.message_by_topic('/gps')
veldf = pd.read_csv(velmsgs)
# print(veldf)
x = veldf['UTM_easting'] - veldf['UTM_easting'][0]
y = veldf['UTM_northing'] - veldf['UTM_northing'][0]

a, b1 =np.polyfit(x,y,1)
print(np.polyfit(x,y,1))

print(len(x))
error = y - (a*x + b1)
error = np.sqrt(np.mean(error))
print("gg")
print(error)
print(a,b1)
plt.scatter(x, y)

plt.plot(x, a*x+b1)

# quickly plot velocities
b.plot_vel(save_fig=True)


#altitude scatter plot of closed stationaary data
# b = bagreader('../Data/closed_stationary.bag')

# velmsgs = b.message_by_topic('/gps')
# veldf = pd.read_csv(velmsgs)
# # print(veldf)\
# plt.xlabel('Time')
# plt.ylabel("Altitude")
# plt.scatter(veldf['Time'] - veldf['Time'][0], veldf['Altitude'] - veldf['Altitude'][0])

# # quickly plot velocities

# b.plot_vel(save_fig=True)

#altitude scatter plot of open stationaary data
# b = bagreader('../Data/open_stationary.bag')

# velmsgs = b.message_by_topic('/gps')
# veldf = pd.read_csv(velmsgs)
# # print(veldf)\
# plt.xlabel('Time')
# plt.ylabel("Altitude")
# plt.scatter(veldf['Time'] - veldf['Time'][0], veldf['Altitude'] - veldf['Altitude'][0])

# # quickly plot velocities

# b.plot_vel(save_fig=True)

#altitude scatter plot of moving data
# b = bagreader('../Data/moving.bag')

# velmsgs = b.message_by_topic('/gps')
# veldf = pd.read_csv(velmsgs)
# # print(veldf)\
# plt.xlabel('Time')
# plt.ylabel("Altitude")
# plt.scatter(veldf['Time'] - veldf['Time'][0], veldf['Altitude'] - veldf['Altitude'][0])

# # quickly plot velocities

# b.plot_vel(save_fig=True)



