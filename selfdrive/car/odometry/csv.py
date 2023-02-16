import pandas as pd
import pymap3d as pm

# read the CSV file into a pandas dataframe
df = pd.read_csv('encoder_odom__.csv')

# use pymap3d to convert the latitude, longitude, and altitude to ENU coordinates
e, n, u = pm.geodetic2enu(df['latitude'], df['longitude'], df['altitude'])

# add the ENU coordinates to the dataframe
df['E'] = e
df['N'] = n
df['U'] = u

# save the updated dataframe to a new CSV file
df.to_csv('output_file.csv', index=False)