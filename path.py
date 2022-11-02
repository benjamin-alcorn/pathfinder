###
# This file was created by Ryan Middleton for the Aggie Map Navigation Helper team for Senior Capstone Project.
# The Aggie Map Navigation Helper guides users through the Texas A&M University campus via an Android App.
# The user inputs their destination and a picture of their surroundings(with the nearest building)and the phone app will
# determine their location. This code guides the user through campus from their current location to their destination.
# The code takes in a set of two gps coordinates, one for a user's current location, and another for destination.
# This code contains functions and code executions for the pathfinder.
###

def pathfinder(event, context):
  # This function finds the shortest distance path between two points(current and destination)
  # on the network (graph)

  import pandas as pd
  import geopandas as gpd
  from shapely.geometry import LineString, Point
  import osmnx as ox
  import networkx as nx
  from math import sin, cos, asin, sqrt, pi
  import numpy as np

  # This function performs the following actions in order:
  # 1. Convert input coordinates
  # 2. Find the shortest path
  # 3. Format results for output as a list
  # 4. Calculate shortest path length

  # Collect user current location and destination from S3
  import csv
  import boto3
  s3 = boto3.resource('s3')
  s3.Bucket('user-input-image').download_file('start_end_coordinates.csv', '/tmp/start_end_coordinates.csv') 
  CSVData = open('/tmp/start_end_coordinates.csv')
  coord_arr = np.genfromtxt(CSVData, delimiter=",")
  coord1 = coord_arr[0,0]
  coord2 = coord_arr[0,1]
  coord3 = coord_arr[1,0]
  coord4 = coord_arr[1,1]

  # Pull in graph from .graphml file
  graph_paths = ox.load_graphml(filepath='graph.graphml')

  # 1. Convert input coordinates
  # This first code creates separate GeoDataFrames to store both locations
  # It takes in a numpy array and returns the two location GeoDataFrames
  # Include CRS 4326
  current_coord = gpd.GeoDataFrame(columns=['geometry'], crs=4326, geometry='geometry')
  current_coord.at[0, 'geometry'] = Point(coord2, coord1)
  destination_coord = gpd.GeoDataFrame(columns=['geometry'], crs=4326, geometry='geometry')
  destination_coord.at[0, 'geometry'] = Point(coord4, coord3)

  # 2. Find the shortest path
  # Take array of dots and project them
  graph_proj = ox.project_graph(graph_paths)
  # Convert projection into a geo-dataframe
  map_edges = ox.graph_to_gdfs(graph_proj, nodes=False)
  # Returns the Coordinate Reference system of the GeoDataFrame area
  crs_map = map_edges.crs
  # Given the pulled CRS type, re-project the coordinates onto the same coordinate system of the map
  origin_projection = current_coord.to_crs(crs=crs_map)
  destination_projection = destination_coord.to_crs(crs=crs_map)
  # Create a GeoDataFrame to store the route of the shortest path
  output_path = gpd.GeoDataFrame()
  # Store all nodes from the graph area without the edge nodes
  map_nodes = ox.graph_to_gdfs(graph_proj, edges=False)
  # Extract coordinate info from origin node
  for oidx, orig in origin_projection.iterrows():
      # Locate the graph node closest to the user's location coordinates
      # nearest_nodes uses k-d trees to find nearest neighbor
      nearest_origin_node = ox.nearest_nodes(G=graph_proj, X=orig.geometry.x, Y=orig.geometry.y)
      # Extract coordinate info from destination node
      for tidx, target in destination_projection.iterrows():
          # Locate the graph node closest to the user's destination coordinates
          # nearest_nodes uses k-d trees to find nearest neighbor
          nearest_target_node = ox.nearest_nodes(graph_proj, X=target.geometry.x, Y=target.geometry.y)
          # Use dijkstra's algorithm to find the shortest path
          shortest_path = nx.dijkstra_path(graph_proj,
                                            source=nearest_origin_node,
                                            target=nearest_target_node, weight='length')
          # Isolate only the nodes of this shortest path
          shortest_path_nodes = map_nodes.loc[shortest_path]
          # Format results into a linestring to be used for plotting the shortest path for demo purposes
          # List the nodes into a linestring so that they can be written into a new GeoDataFrame
          #return len(list(shortest_path_nodes))
          nodes_list = LineString(list(shortest_path_nodes.geometry.values))
          # Place results into a GeoDataFrame
          output_path = output_path.append([[nodes_list]], ignore_index=True)

  # This next code takes the GeoDataFrame of output nodes and converts them into
  # a numpy array with proper labeling for output to the phone app. The array is
  # a versatile choice and it is easy to load into new python code

  # Now the output nodes need to be formatted so that they can used as an output to the navigation system
  new_coord = pd.DataFrame(shortest_path_nodes[['lon', 'lat']])
  new_coord = new_coord[['lat', 'lon']]
  new_index = list(range(0, len(new_coord)))
  # rename the node number index
  new_coord['num'] = new_index
  new_coord = new_coord.set_index('num')
  path_arr = new_coord.to_numpy()
  #print(" The array of path coordinates:")
  #print(path_arr)

  # This next code uses haversine formula to find the distance between two coordinates
  # First converts from degrees to radians
  # Iterates over the path array to add up all path lengths
  # Outputs path length in miles

  # Convert locations to radians
  # starting and ending locations
  current_lat = coord1 * (pi / 180)
  current_lon = coord2 * (pi / 180)
  destination_lat = coord3 * (pi / 180)
  destination_lon = coord4 * (pi / 180)
  # first node on the graph path
  lat1 = path_arr[0, 0] * (pi / 180)
  lon1 = path_arr[0, 1] * (pi / 180)

  # Find the length of the total path
  # Add together the distance from starting location to the first path node
  total_dist_mi = 3958.8 * 2 * asin(
    sqrt(sin((lat1 - current_lat) / 2) ** 2 + cos(current_lat) * cos(lat1) * sin((lon1 - current_lon) / 2) ** 2))

  # Compute distance between each node, then sum up the distances
  for i in range(1, (int(path_arr.size / 2))):
      # Update next destination lat and long points
      lat2 = path_arr[i, 0] * (pi / 180)
      lon2 = path_arr[i, 1] * (pi / 180)
      # Compute individual node distance
      dist = 3958.8 * 2 * asin(sqrt(sin((lat2 - lat1) / 2)
                                    ** 2 + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2) ** 2))
      total_dist_mi = total_dist_mi + dist
      # Update next location lat and long points
      lat1 = path_arr[i, 0] * (pi / 180)
      lon1 = path_arr[i, 1] * (pi / 180)
  # Add together the distance from destination location to the last path node
  total_dist_mi = total_dist_mi + (3958.8 * 2 * asin(sqrt(
    sin((destination_lat - lat1) / 2) ** 2 + cos(lat1) * cos(destination_lat) * sin(
      (destination_lon - lon1) / 2) ** 2)))
  #print("The shortest path has length: ", total_dist_mi, "miles")

  import boto3
  temp_csv_file = csv.writer(open("/tmp/path_coordinates.csv", "w+"))

  len_arr = np.array([[round(total_dist_mi,2),0]])
  path_arr = np.concatenate([len_arr, path_arr])
  np.savetxt('/tmp/path_coordinates.csv', path_arr, delimiter=",")
  np.savetxt('/tmp/path_coordinates.csv', path_arr, fmt="%f", delimiter=",")

  BUCKET_NAME = 'user-input-image'
  client = boto3.client('s3')
  client.upload_file('/tmp/path_coordinates.csv', BUCKET_NAME,'path_coordinates.csv')

  return path_arr.tolist(), total_dist_mi
