import rosbag2_py
import folium
import rclpy.serialization
from gps_msgs.msg import GPSmsg


def read_gps_data_from_bag(bag_path, topic_name):
    # Initialize bag reader
    bag_reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    bag_reader.open(storage_options, converter_options)

    # Retrieve topic and types
    topics_and_types = bag_reader.get_all_topics_and_types()
    selected_topic = None
    for topic in topics_and_types:
        if topic.name == topic_name:
            selected_topic = topic.name
            break

    if selected_topic is None:
        print(f"Topic {topic_name} not found in the bag.")
        return []

    # Reading messages from the topic
    gps_data = []
    while bag_reader.has_next():
        (topic, data, t) = bag_reader.read_next()
        if topic == selected_topic:
            msg = rclpy.serialization.deserialize_message(data, GPSmsg)
            gps_data.append({
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude,
                'utm_easting': msg.utm_easting,
                'utm_northing': msg.utm_northing,
                'zone': msg.zone,
                'letter': msg.letter
            })
    return gps_data


def calculate_average_position(gps_data):
    # Calculate the mean latitude and longitude
    if len(gps_data) == 0:
        return None
    
    total_latitude = sum(point['latitude'] for point in gps_data)
    total_longitude = sum(point['longitude'] for point in gps_data)
    
    avg_latitude = total_latitude / len(gps_data)
    avg_longitude = total_longitude / len(gps_data)
    
    return avg_latitude, avg_longitude


def plot_gps_data_on_map(gps_data, map_output_path='gps_map.html'):
    # Create a Folium map centered on the first GPS point
    if len(gps_data) == 0:
        print("No GPS data to plot.")
        return

    first_point = gps_data[0]
    gps_map = folium.Map(location=[first_point['latitude'], first_point['longitude']], zoom_start=15)

    # Add markers for each GPS data point
    for point in gps_data:
        folium.Marker(
            location=[point['latitude'], point['longitude']],
            popup=f"Altitude: {point['altitude']} m, UTM Easting: {point['utm_easting']}, UTM Northing: {point['utm_northing']}",
        ).add_to(gps_map)

    # Save the map as an HTML file
    gps_map.save(map_output_path)
    print(f"Map saved to {map_output_path}")


if __name__ == "__main__":
    bag_path = '/home/tapan/ROS2/lab4_ws/src/nav_driver/data_going_in_circles/data_going_in_circles_0.db3'
    topic_name = '/gps'
    gps_data = read_gps_data_from_bag(bag_path, topic_name)

    # Plot the GPS data on the map
    plot_gps_data_on_map(gps_data, 'data_going_in_circles.html')
