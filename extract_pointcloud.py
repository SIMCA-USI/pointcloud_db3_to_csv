import rosbag2_py
from sensor_msgs_py import point_cloud2 as pc2
import csv
import os
from sensor_msgs.msg import PointCloud2
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import argparse

# Función para guardar los datos en CSV
def save_to_csv(cloud, csv_filename):
    # Obtener los nombres de los campos
    field_names = [field.name for field in cloud.fields]
    print("Campos disponibles:", field_names)
    
    # Abrir archivo CSV
    with open(csv_filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # Escribir cabecera
        csv_writer.writerow(field_names)
        # Iterar sobre los puntos del PointCloud2
        for point in pc2.read_points(cloud, field_names=field_names, skip_nans=True):
            print(point)
            csv_writer.writerow(point)

# Leer el rosbag
def extract_pointclouds_from_bag(bag_filename, topic, csv_dir):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_filename, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    # Recorrer los mensajes
    while reader.has_next():
        (msg_topic, msg, t) = reader.read_next()
        if msg_topic == topic:
            # Obtener el tipo de mensaje para el topic
            msg_type = get_message('sensor_msgs/msg/PointCloud2')
            # Deserializar el mensaje
            deserialized_msg = deserialize_message(msg, msg_type)
            # Guardar el mensaje deserializado
            save_to_csv(deserialized_msg, os.path.join(csv_dir, f"pointcloud_{t}.csv"))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract and save PointCloud2 data from a rosbag file.')
    parser.add_argument('bag_filename', type=str, help='Path to the rosbag file.')
    parser.add_argument('topic', type=str, help='Topic name for the PointCloud2 messages.')
    parser.add_argument('csv_dir', type=str, help='Directory to save the CSV files.')

    args = parser.parse_args()

    # Crear el directorio si no existe
    os.makedirs(args.csv_dir, exist_ok=True)

    # Llamar la función para extraer y convertir los PointClouds
    extract_pointclouds_from_bag(args.bag_filename, args.topic, args.csv_dir)

