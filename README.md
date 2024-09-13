# pointcloud_db3_to_csv

Este proyecto proporciona un script Python para extraer datos de tipo `PointCloud2` desde archivos `rosbag` (formato DB3) y convertirlos en archivos CSV. Utiliza `rosbag2_py` para leer los datos del archivo `rosbag` y `sensor_msgs_py` para procesar los mensajes `PointCloud2`.

## Funcionalidades

- **Extracción de Datos**: Lee mensajes de tipo `PointCloud2` desde un archivo `rosbag` en formato DB3.
- **Exportación a CSV**: Guarda los datos de los puntos en archivos CSV, incluyendo los campos disponibles en el mensaje `PointCloud2` como `x`, `y`, `z`, `intensity`, y otros campos adicionales si están presentes.
- **Parámetros Configurables**: Permite especificar el archivo `rosbag`, el tópico de `PointCloud2`, y el directorio de salida a través de argumentos de línea de comandos.

## Requisitos

- ROS 2 (para `rosbag2_py`)
- `sensor_msgs_py`
- `argparse` (generalmente incluido en la biblioteca estándar de Python)

## Uso

1. **Clona el repositorio**:
   ```bash
   git clone https://github.com/SIMCA-USI/pointcloud_db3_to_csv.git
   cd pointcloud_db3_to_csv

2. **Instala las dependencias (si es necesario)**:
   ```bash
   pip install rosbag2_py sensor_msgs_py

3. **Ejecuta el script**:
   ```bash
   python extract_pointcloud.py path/to/rosbag /topic path/to/csv_directory
- path/to/rosbag: Ruta al rosbag completro. El scrip accede al archivo db3.
- /topic: Nombre del topic para los mensajes PointCloud2.
- path/to/csv_directory: Directorio donde se guardarán los archivos CSV.

## Contribuciones
Las contribuciones son bienvenidas. Si tienes alguna mejora o corrección, por favor, abre un issue o un pull request.

## Créditos
Este proyecto ha sido desarrollado por [Guillermo Sánchez](https://github.com/guillermosanchezg).
