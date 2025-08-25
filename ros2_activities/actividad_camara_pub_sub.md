# Actividad: Procesamiento de Imágenes Inter-Nodo en ROS2

Esta actividad te guiará en la creación de dos nodos de ROS2 que trabajan juntos. Un nodo capturará video de una cámara web y el segundo nodo recibirá y procesará ese video en tiempo real.

---

## Objetivos de Aprendizaje

Al completar con éxito esta actividad, serás capaz de:

* Crear un paquete de ROS2 con nodos de Python.
* Desarrollar un **nodo publicador** que utiliza OpenCV para capturar imágenes de una cámara.
* Usar `cv_bridge` para convertir imágenes de OpenCV (`cv::Mat`) en mensajes de ROS2 (`sensor_msgs/msg/Image`).
* Desarrollar un **nodo suscriptor** que recibe mensajes de imagen.
* Usar `cv_bridge` para convertir mensajes de imagen de ROS2 de vuelta a imágenes de OpenCV para su procesamiento.
* Construir y ejecutar dos nodos simultáneamente para establecer una tubería (pipeline) completa de procesamiento de imágenes.

---

## Instrucciones

### Parte 1: Configurando el Paquete

1.  **Navega a tu Espacio de Trabajo**: Abre una terminal y ve al directorio `src` de tu espacio de trabajo de ROS2.
    ```bash
    cd ~/ros2_ws/src
    ```

2.  **Crea el Paquete de ROS2**: Crea un nuevo paquete de Python llamado `ros2_inter_node_practice` con las dependencias necesarias.
    ```bash
    ros2 pkg create --build-type ament_python ros2_inter_node_practice --dependencies rclpy sensor_msgs cv_bridge image_transport
    ```

### Parte 2: Creando el Nodo Publicador de Imágenes

1.  **Navega al Directorio del Paquete**:
    ```bash
    cd ~/ros2_ws/src/ros2_inter_node_practice/ros2_inter_node_practice
    ```

2.  **Crea el Archivo Python**: Crea un nuevo archivo llamado `camera_publisher.py`.
    ```bash
    touch camera_publisher.py
    ```

3.  **Añade el Código**: Abre `camera_publisher.py` y pega el siguiente código. Este nodo abrirá tu cámara web, capturará fotogramas y los publicará en el tópico `/camera/image_raw`.

    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class CameraPublisher(Node):
        def __init__(self):
            super().__init__('camera_publisher')
            self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
            timer_period = 0.05  # segundos (para ~20 FPS)
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.cap = cv2.VideoCapture(0) # Usa 0 para la cámara web por defecto
            self.br = CvBridge()
            self.get_logger().info('Nodo Publicador de Cámara iniciado.')

        def timer_callback(self):
            ret, frame = self.cap.read()
            if ret:
                self.publisher_.publish(self.br.cv2_to_imgmsg(frame, 'bgr8'))
                # self.get_logger().info('Publicando fotograma de video') # Descomenta para depurar
            else:
                self.get_logger().warn('No se pudo leer el fotograma de la cámara.')

    def main(args=None):
        rclpy.init(args=args)
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
        camera_publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### Parte 3: Creando el Nodo de Procesamiento de Imágenes

1.  **Crea el Archivo Python**: En el mismo directorio, crea un archivo llamado `image_processor.py`.
    ```bash
    touch image_processor.py
    ```

2.  **Añade el Código**: Abre `image_processor.py` y pega el siguiente código. Este nodo se suscribe al tópico `/camera/image_raw`, convierte las imágenes a escala de grises y las muestra.

    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImageProcessor(Node):
        def __init__(self):
            super().__init__('image_processor')
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.listener_callback,
                10)
            self.br = CvBridge()
            self.get_logger().info('Nodo Procesador de Imágenes iniciado.')

        def listener_callback(self, data):
            # self.get_logger().info('Recibiendo fotograma de video') # Descomenta para depurar
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
            
            # --- Lógica de Procesamiento de Imagen ---
            gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            
            # Muestra el fotograma procesado
            cv2.imshow("Video en Escala de Grises", gray_frame)
            cv2.waitKey(1)

    def main(args=None):
        rclpy.init(args=args)
        image_processor = ImageProcessor()
        rclpy.spin(image_processor)
        image_processor.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### Parte 4: Construyendo y Ejecutando los Nodos

1.  **Configura `setup.py`**: Abre el archivo `setup.py` ubicado en `~/ros2_ws/src/ros2_inter_node_practice/` y añade los `entry_points` para registrar tus nodos como ejecutables.
    ```python
    # Dentro de la función setup(), añade:
    entry_points={
        'console_scripts': [
            'camera_pub = ros2_inter_node_practice.camera_publisher:main',
            'img_proc = ros2_inter_node_practice.image_processor:main',
        ],
    },
    ```

2.  **Construye el Paquete**: Navega a la raíz de tu espacio de trabajo y usa `colcon` para construir tu nuevo paquete.
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select ros2_inter_node_practice
    ```

3.  **Activa el "Source" del Espacio de Trabajo**: En cada nueva terminal que uses, debes activar el archivo de configuración de tu espacio de trabajo.
    ```bash
    source install/setup.bash
    ```

4.  **Ejecuta los Nodos**:
    * En tu **primera terminal**, activa el "source" y ejecuta el publicador de la cámara:
        ```bash
        source ~/ros2_ws/install/setup.bash
        ros2 run ros2_inter_node_practice camera_pub
        ```
    * Abre una **segunda terminal**, activa el "source" y ejecuta el procesador de imágenes:
        ```bash
        source ~/ros2_ws/install/setup.bash
        ros2 run ros2_inter_node_practice img_proc
        ```

Ahora deberías ver una ventana titulada "Video en Escala de Grises" mostrando el video en vivo y procesado de tu cámara web.