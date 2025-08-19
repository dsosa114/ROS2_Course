## Add Devices to WSL Instructions

### Terminal Commands

The following commands should be executed in a **PowerShell terminal as an administrator**.

1.  **Install `usbipd`**:
    `winget install usbipd`

2.  **Identify the USB device**:
    `usbipd list`

3.  **Share the device with WSL**:
    Replace `X-X` with the desired device's `busid` number.
    `usbipd bind --busid=X-X`

4.  **Attach the device to WSL**:
    `usbipd attach --wsl --busid=X-X`

| Optional Commands |
| :--- |
| If you receive the error `"usbipd: error: Loading vhci_hcd failed."` when attaching a camera, run `sudo modprobe vhci_hcd` inside your WSL environment. |
| If you get a `VIDEOIO(V4L2:/dev/video0): select() timeout.` error, it may be due to missing codecs in the WSL environment. To fix this, add the following line to your OpenCV code before the capture loop: `camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))`. |

-----

### Python Code for Testing Camera in WSL

Below is a script to test if the camera is working correctly within the WSL environment.

```python
import cv2

def read_usb_camera():
    """
    Reads frames from the first available USB camera and displays them.
    Press 'q' to quit the application.
    """
    # Open the default camera (usually the first available USB camera)
    # The argument '0' refers to the default camera. If you have multiple cameras,
    # you might need to try '1', '2', etc.
    cap = cv2.VideoCapture(0)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Camera opened successfully. Press 'q' to quit.")

    # To avoid the timeout error inside WSL maybe due to the video codecs
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        # If 'ret' is False, it means there was an error reading the frame
        if not ret:
            print("Error: Could not read frame.")
            break

        # Display the captured frame
        cv2.imshow('USB Camera Feed', frame)

        # Wait for 1 millisecond and check if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and destroy all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
    print("Camera released and windows closed.")

if __name__ == "__main__":
    read_usb_camera()

```

# Instrucciones para Añadir Dispositivos a WSL

### Comandos de Terminal

Los siguientes comandos deben ejecutarse en una **terminal de PowerShell como administrador**.

1.  **Instalar `usbipd`**:
    `winget install usbipd`

2.  **Identificar el dispositivo USB**:
    `usbipd list`

3.  **Habilitar la opción de compartir el dispositivo con WSL**:
    Reemplace `X-X` con el `busid` del dispositivo deseado.
    `usbipd bind --busid=X-X`

4.  **Adjuntar el dispositivo a WSL**:
    `usbipd attach --wsl --busid=X-X`

| **Comandos Opcionales** |
| :--- |
| Si aparece el error `"usbipd: error: Loading vhci_hcd failed."` al adjuntar una cámara, ejecute `sudo modprobe vhci_hcd` dentro de su entorno WSL. |
| Si obtiene un error similar a este: `VIDEOIO(V4L2:/dev/video0): select() timeout.`, podría deberse a la falta de códecs en el entorno WSL. Para solucionarlo, añada la siguiente línea a su código de OpenCV antes del bucle de captura: `camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))`. |

-----

### Código Python para Probar la Cámara en WSL

A continuación se presenta un script para probar si la cámara funciona correctamente dentro del entorno WSL.

```python
import cv2

def read_usb_camera():
    """
    Lee fotogramas de la primera cámara USB disponible y los muestra.
    Presione 'q' para salir de la aplicación.
    """
    # Abre la cámara predeterminada (normalmente la primera cámara USB disponible)
    # El argumento '0' se refiere a la cámara predeterminada. Si tiene varias cámaras,
    # es posible que necesite probar '1', '2', etc.
    cap = cv2.VideoCapture(0)

    # Verifica si la cámara se abrió correctamente
    if not cap.isOpened():
        print("Error: No se pudo abrir la cámara.")
        return

    print("Cámara abierta correctamente. Presione 'q' para salir.")

    # Para evitar el error de tiempo de espera dentro de WSL, posiblemente debido a los códecs de video
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    while True:
        # Lee un fotograma de la cámara
        ret, frame = cap.read()

        # Si 'ret' es False, significa que hubo un error al leer el fotograma
        if not ret:
            print("Error: No se pudo leer el fotograma.")
            break

        # Muestra el fotograma capturado
        cv2.imshow('Alimentación de Cámara USB', frame)

        # Espera 1 milisegundo y verifica si se presiona la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libera la cámara y destruye todas las ventanas de OpenCV
    cap.release()
    cv2.destroyAllWindows()
    print("Cámara liberada y ventanas cerradas.")

if __name__ == "__main__":
    read_usb_camera()

```