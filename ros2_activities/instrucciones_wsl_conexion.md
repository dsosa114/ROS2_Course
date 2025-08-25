## Instructions for communicating WSL2 ROS 2 with other devices

By default, WSL2 uses NAT, meaning it will have its own IP address different from Windows. This prevents your devices and WSL2 from being on the same subnet and communicating via multicast by default. The NAT will block some traffic, and discovery won't work. The DDS protocol, used by ROS2 to send packets, requires multicast, or you need to update it to unicast (specifying the IP of the node directly).

-----

## Using Multicast

The "Mirrored Networking" mode, explained [here](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking), enables multicast support. This mode essentially allows WSL and Windows to share the same IP address.

To enable it:

1.  **Create or update** a `.wslconfig` file in Windows at `%UserProfile%\.wslconfig` with the following content:

    ```ini
    [wsl2]
    networkingMode=mirrored
    ```

2.  To **enable inbound packets**, run the following in PowerShell:

    ```powershell
    Set-NetFirewallHyperVVMSetting -Name '{%VM_GUID%}' -DefaultInboundAction Allow
    ```

    This command modifies a specific setting on a Hyper-V virtual machine (VM) firewall to allow all inbound network traffic to the VM by default.

      * `Set-NetFirewallHyperVVMSetting`: This cmdlet modifies Hyper-V VM firewall settings.
      * `-Name '{%VM_GUID%}'`: This parameter specifies the VM to configure using its unique identifier (GUID or VM ID).
      * `-DefaultInboundAction Allow`: This parameter sets the default action for incoming network traffic to `Allow`, overriding the more secure default of blocking. This is often used in testing or troubleshooting to rule out firewall issues.

-----

## Finding Your WSL2 VM GUID

You can find the GUID of a WSL2 Linux instance by checking the Windows Terminal settings or by querying an environment variable from within the WSL instance itself.

### Method 1: Check Windows Terminal Settings

1.  Open **Windows Terminal**.

2.  Go to the **Settings** menu (click the dropdown arrow in the terminal title bar and select "Settings" or press `Ctrl + ,`).

3.  The `settings.json` file will open. Look for the `"profiles"` section.

4.  Within the `profiles` section, find the `"list"` containing entries for your installed shells, including your WSL distributions.

5.  Each WSL distribution will have an entry similar to this:

    ```json
    {
        "guid": "{<your-GUID-here>}",
        "hidden": false,
        "name": "Ubuntu",
        "source": "Microsoft.WSL"
    }
    ```

6.  The value next to `"guid"` is the unique identifier for that specific WSL instance.

-----

### Method 2: Use the `WT_PROFILE_ID` Environment Variable

1.  Open your WSL instance in a terminal.

2.  Run the following command:

    ```bash
    echo $WT_PROFILE_ID
    ```

3.  This command will print the GUID of the profile you are currently using. This method is especially useful for finding the GUID of the specific instance you are running.

You can also watch a video walkthrough on how to find the profile ID (GUID) for your WSL2 instance [here](https://www.youtube.com/watch?v=vCpRU9yJw9o).

-----

After applying these changes, make sure to **restart WSL completely or reboot your system** for the settings to take effect. You should then see the same `inet` address when running `ifconfig` in WSL and `ipconfig` in Windows.

-----

To revert the command `Set-NetFirewallHyperVVMSetting -Name '{%VM_GUID%}' -DefaultInboundAction Allow`, you would use the same cmdlet but change the `-DefaultInboundAction` parameter to its original value, which is **Block**.

-----

### Reverting the Command

The original command **allowed** all inbound traffic. To revert this, you need to change the action back to **blocking** inbound traffic by default. The command to do this is:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{%VM_GUID%}' -DefaultInboundAction Block
```

  * `Set-NetFirewallHyperVVMSetting`: This is the same cmdlet used to modify the firewall settings.
  * `-Name '{%VM_GUID%}'`: This parameter still targets the specific VM by its GUID.
  * `-DefaultInboundAction Block`: This parameter is changed to **Block**, which reverts the default behavior of the firewall to its more secure state, preventing all inbound traffic by default unless a specific rule allows it.

  -----

  # Instrucciones para la comunicación de WSL2 ROS 2 con otros dispositivos

Por defecto, WSL2 utiliza NAT, lo que significa que tendrá su propia dirección IP diferente a la de Windows. Esto implica que sus otros dispositivos y WSL2 no estarán en la misma subred y no podrán comunicarse a través de multidifusión por defecto. El NAT bloqueará parte del tráfico y el descubrimiento no funcionará. El protocolo DDS para enviar paquetes en ROS2 necesita multidifusión o debe actualizarlo a unidifusión (especificando directamente la IP del nodo).

-----

## Uso de la multidifusión

El modo de "red en espejo" explicado aquí ([https://learn.microsoft.com/es-es/windows/wsl/networking\#mirrored-mode-networking](https://www.google.com/search?q=https://learn.microsoft.com/es-es/windows/wsl/networking%23mirrored-mode-networking)) habilitaría este soporte. Básicamente, WSL y Windows tendrían la misma dirección IP.

Esto implica crear/actualizar un archivo `.wslconfig` en Windows: `%UserProfile%\.wslconfig` con el siguiente contenido:

```ini
[wsl2]
networkingMode=mirrored
```

Para habilitar los paquetes entrantes, ejecute lo siguiente en PowerShell:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{%VM_GUID%}' -DefaultInboundAction Allow
```

Este comando modifica una configuración específica en un firewall de máquina virtual (VM) de Hyper-V. Permite todo el tráfico de red entrante a la VM por defecto.

  * `Set-NetFirewallHyperVVMSetting`: Este es el cmdlet (comando) principal que modifica la configuración del firewall de la VM de Hyper-V.
  * `-Name '{%VM_GUID%}'`: Este parámetro especifica la VM en particular que desea configurar. La cadena larga de números y letras entre llaves es el identificador único de la VM, también conocido como su GUID (Identificador Único Global) o ID de VM. Cada máquina virtual tiene su propio GUID distinto.
  * `-DefaultInboundAction Allow`: Este parámetro establece la acción predeterminada para cualquier tráfico de red entrante que no coincida con una regla de firewall específica y más granular. Al establecerlo en `Allow`, está abriendo la puerta a todas las conexiones entrantes a esa VM específica, anulando el valor predeterminado más seguro de bloquear dicho tráfico.

En términos simples, este comando es una excepción a la regla. En lugar de la medida de seguridad típica de bloquear el tráfico entrante por defecto y solo permitir conexiones específicas, este comando le dice al firewall que permita todo lo que entre para la VM identificada por ese código único. Esto se utiliza a menudo en entornos de prueba o de laboratorio específicos donde la seguridad es menos preocupante que la conectividad, o cuando se está solucionando un problema de conexión y se quiere descartar el firewall como la causa.

-----

## Cómo encontrar el GUID de su instancia WSL2

Puede encontrar el GUID de una instancia de Linux de WSL2 comprobando la configuración de la Terminal de Windows o consultando una variable de entorno desde la propia instancia de WSL.

### Método 1: Comprobar la configuración de la Terminal de Windows

1.  Abra la Terminal de Windows.

2.  Vaya al menú de **Configuración**. Puede hacerlo haciendo clic en la flecha desplegable en la barra de título de la terminal y seleccionando "Configuración" o presionando `Ctrl + ,`.

3.  Se abrirá el archivo `settings.json`. Dentro de este archivo, busque la sección `"profiles"`.

4.  Dentro de la sección `profiles`, encontrará una `"list"` que contiene entradas para cada uno de sus shells instalados, incluidas sus distribuciones de WSL.

5.  Cada distribución de WSL tendrá una entrada similar a esta:

    ```json
    {
        "guid": "{<su-GUID-aquí>}",
        "hidden": false,
        "name": "Ubuntu",
        "source": "Microsoft.WSL"
    }
    ```

6.  El valor junto a `"guid"` es el identificador único para esa instancia específica de WSL.

-----

### Método 2: Usar la variable de entorno `WT_PROFILE_ID`

1.  Abra su instancia de WSL en una terminal.

2.  Ejecute el siguiente comando:

    ```bash
    echo $WT_PROFILE_ID
    ```

3.  Este comando imprimirá el GUID del perfil que está utilizando actualmente. Este método es especialmente útil si está intentando encontrar el GUID de la instancia específica que está ejecutando.

[Cómo encontrar el ID de perfil de WSL 2 (GUID)](https://www.youtube.com/watch?v=vCpRU9yJw9o)

Este video proporciona un tutorial sobre cómo encontrar el ID de perfil (GUID) para su instancia WSL2.

-----

Asegúrese de reiniciar WSL por completo o reiniciar el sistema, y debería tener todo habilitado.

`ifconfig` e `ipconfig` deberían mostrar la misma dirección `inet`.