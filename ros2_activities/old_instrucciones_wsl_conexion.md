

````markdown:instructions for communicating with other devices:instrucciones_wsl_conexion_en.md
# Instructions for communicating WSL2 ROS 2 with other devices

[cite_start]By default, WSL2 uses NAT, meaning it will have its own IP address different from Windows[cite: 1, 2]. [cite_start]This prevents your devices and WSL2 from being on the same subnet and communicating via multicast by default[cite: 2]. [cite_start]The NAT will block some traffic, and discovery won't work[cite: 3]. [cite_start]The DDS protocol, used by ROS2 to send packets, requires multicast, or you need to update it to unicast (specifying the IP of the node directly)[cite: 4].

## Using Multicast

[cite_start]The "Mirrored Networking" mode, explained [here](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking), enables multicast support[cite: 5]. [cite_start]This mode essentially allows WSL and Windows to share the same IP address[cite: 6].

To enable it:

1. [cite_start]**Create or update** a `.wslconfig` file in Windows at `%UserProfile%\.wslconfig` with the following content[cite: 7]:

   ```ini
   [wsl2]
   networkingMode=mirrored
````

2.  [cite\_start]To **enable inbound packets**, run the following in PowerShell[cite: 7]:

    ```powershell
    Set-NetFirewallHyperVVMSetting -Name '{%VM_GUID%}' -DefaultInboundAction Allow
    ```

    [cite\_start]This command modifies a specific setting on a Hyper-V virtual machine (VM) firewall to allow all inbound network traffic to the VM by default[cite: 8].

      * [cite\_start]`Set-NetFirewallHyperVVMSetting`: This cmdlet modifies Hyper-V VM firewall settings[cite: 9].
      * [cite\_start]`-Name '{%VM_GUID%}'`: This parameter specifies the VM to configure using its unique identifier (GUID or VM ID)[cite: 10, 11].
      * [cite\_start]`-DefaultInboundAction Allow`: This parameter sets the default action for incoming network traffic to `Allow`, overriding the more secure default of blocking[cite: 12, 13, 15]. [cite\_start]This is often used in testing or troubleshooting to rule out firewall issues[cite: 16].

## Finding Your WSL2 VM GUID

[cite\_start]You can find the GUID of a WSL2 Linux instance by checking the Windows Terminal settings or by querying an environment variable from within the WSL instance itself[cite: 17].

### Method 1: Check Windows Terminal Settings

1.  [cite\_start]Open **Windows Terminal**[cite: 18].

2.  [cite\_start]Go to the **Settings** menu by clicking the dropdown arrow in the terminal title bar and selecting "Settings" or by pressing `Ctrl + ,`[cite: 19].

3.  [cite\_start]The `settings.json` file will open[cite: 20]. [cite\_start]Look for the `"profiles"` section[cite: 21].

4.  [cite\_start]Within the `profiles` section, find the `"list"` containing entries for your installed shells, including your WSL distributions[cite: 21].

5.  [cite\_start]Each WSL distribution will have an entry similar to this[cite: 22]:

    ```json
    {
        "guid": "{<your-GUID-here>}",
        "hidden": false,
        "name": "Ubuntu",
        "source": "Microsoft.WSL"
    }
    ```

6.  [cite\_start]The value next to `"guid"` is the unique identifier for that specific WSL instance[cite: 22].

### Method 2: Use the `WT_PROFILE_ID` Environment Variable

1.  [cite\_start]Open your WSL instance in a terminal[cite: 24].

2.  [cite\_start]Run the following command[cite: 24]:

    ```bash
    echo $WT_PROFILE_ID
    ```

3.  [cite\_start]This command will print the GUID of the profile you are currently using[cite: 25]. [cite\_start]This method is especially useful for finding the GUID of the specific instance you are running[cite: 26].

[cite\_start]You can also watch a video walkthrough on how to find the profile ID (GUID) for your WSL2 instance [here](https://www.youtube.com/watch?v=vCpRU9yJw9o)[cite: 26, 27].

[cite\_start]After applying these changes, make sure to **restart WSL completely or reboot your system** for the settings to take effect[cite: 28]. [cite\_start]You should then see the same `inet` address when running `ifconfig` in WSL and `ipconfig` in Windows[cite: 28].

### Reverting the Command

The original command **allowed** all inbound traffic. To revert this, you need to change the action back to **blocking** inbound traffic by default. The command to do this is:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{%VM_GUID%}' -DefaultInboundAction Block
```

  * `Set-NetFirewallHyperVVMSetting`: This is the same cmdlet used to modify the firewall settings.
  * `-Name '{%VM_GUID%}'`: This parameter still targets the specific VM by its GUID.
  * `-DefaultInboundAction Block`: This parameter is changed to **Block**, which reverts the default behavior of the firewall to its more secure state, preventing all inbound traffic by default unless a specific rule allows it.

## Troubleshooting

If you get issues with the `ros2 node list` command, it could be that the daemon service is not running. This command should fix it:

```bash
ros2 daemon start
```

If you still have issues detecting nodes on other machines, it is likely that the Windows firewall is blocking some of the traffic. Run these commands (in an administrator PowerShell) to make exceptions for ROS2 ports, and afterwards reboot your machine:

```powershell
New-NetFirewallRule -DisplayName "ROS2 UDP 7400-7600" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 7400-7600
```

```powershell
New-NetFirewallRule -DisplayName "ROS2 UDP 7400-7600 Outbound" -Direction Outbound -Action Allow -Protocol UDP -LocalPort 7400-7600
```

````eof
```markdown:Instrucciones para la comunicacion de WSL2 con otros dispositivos:instrucciones_wsl_conexion_es.md
# Instrucciones para la comunicación de WSL2 ROS 2 con otros dispositivos

[cite_start]Por defecto, WSL2 utiliza NAT, lo que significa que tendrá su propia dirección IP diferente a la de Windows[cite: 1, 2]. [cite_start]Esto implica que sus otros dispositivos y WSL2 no estarán en la misma subred y no podrán comunicarse a través de multidifusión por defecto[cite: 2]. [cite_start]El NAT bloqueará parte del tráfico y el descubrimiento no funcionará[cite: 3]. [cite_start]El protocolo DDS para enviar paquetes en ROS2 necesita multidifusión o debe actualizarlo a unidifusión (especificando directamente la IP del nodo)[cite: 4].

## Uso de la multidifusión

[cite_start]El modo de "red en espejo" explicado aquí ([https://learn.microsoft.com/es-es/windows/wsl/networking#mirrored-mode-networking](https://www.google.com/search?q=https://learn.microsoft.com/es-es/windows/wsl/networking%23mirrored-mode-networking)) habilitaría este soporte[cite: 5]. [cite_start]Básicamente, WSL y Windows tendrían la misma dirección IP[cite: 6].

[cite_start]Esto implica crear/actualizar un archivo `.wslconfig` en Windows: `%UserProfile%\.wslconfig` con el siguiente contenido[cite: 7]:

```ini
[wsl2]
networkingMode=mirrored
````

[cite\_start]Para habilitar los paquetes entrantes, ejecute lo siguiente en PowerShell[cite: 7]:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{%VM_GUID%}' -DefaultInboundAction Allow
```

Este comando modifica una configuración específica en un firewall de máquina virtual (VM) de Hyper-V. [cite\_start]Permite todo el tráfico de red entrante a la VM por defecto[cite: 8].

  * [cite\_start]`Set-NetFirewallHyperVVMSetting`: Este es el cmdlet (comando) principal que modifica la configuración del firewall de la VM de Hyper-V[cite: 9].
  * `-Name '{%VM_GUID%}'`: Este parámetro especifica la VM en particular que desea configurar. La cadena larga de números y letras entre llaves es el identificador único de la VM, también conocido como su GUID (Identificador Único Global) o ID de VM. [cite\_start]Cada máquina virtual tiene su propio GUID distinto[cite: 10, 11].
  * `-DefaultInboundAction Allow`: Este parámetro establece la acción predeterminada para cualquier tráfico de red entrante que no coincida con una regla de firewall específica y más granular. [cite\_start]Al establecerlo en `Allow`, está abriendo la puerta a todas las conexiones entrantes a esa VM específica, anulando el valor predeterminado más seguro de bloquear dicho tráfico[cite: 12, 13, 15]. [cite\_start]En términos simples, este comando es una excepción a la regla[cite: 14]. [cite\_start]Esto se utiliza a menudo en entornos de prueba o de laboratorio específicos donde la seguridad es menos preocupante que la conectividad, o cuando se está solucionando un problema de conexión y se quiere descartar el firewall como la causa[cite: 16].

## Cómo encontrar el GUID de su instancia WSL2

[cite\_start]Puede encontrar el GUID de una instancia de Linux de WSL2 comprobando la configuración de la Terminal de Windows o consultando una variable de entorno desde la propia instancia de WSL[cite: 17].

### Método 1: Comprobar la configuración de la Terminal de Windows

1.  [cite\_start]Abra la Terminal de Windows[cite: 18].

2.  Vaya al menú de **Configuración**. [cite\_start]Puede hacerlo haciendo clic en la flecha desplegable en la barra de título de la terminal y seleccionando "Configuración" o presionando `Ctrl + ,`[cite: 19].

3.  [cite\_start]Se abrirá el archivo `settings.json`[cite: 20]. [cite\_start]Dentro de este archivo, busque la sección `"profiles"`[cite: 21].

4.  [cite\_start]Dentro de la sección `profiles`, encontrará una `"list"` que contiene entradas para cada uno de sus shells instalados, incluidas sus distribuciones de WSL[cite: 21].

5.  [cite\_start]Cada distribución de WSL tendrá una entrada similar a esta[cite: 22]:

    ```json
    {
        "guid": "{<su-GUID-aquí>}",
        "hidden": false,
        "name": "Ubuntu",
        "source": "Microsoft.WSL"
    }
    ```

6.  [cite\_start]El valor junto a `"guid"` es el identificador único para esa instancia específica de WSL[cite: 22].

### Método 2: Usar la variable de entorno `WT_PROFILE_ID`

1.  [cite\_start]Abra su instancia de WSL en una terminal[cite: 24].

2.  [cite\_start]Ejecute el siguiente comando[cite: 24]:

    ```bash
    echo $WT_PROFILE_ID
    ```

3.  [cite\_start]Este comando imprimirá el GUID del perfil que está utilizando actualmente[cite: 25]. [cite\_start]Este método es especialmente útil si está intentando encontrar el GUID de la instancia específica que está ejecutando[cite: 26].

[Cómo encontrar el ID de perfil de WSL 2 (GUID)](https://www.youtube.com/watch?v=vCpRU9yJw9o)

[cite\_start]Este video proporciona un tutorial sobre cómo encontrar el ID de perfil (GUID) para su instancia WSL2[cite: 26, 27].

[cite\_start]Asegúrese de reiniciar WSL por completo o reiniciar el sistema, y debería tener todo habilitado[cite: 28].

[cite\_start]`ifconfig` e `ipconfig` deberían mostrar la misma dirección `inet`[cite: 28].

### Revertir el Comando

El comando original **permitía** todo el tráfico entrante. Para revertir esto, debe cambiar la acción de nuevo a **bloquear** el tráfico entrante por defecto. El comando para hacerlo es:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{%VM_GUID%}' -DefaultInboundAction Block
```

  * `Set-NetFirewallHyperVVMSetting`: Este es el mismo cmdlet utilizado para modificar la configuración del firewall.
  * `-Name '{%VM_GUID%}'`: Este parámetro sigue apuntando a la VM específica por su GUID.
  * `-DefaultInboundAction Block`: Este parámetro se cambia a **Block**, lo que revierte el comportamiento predeterminado del firewall a su estado más seguro, impidiendo todo el tráfico entrante por defecto a menos que una regla específica lo permita.

## Solución de Problemas

Si tiene problemas con el comando `ros2 node list`, podría ser que el servicio de demonio no se esté ejecutando. Este comando debería solucionarlo:

```bash
ros2 daemon start
```

Si aún tiene problemas para detectar nodos en otras máquinas, es probable que el firewall de Windows esté bloqueando parte del tráfico. Ejecute estos comandos (en un PowerShell de administrador) para hacer excepciones para los puertos de ROS2, y luego reinicie su máquina:

```powershell
New-NetFirewallRule -DisplayName "ROS2 UDP 7400-7600" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 7400-7600
```

```powershell
New-NetFirewallRule -DisplayName "ROS2 UDP 7400-7600 Outbound" -Direction Outbound -Action Allow -Protocol UDP -LocalPort 7400-7600
```

```eof
```