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
    Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
    ```

    This command modifies a specific setting on a Hyper-V virtual machine (VM) firewall to allow all inbound network traffic to the VM by default.

      * `Set-NetFirewallHyperVVMSetting`: This cmdlet modifies Hyper-V VM firewall settings.
      * `-Name '{%VM_GUID%}'`: This parameter specifies the VM to configure using its unique identifier (GUID or VM ID).
      * `-DefaultInboundAction Allow`: This parameter sets the default action for incoming network traffic to `Allow`, overriding the more secure default of blocking. This is often used in testing or troubleshooting to rule out firewall issues.

-----

After applying these changes, make sure to **restart WSL completely or reboot your system** for the settings to take effect. You should then see the same `inet` address when running `ifconfig` in WSL and `ipconfig` in Windows.

-----

To revert the command `Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow`, you would use the same cmdlet but change the `-DefaultInboundAction` parameter to its original value, which is **Block**.

-----

### Reverting the Command

The original command **allowed** all inbound traffic. To revert this, you need to change the action back to **blocking** inbound traffic by default. The command to do this is:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Block
```

  * `Set-NetFirewallHyperVVMSetting`: This is the same cmdlet used to modify the firewall settings.
  * `-Name '{%VM_GUID%}'`: This parameter still targets the specific VM by its GUID.
  * `-DefaultInboundAction Block`: This parameter is changed to **Block**, which reverts the default behavior of the firewall to its more secure state, preventing all inbound traffic by default unless a specific rule allows it.

  -----

## Troubleshooting

If you get issues with the `ros2 node list`, `ros2 node list`, or other CLI command, it could be that the daemon service is not running. This command should fix it:

```bash
ros2 daemon start
```

If you still have issues detecting nodes on other machines, it is likely that the Windows firewall is blocking some of the traffic. Run these commands (in an administrator PowerShell) to make exceptions for ROS2 ports, and afterwards reboot your machine:

```powershell
New-NetFirewallRule -DisplayName "ROS2 UDP 7400-7600" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 7400-7600 -Profile Private
```

```powershell
New-NetFirewallRule -DisplayName "ROS2 UDP 7400-7600 Outbound" -Direction Outbound -Action Allow -Protocol UDP -LocalPort 7400-7600 -Profile Private
```
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
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

Este comando modifica una configuración específica en un firewall de máquina virtual (VM) de Hyper-V. Permite todo el tráfico de red entrante a la VM por defecto.

  * `Set-NetFirewallHyperVVMSetting`: Este es el cmdlet (comando) principal que modifica la configuración del firewall de la VM de Hyper-V.
  * `-Name '{%VM_GUID%}'`: Este parámetro especifica la VM en particular que desea configurar. La cadena larga de números y letras entre llaves es el identificador único de la VM, también conocido como su GUID (Identificador Único Global) o ID de VM. Cada máquina virtual tiene su propio GUID distinto.
  * `-DefaultInboundAction Allow`: Este parámetro establece la acción predeterminada para cualquier tráfico de red entrante que no coincida con una regla de firewall específica y más granular. Al establecerlo en `Allow`, está abriendo la puerta a todas las conexiones entrantes a esa VM específica, anulando el valor predeterminado más seguro de bloquear dicho tráfico.

En términos simples, este comando es una excepción a la regla. En lugar de la medida de seguridad típica de bloquear el tráfico entrante por defecto y solo permitir conexiones específicas, este comando le dice al firewall que permita todo lo que entre para la VM identificada por ese código único. Esto se utiliza a menudo en entornos de prueba o de laboratorio específicos donde la seguridad es menos preocupante que la conectividad, o cuando se está solucionando un problema de conexión y se quiere descartar el firewall como la causa.

-----

Asegúrese de reiniciar WSL por completo o reiniciar el sistema, y debería tener todo habilitado. `ifconfig` e `ipconfig` deberían mostrar la misma dirección `inet`.

### Revertir el Comando

El comando original **permitía** todo el tráfico entrante. Para revertir esto, debe cambiar la acción de nuevo a **bloquear** el tráfico entrante por defecto. El comando para hacerlo es:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Block
```

  * `Set-NetFirewallHyperVVMSetting`: Este es el mismo cmdlet utilizado para modificar la configuración del firewall.
  * `-Name '{%VM_GUID%}'`: Este parámetro sigue apuntando a la VM específica por su GUID.
  * `-DefaultInboundAction Block`: Este parámetro se cambia a **Block**, lo que revierte el comportamiento predeterminado del firewall a su estado más seguro, impidiendo todo el tráfico entrante por defecto a menos que una regla específica lo permita.

## Solución de Problemas

Si tiene problemas con el comando `ros2 node list`, `ros2 topic list`, etc., podría ser que el servicio de demonio no se esté ejecutando. Este comando debería solucionarlo:

```bash
ros2 daemon start
```

Si aún tiene problemas para detectar nodos en otras máquinas, es probable que el firewall de Windows esté bloqueando parte del tráfico. Ejecute estos comandos (en un PowerShell de administrador) para hacer excepciones para los puertos de ROS2, y luego reinicie su máquina:

```powershell
New-NetFirewallRule -DisplayName "ROS2 UDP 7400-7600" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 7400-7600 -Profile Private
```

```powershell
New-NetFirewallRule -DisplayName "ROS2 UDP 7400-7600 Outbound" -Direction Outbound -Action Allow -Protocol UDP -LocalPort 7400-7600 -Profile Private
```

-----
