## Instructions for communicating WSL2 ROS 2 with other devices

By default, WSL2 uses NAT, meaning it will have its own IP address different from Windows. This prevents your Raspberry Pi (RPi) and WSL2 from being on the same subnet and communicating via multicast by default. The NAT will block some traffic, and discovery won't work. The DDS protocol, used by ROS2 to send packets, requires multicast, or you need to update it to unicast (specifying the IP of the node directly).

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