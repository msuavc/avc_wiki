---
icon: server
order: 99
---

# VPN Configuration

To access AVC compute/IT resources available for you to develop, you will need
to configure our VPN on your current system. The following instructions will tell
you how to install the VPN on whatever platform you use.

| Operating System    |
| ------------------- |
| [Ubuntu](#ubuntu)   |
| [Windows](#windows) |
| [MacOS](#macos)     |

## Ubuntu

### 1. Set up apt to use the openvpn.net repository

!!!info Ubuntu 18.04+
Ubuntu 18.04+ users can skip step one
!!!

#### a. Download the OpenVPN key and add it to Ubuntu's apt key repository

```bash
wget https://swupdate.openvpn.net/repos/repo-public.gpg
sudo apt-key add repo-public.gpg
```

Alternatively, can be done from the "Software and Updates" application:
under the "Authentication" tab, click "Import Key File," and select the
key downloaded from the above URL.

#### b. Add the repository to your apt lists

This step can be done from the GUI or from the command line.
Choose one of the methods below.

- **command line:**

```bash
sudo sh -c \
  'echo "deb http://build.openvpn.net/debian/openvpn/stable xenial main" \
      >> /etc/apt/sources.list.d/openvpn-repo.list'
```

- **gui:** in the "Software and Updates" application, under the "Other Software"
  tab, click "Add" and copy the following line into the box:
  deb <http://build.openvpn.net/debian/openvpn/stable> xenial main

#### c. Refresh your package lists

```bash
sudo apt update
```

### 2. Install OpenSSL and related support packages

```bash
sudo apt install openvpn network-manager-openvpn network-manager-openvpn-gnome
```

### 3. Configure Network Manager with OpenSSL

1. Download and uncompress all the contents of the AVC VPN configuration ZIP
   file: <https://drive.google.com/file/d/1u9A70Hsg0mK2TmBXbBXGWJ7PsEyG7i4H/view?usp=sharing>
   !!!info Credentials
   Note: for step 1 you will need to have been given access to the Google
   Drive, and be signed in using your MSU Google Apps account
   (your @msu.edu email).
   !!!
2. Open the "Settings" program and navigate to "Network". Then click "+"

![Screen_Shot_2021-01-14_at_6.54.22_PM](~/static/uploads/0d9900602e6d7100b3ae052072c0e72e/Screen_Shot_2021-01-14_at_6.54.22_PM.png)

3. Next to the VPN section, click the + button to add a new connection.

![Screenshot_from_2021-01-14_15-53-59](~/static/uploads/aee5ac5c7ff43e55df7ec7fce356a4be/Screenshot_from_2021-01-14_15-53-59.png)

4. Select "Import a saved file..."

![Screenshot_from_2021-01-14_15-58-00](~/static/uploads/b90876dd07afa2ac801b5a39e627beda/Screenshot_from_2021-01-14_15-58-00.png)

5. Browse to the "openvpn.conf" from the .zip file.

6. On the next screen, give the connection a different name if desired
   (like AVC server), fill in username (password is optional; if not filled
   you will be prompted for a password upon connecting), and hit "Save"

   !!!info Usernames and Passwords
   Note: the username and password are your DECS username and password, not
   necessarily your AVC server username and password (unless you chose to use the
   same password for both, which is generally not recommended).
   !!!

7. Once configured, select the gear icon next to the VPN configuration in the
   VPN section of the "Network" settings page, select the "IPV4" tab, then check
   "Use this connection only for resources on its network".

![Screenshot_from_2021-01-15_12-10-18](~/static/uploads/a9722ba6cb3a967afe6a5fcedb35b392/Screenshot_from_2021-01-15_12-10-18.png)

### 4. Test Connection

Click the network icon, hover over "VPN Connections", and then select the new
VPN connection you just made. You should get a message saying a VPN connection
has been successfully established.

## Windows

### 1. Download and Install OpenVPN

Download OpenVPN from:
<https://openvpn.net/community-downloads/> and follow the installer to complete installation.

### 2. Configure the VPN

#### a. Download the VPN config

The OpenVPN configuration .zip file can be downloaded from the following link: <https://drive.google.com/file/d/1u9A70Hsg0mK2TmBXbBXGWJ7PsEyG7i4H/view>

> Note: for step 1 you will need to have 1) been given access to the Google Drive,
> and b) be signed in using your MSU Google Apps account (your @msu.edu email).

#### b. Set Configurations

1. Unzip the file to a permanent location
2. Change the extension of the .conf file to .ovpn
3. Open the .conf file and find these lines:

```cmd
ca ca.crt
cert client.crt
key client.key
```

Change the ca.crt, client.crt, and client.key to be full paths, for example
(with \\ as path separators instead of single backslashes):

```cmd
ca C:\\my_vpn\\ca.crt
cert C:\\my_vpn\\client.crt
key C:\\my_vpn\\client.key
```

Then, find the lines that say:

```cmd
tls-auth ta.key 1
auth SHA512
```

Change the ta.key part of the first line to be a full path, similar to the
previous step, so it looks something like this:

```cmd
tls-auth C:\\my_vpn\\ta.key 1
auth SHA512
```

#### c. Import Configurations

Then, run the OpenVPN GUI, then right-click the tray icon, then click “Import
file”, then select your modified .ovpn file.

### 3. Connecting to the VPN

To connect, right-cick the OpenVPN tray icon and click “Connect”

!!! info Usernames and Passwords
The username and password are your DECS username and password, not necessarily
your AVC server username and password (unless you chose to use the same
password for both, which is generally not recommended).
!!!

## MacOS

To access the AVC VPN through macOS an OpenVPN client is required (not
supported natively). The preferred OpenVPN client for macOS is Tunnelblick.

### 1. Install Tunnelblick

To install Tunnelblick, download and run installer from the following page: <https://tunnelblick.net/downloads.html>

### 2. Download the VPN Configuration

Download and uncompress the AVC VPN configuration file: <https://drive.google.com/file/d/1u9A70Hsg0mK2TmBXbBXGWJ7PsEyG7i4H/view?usp=sharing>

> Note: for step 1 you will need to have 1) been given access to the Google
> Drive, and b) be signed in using your MSU Google Apps account (your @msu.edu email).

### 3. Import Tunnelblick Configurations

Once Tunnelblick is installed and running, navigate to the uncompressed zip
folder from (2), then drag the `openvpn.conf` file onto the tunnelblick "arch"
icon in the menu bar (a plus icon should appear on the cursor) and proceed to add
the configuration.

![Screen_Shot_2021-01-14_at_6.20.44_PM](~/static/uploads/b1d129ec23745260eebc31ed8e2ff245/Screen_Shot_2021-01-14_at_6.20.44_PM.png)

### 4. Connect to the VPN

To connect, click on the Tunnelblick icon in the menu bar, then select
"Connect <vpn_name>" where <vpn_name> will likely be something like "openvpn";
in the image below the configuration was renamed to "SOAR" (details on renaming
at the end of this page):

![Screen_Shot_2021-01-14_at_6.23.48_PM](~/static/uploads/db5bd49f08388ff685b45203f0a60ed4/Screen_Shot_2021-01-14_at_6.23.48_PM.png)

!!!info Username and Password
Note: the username and password are your DECS username and password, not
necessarily your autodrive username and password (unless you chose to use
the same password for both, which is generally not recommended).
!!!

To confirm you have been connected you should see:

![Screen_Shot_2021-01-14_at_6.24.47_PM](~/static/uploads/7894a9becc0578ec52a6a423a53676fa/Screen_Shot_2021-01-14_at_6.24.47_PM.png)

To disconnect, simply press "Disconnect...".
!!!info Changing the Name of the VPN Configuration
Note: the name of the VPN configuration may be changed by clicking on the
Tunnelblick icon in the menu bar, selecting "VPN Details..." then clicking
on the desired in the left-hand side of the "Configurations" tab of the window
which opens, then pressing the gear icon at the bottom of the lef-thand side-bar
and selecting "rename configuration". This can be renamed to something more recognizable
such as "AVC".
![Screen_Shot_2021-01-14_at_6.26.15_PM](~/static/uploads/15bcf458a9b6f304d0867c638a59a497/Screen_Shot_2021-01-14_at_6.26.15_PM.png)
