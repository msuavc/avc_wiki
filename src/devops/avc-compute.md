---
title: AVC Remote Computing and IT Servers
icon: browser
order: 100
---

The AVC IT network consists of many computing devices to facilitate the
development and simulation needs of the organization. All team members
may have a network account created, upon request, to gain access to the compute resources.

!!!warning Outside Connections
Please note that to access these devices outside of Spartan Village, a
[VPN connection](~/devops/vpn-configuration) must be used. Ask a team/project
lead about getting a VPN.
!!!

## Quick Links

### Physical Resources

| Device Name                                                             | Device URL                 |
| ----------------------------------------------------------------------- | -------------------------- |
| [NUC Server](#intel-nuc)                                                | autodrive-nuc.egr.msu.edu  |
| [DevOps Server](#devopsitmapping-server-carla-and-deep-learning-server) | autodrive-dev.egr.msu.edu  |
| [NAS](#synology-nas-network-attached-storage)                           | autodrive-nas.egr.msu.edu  |
| [Bolt (Vehicle Server)](#bolt-server-driving-computer)                  | autodrive-bolt.egr.msu.edu |

## VPN Access

All AVC resources can be accessed through the team VPN. For more
information and to configure this, refer to the
[VPN configuration](~/devops/vpn-configuration) wiki page.

## User Accounts

User accounts are managed using the Lightweight Directory Access Protocol (LDAP)
server hosted on the NAS. Each user has a file share associated with their LDAP
account as well which is mounted as their ‘/home’ folder on Linux servers.

When your account is first created you have a temporary password sent to you via
Slack from an IT administrator. To change this password you must first need to
log into [http://autodrive-nas.egr.msu.edu](http://autodrive-nas.egr.msu.edu).
You will be prompted to change your password upon your first login. After that
your account will be fully configured with your new password and you will be
able to log into other compute devices on the network with your new password.

## Web Server Certificates

There are two web interfaces accessible on the network: the NAS (for file
access/sharing, and network credential managements). This server contains
a self-signed SSL certificates, thus they may throw browser warnings. To
avoid this, you may need to set a permanent exemption in your OS or browser.
This may be possible in-browser, or by adding the certificates in this folder
to the exemptions list on your device/browser:
[https://drive.google.com/drive/folders/1CSdak3fJWRf3VVBJbs8RgVj0G6yy14DI](https://drive.google.com/drive/folders/1CSdak3fJWRf3VVBJbs8RgVj0G6yy14DI)

## Computing Resources

### Intel NUC

URL: `autodrive-nuc.egr.msu.edu (35.9.177.163)`

#### Description

The Intel NUC platform is the primary prototyping device for ROS software.

#### Device Access

### SSH

This device can be accessed via SSH by issuing the command:
ssh msu_netid@autodrive-nuc.egr.msu.edu
To achieve X-window forwarding use:

```bash
ssh -YC msu_netid@autodrive-nuc.egr.msu.edu
```

#### Hardware Specifications

| Component | Specification                                     |
| --------- | ------------------------------------------------- |
| CPU       | Intel® Core™ i7-6770HQ Processor                  |
|           | (64-bit, Quad-Core (With HT), 6M Cache, 3.50 GHz) |
| RAM       | 32 GB DDR4-2133 (34.1 GB/s)                       |
| Storage   | 200 GB M.2 SSD (1/2 Slots Occupied)               |
| GPU       | Intel® Iris™ Pro Graphics 580                     |

#### Software Specifications

| Parameter        | Specification                                          |
| ---------------- | ------------------------------------------------------ |
| Operating System | Ubuntu 16.04.3 LTS (GNU/Linux 4.7.0.intel.r5.0 x86_64) |

### DevOps/IT/Mapping Server (CARLA, and Deep Learning Server)

URL: `autodrive-dev.egr.msu.edu (35.9.177.167)`

#### Description

The DevOps server is a custom compute server built to run CARLA simulations,
and perform rapid deep learning model training.

This server is additionally used to run the mapping server for off-vehicle
applications and development for on-vehicle usage.

#### Device Access

### SSH

This device can be accessed via SSH by issuing the command:
ssh msu_netid@autodrive-dev.egr.msu.edu
To achieve X-window forwarding use:

```bash
ssh -YC msu_netid@autodrive-dev.egr.msu.edu
```

#### TurboVNC

This device can be accessed with a graphically accelerated virtual desktop remotely by using TurboVNC+VirtualGL.
For more information, refer to the [virtual remote desktop access](~/devops/vpn-configuration) wiki page.

##### Hardware Specifications

| Component | Specification                                             |
| --------- | --------------------------------------------------------- |
| CPU       | Intel® Xeon™ E5-1650v4 Processor (64-bit,                 |
|           | Six-Core (With HT), 15M Cache, 3.6GHz)                    |
| RAM       | 32 GB DDR4-2400 ECC (43.2 GB/s)                           |
| Storage   | 256 GB M.2 SSD (W:560 MB/s, R:530 MB/s)                   |
| GPU-0     | Nvidia® 1080 Ti (11.26 GB G5X, 484.4 GB/s, 1556/1670 MHz) |
| GPU-1     | Nvidia® Titan™ Xp (12GB G5X, 547.7 GB/s, 1582 MHz)        |
| FPGA      | Intel® Arria® 10 GX FPGA                                  |

##### Software Specifications

| Parameter        | Specification                                           |
| ---------------- | ------------------------------------------------------- |
| Operating System | Ubuntu 16.04.5 LTS (GNU/Linux 4.4.0-138-generic x86_64) |

### Simulation Server (Matlab and CARLA)(Currently Offline)

URL: `autodrive-sim.egr.msu.edu (35.9.177.165)`

#### Description

The Simulation server is dedicated to running autonomous vehicle simulations in
the Matlab autonomous vehicle simulator and the CARLA vehicle simulator.

#### Device Access

##### SSH

This device can be accessed via SSH by issuing the command:
ssh msu_netid@autodrive-sim.egr.msu.edu
To achieve X-window forwarding use:

```
ssh -YC msu_netid@autodrive-sim.egr.msu.edu
```

##### Hardware Specifications

| Component | Specification                                                   |
| --------- | --------------------------------------------------------------- |
| CPU       | Intel® i7™ Processor (64-bit, Four-Core (With HT),              |
|           | 15M Cache, 4.8GHz)                                              |
| RAM       | 16 GB DDR4-3000 ECC (43.2 GB/s)                                 |
| Storage   | 256 GB M.2 SSD (W:560 MB/s, R:530 MB/s), 2x 1.8 TB HDD (RAID 1) |

##### Software Specifications

| Parameter        | Specification    |
| ---------------- | ---------------- |
| Operating System | Ubuntu 18.04 LTS |

### Synology NAS (Network Attached Storage)

URL: `autodrive-nas.egr.msu.edu (35.9.177.164)`

#### Description

The Network Attached Storage device is a lightweight, ARM-based, server which hosts
all shared file storage as well as users’ personal home directories. In addition,
the NAS also hosts the Lightweight Directory Access Protocol (LDAP) server, which
is used to manage all trust and authentication for the lab infrastructure.
All user accounts are managed through the NAS device.

#### Device Access

##### Webpage

This device can be accessed via the device webpage [<https://autodrive-nas.egr.msu.edu>](https://autodrive-nas.egr.msu.edu). From here, all personal files and team-wide datasets can be viewed, uploaded to, and downloaded from. Additionally your LDAP password may be updated from this website as well.

#### Hardware Specifications

| Component | Specification                                            |
| --------- | -------------------------------------------------------- |
| CPU       | Realtek RTD1296 Processor (64-bit, Quad Core 1.4 GHz)    |
| RAM       | 2 GB DDR4                                                |
| Storage   | 10 TB – 2x 4TB RAID1 + 2x 6TB RAID1 (4/4 Slots Occupied) |

#### Software Specifications

| Parameter        | Specification                          |
| ---------------- | -------------------------------------- |
| Operating System | Synology OS (GNU/Linux 4.4.15+ Kernel) |

### Bolt Server (Driving Computer)

URL: `autodrive-bolt.egr.msu.edu (35.9.177.166)`

#### Description

The Bolt Server is the main driving computer installed in the Chevrolet Bolt vehicle. Aside from actual vehicle testing and data collection, this computer is to be used only for low-level testing which requires the real hardware interfaces in the vehicle (drivers, etc.), performance testing and benchmarking requiring the actual vehicle hardware, and integration testing requiring all software running simultaneously on the same platform.

#### Device Access

##### SSH

This device can be accessed via SSH by issuing the command:
ssh msu_netid@autodrive-bolt.egr.msu.edu
To achieve X-window forwarding use:

```
ssh -YC msu_netid@autodrive-bold.egr.msu.edu
```

#### Hardware Specifications

| Component | Specification                                  |
| --------- | ---------------------------------------------- |
| CPU       | (2x, QPI) Intel® Xeon™ E5 E5-2699 V4 Processor |
|           | (64-bit, 22-Core (With HT), 55M Cache, 3.6GHz) |
| RAM       | 128 GB DDR4-2400 ECC (153.6 GB/s)              |
| Storage   | 128 GB NVMe SSD                                |
| FPGA      | Intel® Arria® 10 GX FPGA                       |

#### Software Specifications

| Parameter        | Specification |
| ---------------- | ------------- |
| Operating System | Ubuntu 16.04  |
