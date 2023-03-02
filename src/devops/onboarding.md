---
title: New Member Credentials
icon: diff-added
order: 96
---

SOAR has a lot of development resources that all need permissions in order to operate.
:This page serves as a resource for Team and Program Leads to get new students into SOAR.
!!!danger Non-Admins
_Do not attempt to do this as a new member, you do not have the permissions._
!!!

## VPN

For people who need to access the VPN, DECS needs to be contacted. Use this only for those who do not have the
resources to develop in a VM/on their own machine.

Once they have access, direct members to the [vpn configuration](~/devops/vpn-configuration)

## Development Server

To access the dev server, LDAP credentials need to be created. This will allow for members to have a home directory and login
credentials for the various remote resources SOAR has at it’s disposal. The steps for this are as follows:

1. Visit [autodrive-nas.egr.msu.edu](http://autodrive-nas.egr.msu.edu/)
2. Open the `Directory Server` program
3. Access the `Manage Users` tab on the sidebar
4. Click the `Create` button on the top menu, and fill fields below:
   1. `Name`: [msu_netid]
   2. `Description`: [firstname lastname]
   3. `Email`: [msu_netid@msu.edu]
   4. `Password`: [https://passwordsgenerator.net/]
   5. `Confirm Password`: [same-as-above]
5. Open `File Station` program
6. Click `Create > Create New Shared Folder` in the top menu:
   1. Name: [msu_netid]
   2. Uncheck “Enable Recycle Bin”
7. Right-click on the newly created folder and select `Properties`
8. Switch to the `Permission` tab and select `Create`
   1. Begin typing the user `msu_netid` (it may take a few minutes for the newly created user to propagate
      through the system - if the user doesn’t show up, wait a few minutes)
   2. Check the `Read` and `Write` boxes (all the sub-boxes should become checked)
   3. Click `Ok`
9. The user should now be created with a home directory - ensure the user changes their temporary password

## Gitlab

Users must also be added to the Gitlab, otherwise they will not be able to access the codebase and contribute.
To do this, you must have owner permissions in the SOAR Software group. If you do not have maintainer permissions,
ask an owner/admin for these credentials.
!!!danger 'Owner' role
Do _not_ give out owner or maintainer permissions to anyone but team leads.
!!!

The steps for this are as follows:

1. Go to [https://gitlab.msu.edu/groups/canvas/soar/](https://gitlab.msu.edu/groups/canvas/soar/)
2. Under `Subgroup Information` on the side bar, click `Members`
3. Select `Invite Members` and then fill out the relevant information
   1. Type in their MSU email address in the address bar
   2. Mark them as `developer`
   3. Mark a month after their planned graduation for expiration date

## Trello

Users must be added to the Trello board as well, and you can do this via the trello website. Steps are as follows:

1. Go to [https://trello.com/msusoar](https://trello.com/msusoar)
2. Click `Invite Workspace Members`
3. Input student’s MSU email address

## Google Drive

!!!info
Under construction.
!!!
