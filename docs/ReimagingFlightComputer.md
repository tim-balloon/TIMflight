# Reimaging a Flight Computer

Making a new hard drive for a new flight computer exactly like an old one.

The easiest way to do this involves connecting the drives to a computer running Ubuntu, one at a time.

Alternatively, you can try a Live USB of [Clonezilla](https://clonezilla.org/liveusb.php).

## What you need

1. flight computer drive (or an existing image of a flight computer, perhaps downloaded or copied from a backup)
2. new hard drive (must be equal or greater capacity than 1.)
3. SATA power cable
4. SATA data cable
5. computer with spaces to connect 3. and 4. so you can connect the hard drives, or a USB hard drive dock

## Steps

> **Caution:** Don't use `dd` if you can avoid it. It's powerful, but with no safety net. If you don't know exactly what you're doing, at worst, a mistake can mess up a disk partition or entire OS. At best, you can choose the wrong settings (or use the defaults) and issue a command that will make this take much longer than it should.

I used a computer with Ubuntu's "Disks" graphical utility. You won't get the 1337 h4ck3r cred of using `dd`, but it's fast and easy. You can boot into an Ubuntu Live USB on any computer to do this, or use a computer already running Ubuntu.

If you already have an image file (\*.img) of a flight computer disk _that you trust_, skip to step 8. Evan Mayer created a backup of the image we received from BLAST-TNG `fc2`, and stores it in [Google Drive](https://drive.google.com/drive/folders/13XO-_MRwYzFS8gKqF0GdhsfY1ASsZHGx?usp=sharing). Request access if you need it.

### Making an Image of the "Donor" Computer

1. Connect the old flight computer hard drive to the computer power/data, or to the dock
2. Boot the Ubuntu computer into the Ubuntu OS, not the donor disk OS. If you're booting a Live USB, pick the "Try Ubuntu" option.
    1. [Mount](https://manpages.ubuntu.com/manpages/xenial/man8/mount.8.html) some storage media. This can also be done from the "Disks" GUI. We will need ~120 GB for our image of the donor flight computer disk.
3. CMD + type `disks` + ENTER to bring up the GNOME disk utility
4. Find the old flight computer disk (128 GB Samsung 840 EVO) and click on it
5. Find the "more" button (3 vertical dots) in the top right and click on that 
6. Click "Create Disk Image..." and follow the popup dialog to select a place to save the image. 
    1. This creates an image of the entire drive in a single file. It's important to do it this way to get the partition table as well as all the data.
    2. Make sure the destination storage media has enough storage space, as this method is "dumb" and will not compress or ignore redundant/empty regions of the disk.
7. Disconnect the old flight computer disk when done. It's important to do this because once the new disk is written, if you ever mount both disks to the same computer at once, you will have multiple partitions with _identical_ UUIDs, and the operating system can get confused.

### Transferring the Image to the New Drive

8. Connect the new flight computer drive to the Ubuntu computer, and do not mount it. It should show up in the Disks window.
9. Find the new flight computer disk and click on it.
10. Find the "more" button (3 vertical dots) in the top right and click on that 
11. Click "Format Disk..."
    1. For Erase, choose "Don't overwrite existing data"
    2. For Partitioning, choose "No partitioning," because the monolithic disk image we made has all of the data from the old drive's partition table.
    3. Format the disk. It should be almost instantaneous because we have chosen not to overwrite old data with 0s or 1s or random 1s and 0s or whatever.
12. Again, click the "more" button
13. Click "Restore Disk Image..."
    1. Locate in your storage media the old flight computer's image to restore with (should be like "\*.img")
    2. Click "Start Restoring..." and wait for the restore to complete.
    3. If your disk capacity is greater than the old flight computer image, you will have the old flight computer OS's partitions, plus a region of "Free Space," which you may wish to use to expand one of the other partitions into later.

> **Optional:** If you wish to make a new flight computer disk that is not necessarily a backup or clone of the original, but a unique image unto itself, then assign the cloned partitions new UUIDs: `sudo tune2fs /dev/sdXY -U random`, where `X` is the new drive's label (usually `a`, `b`, `c`, etc.), and `Y` is the label of each new partition (`1`, `2`, `3`, etc.). 

### Booting the Old Image On a New Computer

The image of Debian 8 we are propagating uses a Legacy boot mode, which is different from the more modern UEFI process for finding a bootloader, starting it up, and booting into the operating system. In order to get it to boot on a newer PC, we need to go into the PC's BIOS and change some settings.

These instructions are for American Megatrends' Aptio BIOS v2.20.1275, but should be relevant to many setups. The basic idea is to enable every Legacy boot option instead of the UEFI options.

14. Shut down the computer you're working on and install the newly cloned SSD into the flight computer.
15. Power on the new computer. Mash the `Del` key until the BIOS comes up (blue and grey screen with text).
16. `Advanced` tab (navigate with arrow keys):
    1. `CSM Configuration` (compatibility support module)
        1. `Boot option filter` > `UEFI and Legacy` - this will enable the BIOS to look for Legacy boot media in its search. You may have noticed the new SSD was not found on boot, and the PC booted directly into BIOS setup, and this is why. Choose both in case you ever want to plug in a flash drive with a Live USB image, which will likely be EFI-boot.
        2. `Network` > `Legacy`
        3. `Storage` > `Legacy`
        4. `Video` > `Legacy` - if you neglect this one, you will likely boot successfully, but be unaware of this and stare at a black screen for two days, wondering what went wrong and trying to debug every other step of the process.
        5. `Other PCI devices` > `Legacy`
    2. Hit `esc` until you reach the top level.
17. `Save & Exit` tab:
    1. `Save Changes and Reset`
18. Select the most sensible option at the GRUB menu and hit `Enter` to boot into the Debian OS.
19. Log in with the username and password.
