# Reimaging a Flight Computer

Making a new hard drive for a new flight computer exactly like an old one.

The easiest way to do this involves connecting the drives to a computer running Ubuntu. The easiest way to do this is to make and use a [Live USB](https://ubuntu.com/tutorials/try-ubuntu-before-you-install#1-getting-started), because we will need one after we finish the initial reimaging.

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

If you already have an image file (\*.img) of a flight computer disk _that you trust_, skip to step 8.

### Making an Image of the "Donor" Computer

1. Connect the old flight computer hard drive to the computer power/data, or to the dock
2. Boot the Ubuntu computer into the Ubuntu OS, not the donor disk OS. If you're booting a Live USB, pick the "Try Ubuntu" option.
    1. [Mount](https://manpages.ubuntu.com/manpages/xenial/man8/mount.8.html) some storage media. We will need ~120 GB for our image of the donor flight computer disk.
3. CMD + type 'disks' + ENTER to bring up the GNOME disk utility
4. Find the old flight computer disk (128 GB Samsung 840 EVO) and click on it
5. Find the "more" button (3 vertical dots) in the top right and click on that 
6. Click "Create Disk Image..." and follow the popup dialog to select a place to save the image. 
    1. This creates an image of the entire drive in a single file. 
    2. Make sure the destination storage media has enough storage space, as this method is "dumb" and will not compress or ignore redundant/empty regions of the disk.
7. Disconnect the old flight computer disk when done.

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

### Making sure GRUB Works Correctly

GRUB, the GNU bootloader, appears to need reinstalling when disks are reimaged this way. I don't understand every way this can go wrong, and specifically how to fix them all, but thankfully there is a shotgun approach that should work.

14. Shut down the computer you're working on and install the newly flashed SSD into the flight computer.
    1. If for some reason you're following these instructions on a non-flight computer (like I did while writing them), I would also unplug all of my other precious, precious drives, to make absolutely certain they could not be touched by `boot-repair`. Safety third!
15. Plug the Live USB into the USB port of the flight computer.
16. Boot the computer into the Live USB's OS. Choose the "Try Ubuntu" option.
17. Open the GNOME "Disks" utility to make sure the new disk is recognized. Close it.
18. From the command line, install and run the [`boot-repair`](https://askubuntu.com/a/182863) utility:

`sudo add-apt-repository ppa:yannubuntu/boot-repair && sudo apt-get update`

`sudo apt-get install -y boot-repair && boot-repair`

19. Use "Recommended Repair." Since the flight computer's new disk is the only other disk installed, the recommended repair will only make changes to that disk.
    1. Why does this work? To the best of my knowledge, doing this ensures that GRUB can locate all the bootable OSs on the new disk, and that the GRUB configuration is correct.
20. Shut down the flight computer and unplug the Live USB.
21. Boot the flight computer into the new disk's OS.
