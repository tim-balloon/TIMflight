# Reimaging a Flight Computer

Making a new hard drive for a new flight computer exactly like an old one.

The easiest way to do this involves connecting the drives to a computer running Ubuntu. It can be a Live USB or a native Ubuntu install.

## What you need

1. flight computer drive (or an existing image of a flight computer, perhaps downloaded or copied from a backup)
2. new hard drive (must be equal or greater capacity than 1.)
3. SATA power cable
4. SATA data cable
5. computer with spaces to connect 3. and 4. so you can connect the hard drives, or a USB hard drive dock

## Steps

> **Caution:** Don't use `dd` if you can avoid it. It's powerful, but with no safety net. It's easy to seriously mess up a disk or OS with it if you miss a keystroke, and it's even easier to simply use the wrong settings (or defaults) and issue a command that will make this take much longer than it should, or screws up the partition table or block size of a disk image.

I used a computer with Ubuntu's "Disks" graphical utility. You won't get the 1337 h4ck3r cred of using `dd`, but it's fast and easy. You can boot into an Ubuntu Live USB on any computer to do this, or use a computer already running Ubuntu.

If you already have an image file of a flight computer disk (\*.img), skip to step 10.

1. Turn off the Ubuntu computer, if it isn't already. 
2. Connect the old flight computer hard drive to the computer power/data, or to the dock
3. Boot the Ubuntu computer into the Ubuntu OS, not the fc disk OS
4. CMD + type 'disks' + ENTER to bring up the GNOME disk utility
5. Find the old flight computer disk (128 GB Samsung 840 EVO) and click on it
6. Find the "more" button (3 vertical dots) and click on that 
7. Click "Create Disk Image..." and follow the popup dialog to select a place to save the image. 
    8. This creates an image of the entire drive in a single file. 
    9. Make sure the destination has enough storage space, as this method is "dumb" and will not compress or ignore redundant/empty regions of the disk.
10. Shut down the Ubuntu computer, unless you have a dock that supports hotswapping drives.
11. Connect the new flight computer drive to the Ubuntu computer.
12. Boot the Ubuntu computer into the Ubuntu OS
13. CMD + type 'disks' + ENTER to bring up the GNOME disk utility
14. Find the new flight computer disk and click on it.
15. Find the "more" button (3 vertical dots) and click on that 
16. Click "Format Disk..."
    17. For Erase, choose "Don't overwrite existing data"
    18. For Partitioning, choose "No partitioning," because the disk image we made has all of the data from the old drive's partition table.
    19. Format the disk. It should be almost instantaneous because we have chosen not to overwrite old data with 0s or 1s or random 1s and 0s or whatever.
20. Again, find the "more" button and click on it
21. Click "Restore Disk Image..."
    22. Locate the old flight computer's image to restore on your local filesystem (should be like "\*.img")
    23. Click "Start Restoring..." and wait for the restore to complete.
    24. If your disk capacity is greater than the old flight computer image, you will have the old flight computer OS's partitions, plus a region of "Free Space," which you may wish to use to expand one of the other partitions into.
25. Shut down the Ubuntu computer and remove the new flight computer's drive.
