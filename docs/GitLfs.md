# Version Control of Large Files with `git-lfs`

[Git Large File Storage](https://git-lfs.github.com/)

Committing large files (and multiple versions of large files) can balloon the size of a repo on disk, since the file versions are stored in blobs in the `.git` dir as history. To mitigate this issue, we use `git-lfs` with GitHub.

The basic premise is to work around `git` by storing pointers to remotely hosted versions of the files, which are automatically replaced with the real versions by `git-lfs` on checkout.

With `git-lfs`, you can commit and pull/push large files as normal.

## Q&A

### What if I don't have `git-lfs` installed?

You will check out a small file with the information needed by `git-lfs` to retrieve the file instead of the actual file.

To install `git-lfs`, instructions [here](https://docs.github.com/en/repositories/working-with-files/managing-large-files/installing-git-large-file-storage).

> Note: Debian 10, Ubuntu 18.04 and newer can just do `sudo apt-get install git-lfs && git lfs install`. Mac OSX can just do `brew install git-lfs && git lfs install` if `brew` is up to date.


<details>
<summary>
For posterity:
</summary>

1. Navigate to git-lfs.github.com and click Download.

> Tip: For more information about alternative ways to install Git LFS for Linux, see this Getting started guide.

2. On your computer, locate and unzip the downloaded file.

3. Open Terminal.

4. Change the current working directory into the folder you downloaded and unzipped: `cd ~/Downloads/git-lfs-1.X.X`

> Note: The file path you use after cd depends on your operating system, Git LFS version you downloaded, and where you saved the Git LFS download.

5. To install the file, run this command:

```
./install.sh`
Git LFS initialized.
```

> Note: You may have to use `sudo ./install.sh` to install the file.

6. Verify that the installation was successful:

```
git lfs install
Git LFS initialized.
```

</details>


You only need to do this once per machine.

### What file types are managed by `git-lfs` already?

Run `git lfs track` at the root of the repo for the most up-to-date list.

### I have a new file type I want to commit and it is big (>~few MB). What do I do?

If you haven't already, don't `git add` the new file. Make `git-lfs` track the file name/type first, then `git add` it. 

> Note: GitHub recommends that you should always run these commands from the root of the repository to ensure all files of the type are caught, no matter which directory they are in. Since we already have a ton of pdfs and images squirreled away everywhere, I recommend a more targeted approach, making and versioning a `.gitattributes` file for each subdirectory. A `.gitattributes` file at the root will catch all sorts of stuff in `external_libs`, etc. You can see this at work in `docs/`.

[More docs](https://docs.github.com/en/repositories/working-with-files/managing-large-files/configuring-git-large-file-storage).

<details>
<summary>
For posterity:
</summary>

If there are existing files in your repository that you'd like to use GitHub with, you need to first remove them from the repository and then add them to Git LFS locally. For more information, see "[Moving a file in your repository to Git LFS](https://docs.github.com/en/articles/moving-a-file-in-your-repository-to-git-large-file-storage)."

If there are referenced Git LFS files that did not upload successfully, you will receive an error message. For more information, see "[Resolving Git Large File Storage upload failures](https://docs.github.com/en/articles/resolving-git-large-file-storage-upload-failures)."

1. Open Terminal.

2. Change your current working directory to an existing repository you'd like to use with Git LFS.

3. To associate a file type in your repository with Git LFS, enter `git lfs track` followed by the name of the file extension you want to automatically upload to Git LFS.

For example, to associate a `.psd` file, enter the following command:

```
git lfs track "*.psd"`
Adding path *.psd
```

Every file type you want to associate with Git LFS will need to be added with `git lfs track`. This command amends your repository's `.gitattributes` file and associates large files with Git LFS.

> Note: We strongly suggest that you commit your local `.gitattributes` file into your repository. Relying on a global `.gitattributes` file associated with Git LFS may cause conflicts when contributing to other Git projects. Including the `.gitattributes` file in the repository allows people creating forks or fresh clones to more easily collaborate using Git LFS. Including the `.gitattributes` file in the repository allows Git LFS objects to optionally be included in ZIP file and tarball archives.

4. Add a file to the repository matching the extension you've associated:

```
git add path/to/file.psd
```

Commit the file and push it to GitHub:

```
git commit -m "add file.psd"
git push
```

You should see some diagnostic information about your file upload:

```
Sending file.psd
44.74 MB / 81.04 MB  55.21 % 14s
64.74 MB / 81.04 MB  79.21 % 3s
```

</details>

### I want to convert an already-committed file to `git-lfs` tracking. What can I do?

[More docs](https://docs.github.com/en/repositories/working-with-files/managing-large-files/moving-a-file-in-your-repository-to-git-large-file-storage).

<details>
<summary>
For posterity:
</summary>

If you've set up Git LFS, and you have an existing file in your repository that needs to be tracked in Git LFS, you need to first remove it from your repository.

After installing Git LFS and configuring Git LFS tracking, you can move files from Git's regular tracking to Git LFS. For more information, see "[Installing Git Large File Storage](https://docs.github.com/en/github/managing-large-files/installing-git-large-file-storage)" and "[Configuring Git Large File Storage](https://docs.github.com/en/github/managing-large-files/configuring-git-large-file-storage)."

If there are referenced Git LFS files that did not upload successfully, you will receive an error message. For more information, see "[Resolving Git Large File Storage upload failures](https://docs.github.com/en/articles/resolving-git-large-file-storage-upload-failures)."

> Tip: If you get an error that "this exceeds Git LFS's file size limit of 100 MB" when you try to push files to Git, you can use `git lfs migrate` instead of `filter branch` or the BFG Repo Cleaner, to move the large file to Git Large File Storage. For more information about the `git lfs migrate` command, see the [Git LFS 2.2.0](https://github.com/blog/2384-git-lfs-2-2-0-released) release announcement.

1. Remove the file from the repository's Git history using either the `filter-branch` command or BFG Repo-Cleaner. For detailed information on using these, see "[Removing sensitive data from a repository](https://docs.github.com/en/articles/removing-sensitive-data-from-a-repository)."
2. Configure tracking for your file and push it to Git LFS. For more information on this procedure, see "[Configuring Git Large File Storage](https://docs.github.com/en/articles/configuring-git-large-file-storage)."

</details>


### Are there limitations?

GitHub imposes as 2 GB per file size limit.

Unless a local `git-lfs` host has been set up, this will not work on the ice, since files must be pulled from/pushed to an Internet server. In this case, it's probably best to check out a repo to download all docs to local machines before leaving fast Internet access zones.
