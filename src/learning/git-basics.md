---
icon: git-commit
order: 99
---

# Git Basics

MSU SOAR uses git for version control. If you are not familiar with git, I’d recommend that you read the guides below to get background information:

- [What Is Version Control?](https://www.atlassian.com/git/tutorials/what-is-version-control)
- [What Is Git?](https://www.atlassian.com/git/tutorials/what-is-git)

We specifically use MSU’s gitlab service, and you can access the AVC group here:
[AVC GitLab](https://gitlab.msu.edu/canvas/avc)

## Branches

We will be using multiple git branches to coordinate development and isolate changes from each other. The main branch of the repository will be the branch `dev`. To get a local copy of the repository on your system, you will first execute the commands:

```bash
# this downloads the git repo, the repo url can be found on the gitlab
git clone --recurse-submodules {repo url}

# this switches your branch to the dev branch
git checkout dev
```

This will result the git repo being synced with the latest changes to our development branch. You will **NOT** be using this branch for your development purposes, however, and all changes to the dev branch will be made via pull requests.

When you are working on anything, whether it is a bugfix, feature, or testing code, you will create a new branch specifically for that purpose using the commands below.

```python
# this command will create the branch needed
# when naming this branch, use snake case, and be as descriptive
# as you can be without being wordy
git branch {branch name}

# this command will switch you to the new branch you just created
git checkout {branch name}

# additionally, local branches can be seen with the command
git branch
```

## Making Changes to a Branch

[Using Branches](https://www.atlassian.com/git/tutorials/using-branches)

When you are actively developing on the car, you will want to commit and push working code to the remote copy of the repo (the gitlab copy) in order to track the changes you’ve made.

---

### `git fetch`

[The Fetch Command](https://www.atlassian.com/git/tutorials/syncing/git-fetch)

The first command we will discuss is git fetch. Run this command around once a week to ensure that your local repository is tracking the remote repository

```bash
# run this to download all changes in the remote repo
git fetch remote
```

---

### `git pull`

[The Pull Command](https://www.atlassian.com/git/tutorials/syncing/git-pull)

This command will be run before you work every single time. This will ensure that your local branch is caught up with the remote repository, and is especially important if you are collaborating on a codebase change. You may have to resolve a merge conflict if you are working in the same areas of code, so consult the other author if need be.

```bash
# run this command to pull the remote's copy of the current branch
git pull remote

# if you want to discard the changes on your local branch, and use the files
# in the remote repo, run this
# WARNING: your changes will be discarded if they have not been committed,
# do not run this command by default
git pull --rebase remote
```

---

### `git add`

[The Add Command](https://www.atlassian.com/git/tutorials/saving-changes)

This command is how you actually tell git that you want to track changes made to any files. You must do this in order to push code to the repo. You will add individual files and directories by yourself, and it is important to only add files that are relevant to the vehicles functionality and operation. For example, if you have developed other scripts to aid in your initial development progress, and they do not need to be run for your software to work in production, do not add those scripts.

```bash
# this command will show "tracked" and "untracked" changes to your repo
# use this to determine what needs to be added to the current commit
git status

# this command will add a file
git add {filepath}

# this command will add all files in a directory
git add {directory/}

# this command will let you interactively add each portion you have changed
# check the atlassian link above for keyboard commands
git add -p
```

---

### `git commit`

[The Commit Command](https://www.atlassian.com/git/tutorials/saving-changes/git-commit)

This is how you actually save your changes to the branch. This will create a commit hash that saves that exact point in time, tracking all changes you’ve added to the branch. Additionally, unlike git add, git commit allows you to add a message detailing your changes concisely.

When you add and commit changes, please try to make your changes relatively minimal. Do not develop an entire new section of the codebase, then add files. Rather, you should add and commit in small steps, such as creating the file structure of your software, then adding and committing. This will allow you to easily rollback changes if you have made a mistake, and track where bugs were introduced more effectively.

```bash
# this command commits all changes you've added to your local branch
# and specifies the commit message
git commit -m "{message}"

# this command will open up a text editor to type a longer commit message,
# allowing for you to be more specific about your changes if the short message
# is not satisfactory
# when committing like this, follow this schema
# {short higher level message}
# Changes
# - {change 1}
# - {change 2}
# - {change 3}
git commit

```

---

### `git push`

[The Push Command](https://www.atlassian.com/git/tutorials/syncing/git-push)

Okay awesome! Now you’ve committed your changes to the local repo. You are able to track them and you can see a nice commit history detailing all the changes you’ve made. However, if you look at the gitlab, you will not see any of your changes there. This is because running `git commit` only commits changes locally. To have the remote repo track your changes, you need to use the `git push` command.

```bash
# this command will push your code to the remote copy of the branch you are
# currently on
git push -u origin {branch name}
```

---

### `git stash`

[The Stash Command](https://www.atlassian.com/git/tutorials/saving-changes/git-stash)

The final git command we will discuss is stashing changes. Stashing changes tells git to put them aside for later, which allows you to work on something else. Additionally, you may need to stash your changes if you have untracked work done on your branch and need to pull the latest changes from remote. You can then apply your stash and have your changes be merged with the most up to date version of your branch without having to mess with adding or committing.

```bash
# this command will stash your changes for later
git stash

# this command will apply your stashed changes when it is later
git stash apply
```

---

## Making a Merge Request

The final git basic we will discuss is making merge requests. Merge requests are needed to ensure that multiple sets of eyes will see what changes you have made to the codebase, and serve as a barrier allowing team and program leads to ensure that the changes look good. Merge requests are to be done when a feature is complete, tested, and ready to be run on the vehicle, or when a bugfix has been completed. Below is a link specifying how to actually create a merge request using GitLab, and its recommended that you follow these steps to create one:

[Creating Merge Requests](https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html)
