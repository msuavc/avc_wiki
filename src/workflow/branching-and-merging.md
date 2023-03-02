---
icon: git-branch
order: 99
---

# Branching

## Overview

!!!warning Before You Read
Before you change anything in the vehicle codebase, it is important that you do
one thing: **branch**. I'd recommend that you consult the [Git Basics](~/basics/git-basics)
section of the wiki for a git tutorial/refresher if needed.
!!!

What branching ensures in our system is compartmentalization of changes across
multiple developers. In our vehicle codebase, there is a lot going on, and multiple
ROS nodes depend on multiple other nodes in order to function.

Say there is work required on a node called `object_detection`. Our work with `object_avoidance` may depend on the earlier version of `object_detection`. If
`object_detection` is not fully functioning during its retooling, `object_avoidance`
is not going to be developed further until that is done, and we can't have that.

**In comes branches. Branches separate changes so that you can't break everyones code by yourself, only yours.**

To manage all this, we will be using the [Git Feature Branch Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

### Creating Branches

!!!info Branching Responsibilities
Feel free to create your own branch
by following these instructions, or ask a lead to do it for you
!!!

1. **Create a new branch off of `dev`** that describes your software. Should be
   prepended with "feature-" (i.e., "feature-2d-sign-detector"), and should be named
   in a similar fashion to the milestone for clarity. You can create a new
   branch by either:

   - a. On the main project page, next to where it displays the current branch in a dropdown, press the "+" icon,
     then select "New branch." On the next page, **make sure you select `dev` for "create from"**

   - b. With a local checkout of the repository, first make sure you are on a recent version of the dev branch:

     ```bash
     git checkout dev; git pull
     ```

     Then create a new branch:

     ```bash
     git checkout -b feature-your-feature-name-here
     ```

2. **Immediately create a merge request** to merge this feature branch back into
   dev. **Make sure to prepend "WIP:"** to the issue title so that it's known that the
   software is not yet done

## For Active Developers

1. **Checkout Your Branch** Run `git checkout {feature-branch}` on the branch associated with your milestone.

2. **Develop software on your branch.** Commit early and often (See [Git Basics](~/basics/git-basics). Run automated testing pipelines if available.

3. **Once software is ready to merge, remove WIP tag.** Notify team lead that
   software is ready to merge (once automatic testing is enabled, must wait until
   automatic testing passes).
