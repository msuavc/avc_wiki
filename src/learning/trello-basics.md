---
icon: credit-card
order: 98
---
# Trello Basics

For AVC development, we will be using Trello to track general development progress. We recommend you use and check Trello often, so that we minimize the amount of effort needed to check in on peoples progress.

Each subteam will have a trello board tracking general development items. For example, anything related to perception on the vehicle will be on the perception board, including bugfixes, optimizations, and new features/integrations. Any task that can completed will live on its respective teams trello board. To unify the boards among teams, the general schema for the boards is below.

# Lists

There will be 4 areas in each trello board: 
- Unassigned tasks 
- Assigned tasks 
- In-development 
- Developed tasks

### Unassigned Tasks

These are tasks that are in the idea stage, and as a staging area for development items that do not have someone working on them currently. The goal is to use this list as an area to quickly mark down tasks during meetings, and to track development items before an issue is created.

Anyone in the team can and should put items into this section, but make sure they are broad enough to justify tracking its development over time.

### Assigned Tasks

These are tasks that are fully defined and fleshed out. This means that an issue has been created for them, and the respective team lead has defined exactly what needs to be accomplished in the issue description.

Before you move a task to assigned tasks, please do the following:

1. Create and fill out the issue
2. Make sure the issue has been assigned to a team member
3. Give the issue has an appropriate tag:
    1. `bug` for bugfixes
    2. `feature` for new development items
    3. `testing` for testing and verification related development
    4. `organizational` for team related tasks, such as meetings or forms

!!!info Assigned Cards
When you get assigned a card, or assign one to yourself, move it to the In-Development list!
!!!
### In-Development

These tasks are tasks that are actively being worked on, or are going to be done in the near future. If you are planning on doing something in the current week, or have a task that is next for you personally, put it here. 

### Developed

This section is for tasks that are finished. Thats kinda it.

# Cards

Cards are what makes up each list, and a card just generally defines a task. Anything you’ll do for SOAR that takes more than a minute will be a card. For example:

- A feature such as developing a new ROS node
- A bugfix such as fixing that ROS node
- A test such as testing that you’ve actually fixed the bug you created

!!!info Maintaining Trello Boards
Cards can be created by everybody, but the team lead is responsible for cleaning up the cards and maintaining general order on the Trello board. Additionally, as mentioned above, team leads will assign people and tags to each card.
!!!