# TODO: specify bump command and remove this description

Bump version workflow is triggered when PR is closed, it bumps version and triggers docker build that will be tagged with version number. To use it you have to specify bump type, available commands: `bump::patch`, `bump::minor` and `bump::major` (simply leave one of them in PR description and remove this description).
