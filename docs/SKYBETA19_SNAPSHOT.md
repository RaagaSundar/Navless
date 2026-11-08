# Skybeta-19 project copy

Copied from `skybeta-19` over Ethernet on November 7, 2026, starting at
19:02 Pacific (03:02 UTC on November 8). Paths under `home/` and `opt/`
in the local snapshot `skybeta19_snapshots/20261108T030223Z/` match the Jetson. This is a file copy from the running host.

The copy includes the Skybeta and Navless checkouts, team workspace, flight
tests and backups, scan and as-built project directories, bootstrap scripts,
localizer GIF, and installed Skybeta status project. It preserves project
dotfiles, Git directories, and symbolic links. Absolute links retain their
Jetson targets. The project scope excludes personal account settings, host
configuration under `/etc`, and container filesystems.

Verified 2,255 regular files (794,803,640 bytes) and 20 symbolic links.
An rsync checksum comparison found no differences for the copied paths.
Two root-owned files in `/home/dronehacks/team_ws/split_sim_state/` could
not be read by the SSH account and are missing from this copy:

- `20261107T224935Z-split-astar-flight-retained-registration.json`
- `20261107T230718Z-split-astar-flight-retained-registration.json`
