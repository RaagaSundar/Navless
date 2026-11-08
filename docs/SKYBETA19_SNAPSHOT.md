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

## Source in this repository

The [source snapshot](../.snapshots/skybeta-19/20261108T030223Z/) contains 651 files from the
Jetson, with their original paths relative to `/`. The organiser checkout was
Skybeta `0.3.3` at `3c42933f5adc7b22997614946f6719b2c2f89f20`.
The copy preserves the deployed source and instructions from the different Navless
checkouts, team workspaces, and Skybeta installation. Repeated run-staging copies
of identical source are omitted.

This is a reference for reviewing the deployed code. Generated models, build and
install trees, firmware binaries, maps, run outputs, credentials, and nested Git
histories are outside the source import. The full local snapshot described above
also holds the generated artifacts.

The public copies of the Skybeta stack and fleet guides omit another aircraft’s Tailscale address and Jetson serial number.
