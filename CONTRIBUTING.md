# Developer info and tools

## USB-B connection

You can connect with the ROSbot hardware on your own computer. To establish a connection, connect your computer to the robot using a USB-B cable. Then build the code locally and specify via the serial_port argument which processor should be used to establish the connection.

```bash
ros2 launch rosbot_bringup rosbot.yaml serial_port:=/dev/rosbot
```

The hardware checks the connection via USB-B only during initialization and when btn1 or btn2 is pressed, so while executing the above command, hold down the reset button together with bnt1/bnt2 and release the reset button. After establishing a connection, you can release bnt1/bnt2.

## pre-commit

[pre-commit configuration](.pre-commit-config.yaml) prepares plenty of tests helping for developing and contributing. Usage:

```bash
# install pre-commit
pip install pre-commit

# initialize pre-commit workspace
pre-commit install

# manually run tests
pre-commit run -a
```

After initialization [pre-commit configuration](.pre-commit-config.yaml) will applied on every commit.

### Auto-fix workflow

Several hooks (`black`, `isort`, `cmake-format`, `prettier`, `trailing-whitespace`)
rewrite the file on the spot when they find something to fix. If they fire during
`git commit`, the commit aborts with `Stashed changes conflicted with hook auto-fixes`
and you have to `git add` the rewritten files and re-commit — easy to miss when
batching.

To avoid the cycle, run pre-commit on the staged files **before** `git commit`:

```bash
git add <files>
pre-commit run --files $(git diff --cached --name-only)
git add <files>   # re-stage anything the hooks rewrote
git commit -m "..."
```

Or, for a small change, just `pre-commit run -a` once before staging.
