"""Provision separate random motor controller/operator keys on the host.

One-time manual operation, never invoked by normal ROS launch. Creates
only two new 0600 regular files in an existing private directory.
Never prints, overwrites, commits or transmits the secrets.
"""
import argparse
import os
from pathlib import Path
import secrets
import stat


def provision(directory):
    root = Path(directory)
    if (not root.is_absolute() or root.is_symlink() or
            not root.is_dir()):
        raise ValueError("existing absolute, non-symlink directory required")
    st = root.stat()
    if (st.st_uid != os.geteuid()
            or st.st_mode & 0o077
            or not stat.S_ISDIR(st.st_mode)):
        raise ValueError("key directory must be private and process-owned")
    names = ("leroy_motion.key", "leroy_operator.key")
    targets = [root / name for name in names]
    if any(path.exists() or path.is_symlink() for path in targets):
        raise FileExistsError("motor keys already present; refusing overwrite")
    created = []
    try:
        for path in targets:
            fd = os.open(
                path, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
            created.append(path)
            try:
                data = secrets.token_bytes(32)
                if os.write(fd, data) != len(data):
                    raise OSError("short key write")
                os.fsync(fd)
            finally:
                os.close(fd)
    except Exception:
        for path in created:
            path.unlink(missing_ok=True)
        raise
    return tuple(str(path) for path in targets)


def main():
    parser = argparse.ArgumentParser(
        description="Provision protected motor role keys; does not arm motors")
    parser.add_argument("--directory", required=True)
    args = parser.parse_args()
    paths = provision(args.directory)
    print("Created distinct protected motor role files:")
    for path in paths:
        print(path)
    print("No motor process launched; keep the files outside Git and logs")


if __name__ == "__main__":
    main()
