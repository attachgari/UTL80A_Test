"""Set the built-in Arduino IDE 2 Serial Plotter to 2,000 points on macOS."""

import argparse
from hashlib import sha256
import os
from pathlib import Path
import stat
from tempfile import NamedTemporaryFile


ASSET = "main.35ae02cb.chunk.js"
RELATIVE = Path("Contents/Resources/app/lib/backend/resources/arduino-serial-plotter-webapp/static/js") / ASSET
OLD = b"U=Object(o.useState)(50),P=Object(c.a)(U,1)[0]"
NEW = b"U=Object(o.useState)(2000),P=Object(c.a)(U,1)[0]"


def update_plotter(bundle, action, backup_dir):
    asset = bundle / RELATIVE
    if bundle.suffix != ".app" or not asset.is_file():
        raise ValueError(f"Supported Arduino Plotter asset not found: {asset}")
    if asset.is_symlink():
        raise ValueError("The Plotter asset is a symbolic link.")
    active = asset.read_bytes()
    if active.count(OLD) == 1 and NEW not in active:
        original, setting = active, 50
    elif active.count(NEW) == 1 and OLD not in active:
        original, setting = active.replace(NEW, OLD, 1), 2000
    else:
        raise ValueError("Unrecognized Plotter bundle. No changes were made.")
    digest = sha256(original).hexdigest()
    backup = backup_dir / f"{ASSET}.{digest}.backup"

    if action == "check":
        return setting
    if action == "apply" and setting == 50:
        if not backup.exists():
            with backup.open("xb") as saved:
                saved.write(original)
        if backup.read_bytes() != original:
            raise ValueError("The existing backup differs from the original asset.")
        content, result = original.replace(OLD, NEW, 1), 2000
    elif action == "restore" and setting == 2000:
        if not backup.is_file() or backup.read_bytes() != original:
            raise ValueError(f"The matching original backup is missing or differs: {backup}")
        content, result = original, 50
    else:
        return setting

    # Stage beside the asset so a failed write leaves the installed file intact.
    staged = None
    try:
        with NamedTemporaryFile(dir=asset.parent, delete=False) as f:
            staged = Path(f.name)
            f.write(content)
        os.chmod(staged, stat.S_IMODE(asset.stat().st_mode))
        os.replace(staged, asset)
    finally:
        if staged is not None and staged.exists():
            staged.unlink()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("check", "apply", "restore"))
    parser.add_argument("--app", type=Path, default=Path("/Applications/Arduino IDE.app"))
    args = parser.parse_args()
    try:
        size = update_plotter(args.app.expanduser(), args.action, Path(__file__).resolve().parent)
    except (OSError, ValueError) as error:
        parser.exit(1, f"{error}\n")
    print(f"Arduino Serial Plotter window: {size:,} samples")


if __name__ == "__main__":
    main()
