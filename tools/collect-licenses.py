"""Bundle dependency license texts from Cargo's already-resolved local sources.

This preserves upstream notices; the inventory is not a legal compatibility audit.
No dependency code is executed and nothing is downloaded by this script.
"""
import json
import pathlib
import shutil
import sys

metadata_path = pathlib.Path(sys.argv[1]).resolve()
destination = pathlib.Path(sys.argv[2]).resolve()
metadata = json.loads(metadata_path.read_text(encoding="utf-8-sig"))
members = set(metadata["workspace_members"])
destination.mkdir(parents=True, exist_ok=True)
inventory = []
for package in metadata["packages"]:
    if package["id"] in members:
        continue
    source = pathlib.Path(package["manifest_path"]).parent
    folder = destination / f"{package['name']}-{package['version']}"
    paths = set()
    for candidate in source.rglob("*"):
        if candidate.is_file() and candidate.name.upper().startswith(("LICENSE", "LICENCE", "COPYING", "NOTICE")):
            paths.add(candidate)
    if package.get("license_file"):
        candidate = source / package["license_file"]
        if candidate.is_file():
            paths.add(candidate)
    copied = []
    for path in sorted(paths):
        relative = path.relative_to(source)
        target = folder / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
        copied.append(relative.as_posix())
    inventory.append({"name": package["name"], "version": package["version"],
                      "license_expression": package.get("license"),
                      "repository": package.get("repository"), "notices": copied})
(destination / "inventory.json").write_text(json.dumps(inventory, indent=2), encoding="utf-8")
print(f"Preserved license inventory for {len(inventory)} resolved packages.")
