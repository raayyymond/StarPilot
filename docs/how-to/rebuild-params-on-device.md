# Rebuilding the params library after adding a param

StarPilot's `Dom` branch is a **prebuilt** branch: the repo root has a `prebuilt` marker file, and the
compiled artifacts (`common/params_pyx.so`, `common/libcommon.a`, `common/params_pyx.cpp`, the panda
objects, ...) are committed. The device never runs `scons` on boot.

That has one consequence for params: the list of valid keys is compiled into `common/params_pyx.so`
from `common/params_keys.h`. Adding a key to the header is not enough. Until the `.so` is rebuilt:

- Galaxy answers `Parameter 'X' is not editable.` (its allow-list comes from the compiled key set), and
- any `Params().get("X")` raises `UnknownKeyName`. `starpilot_variables` guards the custom-patch
  switches against that (it only reads a key the compiled library knows), but other new code may not.

The library must be built **on the device** (aarch64) with the AGNOS venv toolchain. The whole thing
takes about a minute.

## 1. Build on the device

```bash
ssh comma@<device-ip>          # e.g. ssh -i ~/.ssh/id_ed25519_personal comma@10.0.0.168
cd /data/openpilot
git fetch && git status         # make sure the header change you want is checked out
export PATH="/usr/local/venv/bin:$PATH"   # AGNOS venv: scons, cython, numpy, capnpc live here
scons -j4 common/params_pyx.so
```

Notes:
- Do this with the car off (`cat /data/params/d/IsOnroad` prints `0`).
- `scons` is not on the default PATH and the system `python3` has no numpy; the `PATH` export above
  is the whole trick. Do not use `uvx scons`, it will not find `capnpc`.
- Only `common/params_pyx.so`, `common/libcommon.a` and `common/params_pyx.cpp` should change.
  `scons` also touches `panda/board/obj/gitversion.h` and `panda/board/obj/version`; revert those:
  `git checkout -- panda/board/obj/gitversion.h panda/board/obj/version`.

## 2. Check the new keys resolve

```bash
PYTHONPATH=. python3 -c "from openpilot.common.params import Params; p = Params(); print(p.get_default_value(b'AccordRatePlantFF'))"
```

A wrong key raises `UnknownKeyName`; a right one prints its default.

## 3. Commit the artifacts

The device remote is HTTPS with no credentials, so copy the three files to your checkout and commit
there (this is what the historical `build` commits on this branch contain):

```bash
# on your PC
scp comma@<device-ip>:/data/openpilot/common/{params_pyx.so,libcommon.a,params_pyx.cpp} common/
git add common/params_pyx.so common/libcommon.a common/params_pyx.cpp
git commit -m "build: params library with <the new keys>"
git push origin Dom
```

## 4. Put the device on that commit and restart

```bash
# on the device
cd /data/openpilot && git pull --ff-only
sudo reboot        # controlsd, the_galaxy and the UI all import the .so at start
```

After the reboot the new toggles are editable in Galaxy and readable everywhere.

## Rebuilding from Galaxy or the on-device Software panel

The Galaxy Software page ("Rebuild Params and Reboot") and the on-device Software panel ("Rebuild
Params") run the same steps through `starpilot/common/rebuild_params.py`. Two things to know:

- **A failed rebuild restores the previous library.** The three artifacts are copied aside before
  `scons` runs. If the build fails, times out (15 minutes) or the rebuilt `.so` still does not know the
  header keys, the copies are put back (falling back to `git checkout -- <artifact>` if a copy is
  missing) and the error says so. The device keeps booting with the library it had; the new keys just
  stay unknown until the next successful rebuild. Only if the `.so` cannot be restored at all does the
  error tell you to run the `git checkout` yourself before rebooting.
- **A later fast update replaces the rebuilt `.so` with the committed one.** Fast update does
  `git reset --hard FETCH_HEAD`, so the on-device build is thrown away in favour of whatever
  `common/params_pyx.so` is at that commit. If the header changed and no `build:` commit followed it,
  rebuild again after the update (the Software page shows how many header keys the compiled library is
  missing). Committing the artifacts from the device, as in step 3 above, avoids the repeat.

## Why not just rebuild on boot?

Deleting the `prebuilt` marker makes the device run a full `scons` on every boot (many minutes on a
comma three, and it needs the whole toolchain). Committing the artifacts keeps boots fast; the price is
this manual step whenever `common/params_keys.h` (or anything under `common/`) changes.
