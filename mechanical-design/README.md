# Mechanical design

**Owner:** _TBD — put your name here when you start_
**Status:** not started

3D models of the robots, beacons and their mounts.

## Layout

```
3D/
├── design-files/   editable CAD sources (.f3d, .step, .FCStd, .sldprt, …)
└── print-files/    print-ready exports (.stl, .3mf) and slicer output (.gcode)
```

Every printable part in `print-files/` should have its source in
`design-files/`, with the same base name, e.g. `beacon-shell.step` →
`beacon-shell.stl`.

## To fill in when you start

- CAD tool and version used
- Printer, material and slicer settings per part
- Which part fits on which robot / board (see [`docs/hardware/boards.md`](../docs/hardware/boards.md))
