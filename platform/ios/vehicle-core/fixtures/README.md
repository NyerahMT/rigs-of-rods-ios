# Authored vehicle fixtures

These files are complete upstream Rigs of Rods vehicle definitions used to test the portable iOS actor-loading path. They are fixtures, not simplified demo rigs.

## DAF Semi

- File: `dafsemi/b6b0UID-semi.truck`
- Upstream repository: `RigsOfRods/content`
- Upstream content revision pinned by `RigsOfRods/rigs-of-rods` master during this port: `34fefdd126784bf87b068fc283f812525d159dd7`
- Upstream blob: `c68ccda7a6553ecc1fa5ab34f4bc80bc710ebfc2`
- Vehicle authors recorded in the source file: Pricorde; G.Rutka & Pricorde

The fixture is intentionally kept whole, including currently unsupported visual/interaction sections. `PortableRigDef` skips unsupported blocks while the iOS port grows compatibility rather than maintaining a stripped mobile-only vehicle format.
