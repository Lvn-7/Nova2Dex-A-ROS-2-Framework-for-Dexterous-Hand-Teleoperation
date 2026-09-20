# SenseGlove SDK placement

The pinned SenseGlove SDK files used by the Windows reader are included in this
directory.

Place the vendor files in this directory so the existing reader CMake project
can find:

```text
sdk/
├── include/SenseGlove/...
└── lib/win64/msvc143/release/
    ├── sgcore.dll
    ├── sgcore.lib
    ├── sgconnect.dll
    └── sgconnect.lib
```

These files remain subject to the SenseGlove/Adjuvo license terms. Confirm
public redistribution permission before publishing or mirroring the repository.
