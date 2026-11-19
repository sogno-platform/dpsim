# Deprecated PCM synchronous generator models

This directory preserves the source for the following retired models:

- `DP::Ph1::SynchronGenerator4OrderPCM`
- `DP::Ph1::SynchronGenerator6OrderPCM`
- `EMT::Ph3::SynchronGenerator4OrderPCM`

The archived sources are excluded from the production library and are not
installed or exposed through the Python bindings. Their former example programs
and the PCM/VBR comparison notebook are kept under `examples/deprecated/` for
historical reference. These archived files are not part of the build and may
require adaptation to compile against current DPsim APIs.

For supported reduced-order synchronous generator models, use the corresponding
VBR models. The DP 4th-order trapezoidal model remains available as
`DP::Ph1::SynchronGenerator4OrderTPM`.
