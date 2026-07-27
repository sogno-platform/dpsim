# DPsim agent instructions

## Environment

- This repository is built in Ubuntu under WSL2.
- Do not use sudo.
- Use Unix Makefiles.
- Use at most two parallel compiler jobs.
- Preserve the existing build directory unless reconfiguration is necessary.
- Do not modify Git history.
- Never use destructive Git commands.
- Do not commit or push without an explicit user request.

## Scope

The current development objective is to add a physically correct scalar DC
domain and State-Space Nodal DC components for use inside EMT simulations.

The first milestone is standalone DC-network infrastructure and passive DC
components. AC/DC converters and MMC integration are later milestones.

## Modelling rules

- Do not use EMT_Ph1 merely as a semantic substitute for DC.
- State voltage polarity and current orientation explicitly.
- State whether quantities are scalar, phase values, RMS, peak or dq values.
- Separate nodal unknowns, dynamic states, history terms and controller states.
- Preserve consistent passive-sign-convention power.
- Derive every SSN stamp and history source.
- Do not introduce arbitrary damping to conceal instability.
- Keep initialization explicit and reproducible.

## Validation

- Build affected targets with `--parallel 2`.
- Run the smallest relevant test first.
- Check all outputs for NaN, Inf and divergence.
- Validate RC and RL transients against analytical solutions.
- Review the complete diff before declaring completion.
