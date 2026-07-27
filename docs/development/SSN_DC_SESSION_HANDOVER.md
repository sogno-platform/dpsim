# SSN DC Development Handover

## 1. Objective

Long-term goal: add a physically correct scalar DC domain and State-Space Nodal (SSN) DC components usable inside EMT simulations, while retaining a path to mixed `EMT::Ph3` AC/DC multiport converters. The approved current milestone is standalone scalar DC-network infrastructure and passive DC components; AC/DC converters and MMC integration are explicitly later work.

## 2. Current repository state

- Branch: `feature/emt-ssn-dc-components`
- Commit: `04a9fe846880b4c7cd1cdbc1508a076fd72a7014`
- Working tree: not clean; implementation is uncommitted and must be treated as incomplete pending review/integration.
- Modified: `dpsim-models/include/dpsim-models/{Components.h,Definitions.h}`, `dpsim-models/src/{CMakeLists.txt,SimNode.cpp,SimPowerComp.cpp,TopologicalNode.cpp,TopologicalTerminal.cpp}`, `dpsim-models/src/EMT/{EMT_SSNComp.cpp,EMT_VTypeSSNComp.cpp}`, `dpsim/examples/cxx/CMakeLists.txt`, `dpsim/src/{MNASolver.cpp,pybind/main.cpp}`.
- Untracked implementation: `dpsim-models/include/dpsim-models/Base/Base_DC_{Resistor,Capacitor,Inductor,PiLine}.h`; `dpsim-models/include/dpsim-models/EMT/EMT_DC_{TwoTerminalVTypeSSNComp,TwoTerminalITypeSSNComp,SSN_Resistor,SSN_Capacitor,SSN_Inductor,SSN_PiLine,VoltageSource,CurrentSource}.h`; matching eight `.cpp` files under `dpsim-models/src/EMT/`; `dpsim/examples/cxx/Circuits/EMT_DC_SSN_Validation.cpp`.
- Other untracked: `.codex/`, `AGENTS.md`, and this report. These are not product implementation.
- `git diff --stat` for tracked files: 12 files, 81 insertions, 15 deletions; untracked files are excluded from that statistic.

## 3. Architecture decisions

- Scalar DC nodes/terminals use `PhaseType::DC`, one real nodal voltage, and slot 0 of the existing three-slot node-index storage. Ground or `PhaseType::DC` is required at every DC-component terminal; `EMT_Ph1` is not reused.
- Passive SSN parents: resistor/inductor derive through `EMT::DC::TwoTerminalVTypeSSNComp` → `EMT::VTypeSSNComp`; capacitor derives through `EMT::DC::TwoTerminalITypeSSNComp` → `EMT::ITypeSSNComp`. The pi-line is a `CompositePowerComp<Real>`. Ideal sources derive directly from `MNASimPowerComp<Real>`.
- Existing EMT MNA/SSN scheduling and KLU sparse solve are reused. V-type ports stamp companion conductance `W`; I-type ports invert scalar companion impedance `W` and stamp `1/W`. Sources use standard MNA RHS/branch-unknown stamps.
- Nodal unknowns are scalar DC node voltages; an ideal voltage source adds one MNA branch-current unknown. Dynamic component states are capacitor voltage and inductor current. History terms are RHS/companion terms, not nodal or controller states. No controller states exist in this milestone.
- For terminals `(0,1)`, `v = v1-v0`, positive `i` flows from terminal 1 to terminal 0, and `p=vi` is passive-sign-convention power. Positive load power is absorbed; source power is negative when delivering.
- Initialization is explicit: DC node initial voltage is copied as a real scalar (no RMS-to-peak conversion); capacitor state comes from initial terminal voltage; inductor state comes from `i_init`; pi-line sets its virtual-node voltage consistently with series `R*i_init`; unset/non-finite/invalid parameters and non-real DC initial voltages are rejected.
- Integration is trapezoidal through `Math::calculateStateSpaceTrapezoidalMatrices`: `x_k=dA x_{k-1}+dB(u_k+u_{k-1})`, `W=C dB+D`, and `y_k=Wu_k+y_hist`, where `y_hist=C dA x_{k-1}+C dB u_{k-1}` as represented by `calculateHistoryVector()`.
- Future mixed `EMT::Ph3` AC/DC multiport support: retaining the three-slot matrix-index storage and assigning DC to slot 0 avoids collapsing DC into `Ph1`. **Unresolved:** the per-port phase/domain metadata and general mixed-port base-class API for a single converter component were not designed in this milestone.

## 4. Relevant existing DPsim classes

- `CPS::EMT::SSNComp` — `dpsim-models/include/dpsim-models/EMT/EMT_SSNComp.h`, `dpsim-models/src/EMT/EMT_SSNComp.cpp`: continuous/discrete state-space model, history vector, scheduling, and finite-value guards.
- `CPS::EMT::VTypeSSNComp` — `dpsim-models/include/dpsim-models/EMT/EMT_VTypeSSNComp.h`, `dpsim-models/src/EMT/EMT_VTypeSSNComp.cpp`: voltage-input/current-output SSN companion path used by DC resistor and inductor.
- `CPS::EMT::ITypeSSNComp` — `dpsim-models/include/dpsim-models/EMT/EMT_ITypeSSNComp.h`, `dpsim-models/src/EMT/EMT_ITypeSSNComp.cpp`: current-input/voltage-output SSN path adapted by the DC capacitor.
- `CPS::MNASimPowerComp<Real>` — `dpsim-models/include/dpsim-models/MNASimPowerComp.h`: MNA component interface and public scalar `intfVoltage`/`intfCurrent` attributes.
- `CPS::CompositePowerComp<Real>` — `dpsim-models/include/dpsim-models/CompositePowerComp.h`: owns and schedules pi-line subcomponents.
- `CPS::SimNode<Real>` / `CPS::EMT::SimNode` — `dpsim-models/include/dpsim-models/SimNode.h`, `dpsim-models/src/SimNode.cpp`: scalar DC voltage initialization and matrix-index access.
- `CPS::SimPowerComp<Real>` — `dpsim-models/include/dpsim-models/SimPowerComp.h`, `dpsim-models/src/SimPowerComp.cpp`: terminal-to-matrix index mapping, now DC-aware.
- `CPS::TopologicalNode` and `CPS::TopologicalTerminal` — corresponding headers and `dpsim-models/src/Topological{Node,Terminal}.cpp`: scalar initial-voltage propagation for DC.
- `DPsim::MnaSolver<Real>` — `dpsim/include/dpsim/MNASolver.h`, `dpsim/src/MNASolver.cpp`: assigns one unknown to a DC node and three only to `ABC`.
- `CPS::MNAStampUtils` — `dpsim-models/include/dpsim-models/MNAStampUtils.h`: conductance stamping reused by scalar DC companions.

## 5. Implemented work

- `EMT::DC::TwoTerminalVTypeSSNComp` — header/source `dpsim-models/include/dpsim-models/EMT/EMT_DC_TwoTerminalVTypeSSNComp.h` and `dpsim-models/src/EMT/EMT_DC_TwoTerminalVTypeSSNComp.cpp`. Protected `(uid,name,logLevel)` constructor and initial-input builder; public final MNA stamp/update overrides. One voltage input and one current output; no added state. Stamps `W` between two scalar nodes and history-current RHS. Initializes input from real `v1-v0`. Logs inherited `intfVoltage`, `intfCurrent`, SSN state/history attributes.
- `EMT::DC::TwoTerminalITypeSSNComp` — analogous header/source. Public final stamp/current/voltage overrides and protected companion-conductance helper. One current input and one voltage output; no added state. Stamps `G=1/W`, converts voltage history to Norton current, and initializes dynamic input to zero. Logging is inherited.
- `EMT::DC::SSN::Resistor` — `.../EMT_DC_SSN_Resistor.{h,cpp}`. API: constructors, `clone`, `setParameters(R)`. No state; `A/B/C` empty and `D=1/R`, so `i=v/R`. Attribute: `R`, plus inherited interface/SSN attributes. No custom initialization override.
- `EMT::DC::SSN::Capacitor` — `.../EMT_DC_SSN_Capacitor.{h,cpp}`. API: constructors, `clone`, `setParameters(C)`, initialization override. State `x=v_C`; `A=0,B=1/C,C_ss=1,D=0`; I-type companion stamp. Initializes `x` and interface voltage from `v1-v0`, current to zero. Attribute: `C`, inherited `x`, voltage/current/history.
- `EMT::DC::SSN::Inductor` — `.../EMT_DC_SSN_Inductor.{h,cpp}`. API: constructors, `clone`, `setParameters(L,i_init=0)`, initialization override. State `x=i_L`; `A=0,B=1/L,C_ss=1,D=0`; V-type companion stamp. Initializes `x`/current to `i_init` and voltage from nodes. Attributes: `L`, `i_init`, inherited state/interface/history.
- `EMT::DC::SSN::PiLine` — `.../EMT_DC_SSN_PiLine.{h,cpp}`. API: constructors, `clone`, `setParameters(R_series,L_series,C_parallel=0,G_parallel=0,i_init=0)`, composite creation/initialization and parent task overrides. Composite states are the series-inductor current and optional two shunt-capacitor voltages. It creates series R-L through one DC virtual node and splits total shunt `C` and `G` equally to ground at both ends. Initializes public `v=v1-v0`, current `i_init`, and virtual voltage `v0+R_series*i_init`. Attributes: `R_series,L_series,C_parallel,G_parallel,i_init`, public interface attributes, and subcomponent attributes.
- `EMT::DC::VoltageSource` — `.../EMT_DC_VoltageSource.{h,cpp}`. API: constructors, `clone`, `setParameters(V)`, initialization and MNA/task overrides. No dynamic state; adds one branch-current unknown, incidence stamp, and voltage RHS. Orientation is `v1-v0=V_ref`; initializes voltage and reconstructs branch current after solve. Attribute: dynamic `V_ref`, interface voltage/current.
- `EMT::DC::CurrentSource` — `.../EMT_DC_CurrentSource.{h,cpp}`. API mirrors voltage source with `setParameters(I)`. No state or system-matrix stamp; RHS injects `+I` at terminal 0 and `-I` at terminal 1, then reconstructs `v1-v0`. Attribute: dynamic `I_ref`, interface voltage/current.
- Infrastructure: added `PhaseType::DC`, scalar DC initialization/indexing, DC pybind enum export, component aggregation/CMake sources, and finite-value/time-step checks in the generic SSN path.
- Base parameter holders: `Base::DC::{Resistor,Capacitor,Inductor,PiLine}` in `dpsim-models/include/dpsim-models/Base/` create the attributes listed above.

## 6. Equations and conventions

- `v = v_1-v_0`; `i>0` flows `1→0`; `p=vi` (PSC).
- Resistor: `v=Ri`, equivalently `i=v/R`.
- Capacitor: `i=C dv/dt`; state `x=v`, `dx/dt=i/C`. Trapezoidal: `v_k=v_{k-1}+(Δt/2C)(i_k+i_{k-1})`; I-type output form `v_k=Z_C i_k+h_C`, `Z_C=Δt/(2C)`, `h_C=v_{k-1}+Z_C i_{k-1}`. Norton stamping uses `G_C=1/Z_C=2C/Δt` and history injection derived from `h_C`.
- Inductor: `v=L di/dt`; state `x=i`, `di/dt=v/L`. Trapezoidal: `i_k=i_{k-1}+(Δt/2L)(v_k+v_{k-1})`; V-type output form `i_k=G_L v_k+h_L`, `G_L=Δt/(2L)`, `h_L=i_{k-1}+G_L v_{k-1}`.
- Voltage source enforces `v_1-v_0=V_ref`; its MNA current unknown is positive `1→0`. Current source defines `I_ref` positive `1→0` and stamps the corresponding nodal injections.
- Pi-line arguments are total lumped `R_series`, `L_series`, `C_parallel`, and `G_parallel`; the series R and L are not split, while each end receives `C_parallel/2` and `G_parallel/2` to ground.

## 7. Build configuration

- Directory/generator/type: `build/`, Unix Makefiles, `RelWithDebInfo`, static libraries.
- Enabled: `WITH_GPL=ON`, `WITH_LGPL=ON`, `WITH_JSON=ON`, `WITH_KLU=ON`, `WITH_OPENMP=ON`, `WITH_SPARSE=ON`.
- Disabled: ASAN, TSAN, LTO, native-arch tuning, profiling, RT, MNASolver plugin; CIM, Graphviz, GSL, pybind, Sundials, and Villas resolve off.
- Dependencies: system Eigen/filesystem/pybind/spdlog (`FETCH_*=OFF`); fetched JSON, ReaderWriterQueue, and SuiteSparse; KLU is the exercised solver.
- Maximum permitted parallelism: 2.
- Successful command verified in this handover: `cmake --build build --target EMT_DC_SSN_Validation --parallel 2`.
- The preserved cache proves the configuration but not the exact original configure command; do not invent it.
- No failed build command/cause is recoverable from the available session record. Current focused build has no compile error.

## 8. Tests and results

Target/executable: `EMT_DC_SSN_Validation`; command: `./build/dpsim/examples/cxx/EMT_DC_SSN_Validation`; result: pass, “All scalar DC SSN validations passed.”

- Invalid parameters: zero R/L, near-zero C/pi shunt C, and infinite source voltage all rejected.
- Resistor/source: expected and simulated `3 A` (and `-3 A` reversed), absolute error `0`; voltage-source balance `0 W`; current-source/load expected/simulated `2 A`, error `0`; source powers `-36 W`, `-36 W`, `-40 W`.
- RC (`V=10,R=2,C=.01,τ=.02,Δt=τ/1000`): at nominal `τ`, analytical `6.32121 V`, simulated `6.31937 V`, absolute error `1.84001e-3`; with documented half-step startup alignment, analytical/simulated `6.31937 V`, error `1.53513e-7`. At `10τ`, analytical/simulated `9.99955 V`, error `2.26735e-7`.
- RL (`V=10,R=2,L=.02,τ=.01,Δt=τ/1000`): at nominal `τ`, analytical `3.16060 A`, simulated `3.15968 A`, error `9.20005e-4`; half-step-aligned error `7.67566e-8`. At `10τ`, analytical/simulated `4.99977 A`, error `1.13367e-7`.
- Pi steady state: expected/simulated current `1.90476 A`, error `0`; load voltage `19.0476 V`, error `0`; line drop `.952381 V`, error `2.22045e-16`; power-balance absolute error `2.4869e-13 W`.
- Pi source-step final: expected/simulated current `1.73913 A`, error `4.07674e-13`; sending voltage `18.2609 V`, error `1.45661e-13`; receiving voltage `17.3913 V`, error `3.8014e-13`. Current and both shunt-voltage discontinuities at the step were zero; late-window drift/oscillation was zero.
- NaN/Inf: none; every dynamic sample is explicitly checked finite.

## 9. Known problems and risks

- The entire implementation is uncommitted, many product files are untracked, and no complete diff review has yet been recorded; accidental omission is the highest integration risk.
- The validation is an example executable, not a registered automated unit/CTest test; regression coverage will not run automatically.
- The RC/RL nominal-time values exhibit a half-step startup alignment (`1.84e-3 V` and `9.20e-4 A` nominal error). This is documented and tightly matches the half-step analytical value, but the initialization/first-step convention still requires architectural acceptance.
- Mixed `Ph3` AC/DC multiport metadata and base APIs remain unresolved; do not start converter work by assigning a single component-wide `PhaseType::DC`.
- Pi-line is a composite of SSN primitives rather than one coupled state-space stamp. Its topology and task ordering pass current tests but need full diff/API review.
- No compile errors, failing validations, NaN/Inf, or observed divergence remain in the focused target. Broader test-suite compatibility has not been established.

## 10. Exact next task

Smallest next milestone: convert the existing scalar validation executable into an automatically registered CTest (or the repository’s directly adjacent established C++ test mechanism) without changing component equations or APIs.

- Likely changes: `dpsim/examples/cxx/CMakeLists.txt` and/or the directly relevant test CMake file; reuse `dpsim/examples/cxx/Circuits/EMT_DC_SSN_Validation.cpp` unless the established test layout requires moving/copying it.
- Must not change: DC component headers/sources, generic SSN/MNA infrastructure, equations, initialization convention, pybind, or any AC/DC/MMC code.
- Build target: `EMT_DC_SSN_Validation`.
- Required analytical test: RC and RL one-time-constant plus final-value comparisons already in the executable, including finite-value checks; retain resistor/source PSC and pi-line checks.
- Completion: configure only if test registration requires it; build with `cmake --build build --target EMT_DC_SSN_Validation --parallel 2`; run the smallest registered test via `ctest --test-dir build -R EMT_DC_SSN_Validation --output-on-failure`; all current numerical tolerances pass; `ctest -N` shows the test; review the complete task diff. Stop there.

## 11. Recommended next-session prompt

```text
Read /root/dpsim/AGENTS.md and /root/dpsim/docs/development/SSN_DC_SESSION_HANDOVER.md first. Do not repeat repository-wide analysis. Inspect only the existing EMT_DC_SSN_Validation source, its directly adjacent CMake registration, and one nearby established C++ test-registration example if needed.

Perform only the bounded next task from section 10: register the existing scalar DC SSN validation as an automated test without changing DC component equations, APIs, initialization, generic SSN/MNA infrastructure, pybind, or AC/DC/MMC code. Preserve the build directory and Unix Makefiles. Build EMT_DC_SSN_Validation with at most `--parallel 2`, run the smallest registered test, confirm RC/RL analytical checks and NaN/Inf checks remain passing, inspect the complete task diff, and stop. Do not commit or push.
```
