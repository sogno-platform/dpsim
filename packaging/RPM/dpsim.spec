Name:       dpsim
Version:    0.1
Release:    1%{?dist}
Summary:    Real-time power system simulator including powerflow, (dynamic) phasors and EMT

Group:      Applications/Engineering
License:    MPL

%description

DPsim is a solver library for dynamic power system simulation.

- It supports both the electromagnetic transient (EMT) and dynamic phasor (DP) domain for dynamic simulation.
- A powerflow solver is included for standalone usage or initialization of dynamic simulations.
- It provides a Python module which can be embedded in any Python 3 application / scripts.
- The simulation core is implemented in highly-efficient C++ code.
- It supports real-time execution with time-steps down to 50 uS.
- It can load models in the IEC61970 CIM / CGMES XML format.
- It can be interfaced to a variety of protocols and interfaces via VILLASnode.

%define _arch x86_64

%prep

%build

%install
mkdir -p %{buildroot}

mkdir -p %{buildroot}%{_libdir}
mkdir -p %{buildroot}%{_libdir}/python3.7/site-packages

cp -rfa /home/root/workspace/dpsim/packaging/RPM/libs/* %{buildroot}%{_libdir}
mv %{buildroot}%{_libdir}/dpsimpy* ${buildroot}%{_libdir}/python3.7/site-packages

cd %{buildroot}%{_libdir}

%files
/*
