{
  lib,

  patchelf,

  cimpp,
  cmake,
  eigen,
  fmt,
  gsl,
  nlohmann_json,
  pkg-config,
  python312,
  python312Packages,
  readerwriterqueue,
  spdlog,
  sundials321,
  graphviz,
  suitesparse-dpsim,
  villas-node,

  cudatoolkit,
  magma,

  # Options
  withAllExtras ? true,
  withOpenMP ? withAllExtras,
  withCIMpp ? withAllExtras,
  withVILLAS ? withAllExtras,
  withGSL ? withAllExtras,
  withGraphviz ? withAllExtras,
  withPybind ? withAllExtras,
  withSundials ? withAllExtras,
  withSuiteSparse ? withAllExtras,
  withMNASolverPlugin ? withAllExtras,
}:
with python312Packages;
buildPythonPackage {
  name = "dpsimpy";
  src = ../..;

  nativeBuildInputs = [
    cmake
    pkg-config
  ];

  pyproject = true;

  build-system = [
    setuptools
    setuptools-scm
  ];

  dependencies = [ pytest-runner ];

  buildInputs =
    [
      eigen
      fmt
      spdlog
      nlohmann_json
      readerwriterqueue

      # TODO: Add these dependencies
      # cudatoolkit
      # magma
    ]
    ++ lib.optional withCIMpp cimpp
    ++ lib.optional withVILLAS villas-node
    ++ lib.optional withGSL gsl
    ++ lib.optional withGraphviz graphviz
    ++ lib.optional withSundials sundials321
    ++ lib.optional withSuiteSparse suitesparse-dpsim
    ++ lib.optionals withPybind [
      python312
      python312Packages.pybind11
    ];

  enableParallelBuilding = true;
  dontUseCmakeConfigure = true;

  preBuild = ''
    pypaBuildFlags+="-C--global-option=build_ext -C--global-option=--parallel=$NIX_BUILD_CORES"
  '';

  env = {
    # NIX_DEBUG = "7";
    CMAKE_OPTS = builtins.concatStringsSep " " [
      # LTO is currently broken in Nixpkgs (https://github.com/NixOS/nixpkgs/issues/384599)
      "-DWITH_LTO=OFF"

      "-DBUILD_SHARED_LIBS=ON"

      # Feature flags
      "-DWITH_OPENMP=${if withOpenMP then "ON" else "OFF"}"
      "-DWITH_CIM=${if withCIMpp then "ON" else "OFF"}"
      "-DWITH_VILLAS=${if withVILLAS then "ON" else "OFF"}"
      "-DWITH_GSL=${if withGSL then "ON" else "OFF"}"
      "-DWITH_MNASOLVER_PLUGIN=${if withMNASolverPlugin then "ON" else "OFF"}"

      "-DDPSIM_BUILD_EXAMPLES=OFF"

      # We can not fetch external code within the Nix sandbox
      # Instead we packaged those dependencies as separate Nix derivations.
      "-DFETCH_EIGEN=OFF"
      "-DFETCH_SUITESPARSE=OFF"
      "-DFETCH_SPDLOG=OFF"
      "-DFETCH_CIMPP=OFF"
      "-DFETCH_PYBIND=OFF"
      "-DFETCH_GRID_DATA=OFF"
      "-DFETCH_FILESYSTEM=OFF"
      "-DFETCH_JSON=OFF"
      "-DFETCH_READERWRITERQUEUE=OFF"

      "-DCMAKE_SKIP_BUILD_RPATH=ON"
    ];
  };
}
