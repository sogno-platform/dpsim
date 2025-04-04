{
  description = "DPsim - Solver library for dynamic power system simulation";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    villas-node = {
      url = "github:VILLASframework/node";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      villas-node,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;

          # Required for old sundials version
          config.permittedInsecurePackages = [ "python-2.7.18.8" ];

          overlays = [
            villas-node.overlays.default

            (final: prev: {
              readerwriterqueue = final.callPackage ./packaging/Nix/readerwriterqueue.nix { };
              cimpp = final.callPackage ./packaging/Nix/cimpp.nix { };
              suitesparse-dpsim = prev.callPackage ./packaging/Nix/suitesparse.nix { };
              sundials321 = prev.callPackage ./packaging/Nix/sundials.nix { };
              dpsim = pkgs.callPackage ./packaging/Nix/dpsim.nix { };
              dpsimpy = pkgs.callPackage ./packaging/Nix/dpsimpy.nix { };
            })
          ];
        };
      in
      {
        packages = {
          default = pkgs.dpsim;

          inherit (pkgs)
            dpsim
            dpsimpy
            readerwriterqueue
            cimpp
            ;
        };

        devShells = {
          default = pkgs.callPackage ./packaging/Nix/shell.nix { };
        };
      }
    );
}
