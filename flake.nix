{
  description = "DPsim - Solver library for dynamic power system simulation";

  inputs = {
    nixpkgs.url     = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    villas-node = {
      url = "github:VILLASframework/node";
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
            (final: prev: let
            villasPkg = villas-node.packages.${system}.villas-node;
            in {
              readerwriterqueue = final.callPackage ./packaging/Nix/readerwriterqueue.nix { };
              cimpp             = final.callPackage ./packaging/Nix/cimpp.nix { };
              suitesparse-dpsim = prev.callPackage  ./packaging/Nix/suitesparse.nix { };
              sundials321       = prev.callPackage  ./packaging/Nix/sundials.nix { };
              dpsim             = final.callPackage ./packaging/Nix/dpsim.nix { villas-node=villasPkg; };
              dpsimpy           = final.callPackage ./packaging/Nix/dpsimpy.nix { };
            })
          ];
        };
      in {
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
