{
  description = "Double pendulum";

  inputs = { nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable"; };

  outputs = { self, nixpkgs }:
    let forAllSystems = nixpkgs.lib.genAttrs nixpkgs.lib.systems.flakeExposed;
    in {
      packages = forAllSystems (system: rec {
        default = dpend;
        dpend = nixpkgs.legacyPackages.${system}.stdenv.mkDerivation {
          name = "dpend";
          version = "1.0.0";
          src = ./.;
          buildInputs = [ ];
          buildPhase = "./build release";
          installPhase = ''
            mkdir -p $out/bin
            cp out/dpend $out/bin
          '';
        };
      });
    };
}
