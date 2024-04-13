{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";

    flake-utils.url = "github:numtide/flake-utils";

    esp.url = "github:mirrexagon/nixpkgs-esp-dev";
    esp.inputs.nixpkgs.follows = "nixpkgs";
    esp.inputs.flake-utils.follows = "flake-utils";
  };

  outputs = { nixpkgs, flake-utils, esp, ... }: flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = nixpkgs.legacyPackages.${system};
      esp-pkgs = esp.packages.${system};
    in
    {
      packages =
        let

          # curl 'https://components.espressif.com/api/components/${name}' | jq -r '.versions[0].url'
          components = {
            "espressif/led_strip" = pkgs.fetchzip {
              url = "https://components-file.espressif.com/43713b73-fb80-412b-970b-46caea9feeb1.tgz";
              hash = "sha256-VtDpOS3hIn+xLM3uCr0BXLi4evAI2w+cJrQy+hujXLQ";
              stripRoot = false;
            };
            "espressif/mdns" = pkgs.fetchzip {
              url = "https://components-file.espressif.com/54359b9f-347d-4f78-bc42-6244ee849ae2.tgz";
              hash = "sha256-8/T81wMIlBZ/uYPeARMBxncslhr1323ahgNY5V79CgU=";
              stripRoot = false;
            };
          };

          # Config is set using kconfig, see https://github.com/espressif/esp-idf-kconfig/blob/master/docs/DOCUMENTATION.md#kconfig-item-types. Note that the CONFIG_ prefix is stripped.
          defaultConfig = {
            # Disable signing the output binary, leaving it up to the user to sign.
            # https://docs.espressif.com/projects/esp-idf/en/stable/esp32/security/secure-boot-v2.html#remote-signing-of-images
            SECURE_SIGNED_APPS_NO_SECURE_BOOT = false;
            SECURE_BOOT_BUILD_SIGNED_BINARIES = false;
          };

          targets = {
            s3 = "-S3-MINI-N4-R2";
            pico = "-S1-PICO";
            wroom1 = "-S1-V1";
            wroom = "-S1";
            solo = "-S1-SOLO";
          };

          mkPackage = name: suffix: config: pkgs.runCommand
            "faikin-${name}"
            {
              buildInputs = with pkgs; [
                esp-pkgs.esp-idf-full

                cmake
                gcc
                popt
                tcsh
              ];
            }
            ''
              # Copy source.
              cp -r ${./ESP32} src
              chmod -R +w src
              cd src
              if ! [ -e ESP32-BLE-Env ]; then
                >&2 echo "Missing submodules. You need to specify ?submodules=1 when invoking this flake (e.g. `nix build
                'git+https://github.com/MaienM/ESP32-Faikin?submodules=1#${name}'`)."
                exit 1
              fi

              # Patch hardcoded absolute paths in source.
              grep -Rl '/bin/csh' | xargs sed -i 's!/bin/csh!${pkgs.tcsh}/bin/tcsh!'

              # Disable components manager as we manage the components via Nix to get reproducible results.
              IDF_COMPONENT_MANAGER=0
              export IDF_COMPONENT_MANAGER

              # Link components.
              mkdir -p components/
              ${toString (pkgs.lib.attrsets.mapAttrsToList (name: path: ''
                ln -s ${path} "components/${baseNameOf name}"
              '') components)}

              # RevK's CMake file specifically looks in managed_components, so we need to mock the structure mock it expects so it'll do the right thing. It only looks for existence so we can just use files.
              mkdir -p managed_components
              ${toString (builtins.map (name: ''
                touch "managed_components/${builtins.replaceStrings [ "/" ] [ "__" ] name}"
              '') (builtins.attrNames components))}

              # Create sdkconfig with requested options
              make settings.h
              cp sdkconfig.defaults sdkconfig
              ./components/ESP32-RevK/setbuildsuffix ${suffix}
              ${pkgs.lib.strings.toShellVar "config_changes" (builtins.toJSON {
                version = 2;
                set = defaultConfig // config;
                save = null;
              })}
              echo "$config_changes" | idf.py confserver

              # Build.
              make | tee log

              # Take flash command from output and parse it.
              flash_command="$(grep 'esptool.py .* write_flash .*' log | sed 's/-p (PORT)//')"

              # Go through flash command arguments and copy any mentioned files to the output.
              mkdir -p $out/firmware
              flash_command_rewritten=()
              for arg in $flash_command; do
                if [[ "$arg" = *.bin ]]; then
                  cp "$arg" $out/firmware/
                  arg="$out/firmware/$(basename "$arg")"
                elif [[ "$arg" = ../* ]]; then
                  arg="$(realpath "$arg")"
                fi
                flash_command_rewritten=("''${flash_command_rewritten[@]}" "$arg")
              done

              # Figure out which file is the application image by just assuming it's the largest one.
              app_image="$out/firmware/$(ls -1S $out/firmware | head -n1)"

              # Create script to flash.
              mkdir -p $out/bin
              for k in "''${!flash_command_rewritten[@]}"; do
                if [ "''${flash_command_rewritten["$k"]}" = "$app_image" ]; then
                  flash_command_rewritten["$k"]=APP_IMAGE
                fi
              done
              sed 's/^  //' > $out/bin/flash <<END
                #!/usr/bin/env sh
                image="\''${1:-$(printf '%q' "$app_image")}"
                exec $(printf ' %q' "''${flash_command_rewritten[@]}")
              END
              sed -i 's/APP_IMAGE/"$image"/' $out/bin/flash
              chmod +x $out/bin/flash

              # Include sdkconfig for reference.
              cp sdkconfig $out/firmware/
            '';
        in
        builtins.mapAttrs
          (name: suffix:
            let withConfig = mkPackage name suffix;
            in (withConfig { }) // { inherit withConfig; }
          )
          targets;

      devShells.default = pkgs.mkShell {
        packages = with pkgs; [
          esp-pkgs.esp-idf-full

          python3.pkgs.python-pkcs11
        ];
      };
    });
}
