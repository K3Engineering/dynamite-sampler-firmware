# dynamite-sampler-firmware

Dynamite sampler board firmware

## Development

Use the "Add .vscode subdirectory files" to generate a json that will allow vscode to
figure out where things are defined.

### Configurations & `sdkconfig`

ESP & the vscode plugin had a weird way of handling their configurations.
- `sdkconfig` is only generated if it doesn't exist. If the `sdkconfig.defaults` changes, 
nothing happens. You have to delete the `sdkconfig`.
- The VS code plugin figures out the target from the current `sdkconfig`. So if you want 
to regenerate it, you have to delete it, and then reconfigure the target.

This repo deviates from the standard approach.

`esp_idf_project_configuration.json` is used to configure either a debug or release build.
This is for the VS Code plugin.
It change the defaults of `sdkconfig` to be placed in `build/sdkconfig.<config>`, 
where `<config>` is debug or release.

Instead of `sdkconfig.defaults`, there is `sdkconfig.common` and `sdkconfig.<config>`.

To use these manual changes from the CLI, or github actions, there is a script that parses it, 
and generates the equivalent build flags: `generate_idf_flags_from_config_json.py`.

Documentation on multiple configs here: https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/additionalfeatures/multiple-projects.html

You can still use `menuconfig` to open and edit the correct `sdkconfig` to make non-permanent changes.

If you want generate a `sdkconfig.defaults` you can run the following command:

```
idf.py -B build/<config> "-DSDKCONFIG=build/sdkconfig.<config>" save-defconfig
idf.py -B build/release "-DSDKCONFIG=build/sdkconfig.release" save-defconfig
idf.py -B build/debug "-DSDKCONFIG=build/sdkconfig.debug" save-defconfig
```

The quote around the `DSDKCONFIG` are important, it crashes without it.
You can't use the `generate_idf_flags_from_config_json` script because `save-defconfig` 
doesn't like absolute paths.

`idf.py save-defconfig` reads the `sdkconfig` file, and generates a `sdkconfig.defaults` file
which contains only the changes relative to the preset defaults.

You can then move the configurations into the appropriate `.commonn` or `.<config>` file.

### Debuger / breakpoints

launching openocd `openocd -f board/esp32s3-builtin.cfg `
`-d4` for highests debug level

`usblogview` and `USBDriverTool` were quite helpful
