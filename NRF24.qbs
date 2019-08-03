import qbs
import qbs.FileInfo
Product
{
    type: ["application","hex","size"]
    Depends { name: "cpp" }
    property string vendor: "ST"
    property string target: "STM32F1xx"
    property string drv_path: "Drivers/"
    property string src_path: "App/Src/"
    property string inc_path: "App/Inc/"
    property string periph_driver_path: drv_path + "STM32F10x_SPL/"
    property string cmsis_path: drv_path + "CMSIS/"

//"STM32F10X_LD_VL",
    cpp.defines:
    [ "STM32F10X_MD_VL",
        "ARM_MATH_CM3",
        "USE_STDPERIPH_DRIVER",
        "STM32F103xB"
    ]
    cpp.positionIndependentCode: false
    cpp.enableExceptions: false
    cpp.executableSuffix: ".elf"
    cpp.driverFlags:
    [
        "-mthumb",
        "-mcpu=cortex-m3",
        "-fno-strict-aliasing",
        "-g3",
        "-Wall",
        "-mfpu=vfp",
        "-flto",
    ]

    cpp.commonCompilerFlags:
    [
        "-mthumb",
        "-mcpu=cortex-m3",
        "-msoft-float",
        "-mfpu=vfp",
        "-Os",
        "-fdata-sections",
        "-ffunction-sections",
        "-fno-inline",
        "-flto"
    ]

    cpp.linkerFlags:
    [
        "-flto",
        "-mthumb",
        "-mcpu=cortex-m3",
        "-msoft-float",
        "-mfpu=vfp",
        "-specs=nosys.specs",
        "-specs=nano.specs",

        "-Wl,--start-group",
        "-Wl,--gc-sections",
        "-T"+path+"/App/Src/STM32F103X8.ld",
        //"-lnosys",
        "-lgcc",
        "-lc",
        "-u_printf_float",
        "-lstdc++",

        "-lm"
    ]

    cpp.includePaths:
    [
        cmsis_path+"Include",
        cmsis_path+"/Device/ST/",
        inc_path,
        periph_driver_path+"Inc"
    ]
    Group
    {
        name:"App"
        prefix: "App/**/"
        files:["*.s",
            "*.c",
            "*.cpp",
            "*.h"
        ]
    }
    Group
    {
        name: "Drivers"
        prefix: drv_path+"/**/"
        files:
        [
            "*.c",
            "*.cpp",
            "*.h"
        ]
    }

    Properties
    {
        condition: qbs.buildVariant === "debug"
        cpp.defines: outer.concat(["DEBUG=1"])
        cpp.debugInformation: true
        cpp.optimization: "none"
    }

    Properties
    {
        condition: qbs.buildVariant === "release"
        cpp.debugInformation: false
        cpp.optimization: "small"
    }
    Group {
        qbs.install: true
        fileTagsFilter: "application"
    }
    Rule {
        id: hex
        inputs: "application"
        Artifact {
            fileTags: ["hex"]
            filePath: input.baseName + ".hex"
        }
        prepare: {
            var args = ["-O", "ihex", input.filePath, output.filePath];
            var cmd = new Command("arm-none-eabi-objcopy", args);
            cmd.description = "converting to hex: "+FileInfo.fileName(input.filePath);
            cmd.highlight = "linker";
            return cmd;

        }
    }
    Rule {
        id: size
        inputs: ["application"]
        Artifact {
            fileTags: ["size"]
            filePath: "-"
        }
        prepare: {
            var args = [input.filePath];
            var cmd = new Command("arm-none-eabi-size", args);
            cmd.description = "File size: " + FileInfo.fileName(input.filePath);
            cmd.highlight = "linker";
            return cmd;
        }
    }
}
