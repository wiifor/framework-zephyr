# Copyright 2019-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import os
import sys

from SCons.Script import DefaultEnvironment

from platformio import util
from platformio.proc import exec_command
from platformio.util import get_systype

Import("env")


try:
    import yaml
    import pykwalify
except ImportError:
    env.Execute(
        env.VerboseAction(
            "$PYTHONEXE -m pip install pyyaml pykwalify",
            "Installing Zephyr's Python dependencies",
        )
    )

platform = env.PioPlatform()
board = env.BoardConfig()

FRAMEWORK_DIR = platform.get_package_dir("framework-zephyr")
assert os.path.isdir(FRAMEWORK_DIR)

BUILD_DIR = env.subst("$BUILD_DIR")
CMAKE_API_DIR = os.path.join(BUILD_DIR, ".cmake", "api", "v1")
CMAKE_API_QUERY_DIR = os.path.join(CMAKE_API_DIR, "query")
CMAKE_API_REPLY_DIR = os.path.join(CMAKE_API_DIR, "reply")

PLATFORMS_WITH_EXTERNAL_HAL = {
    "atmelsam": "atmel",
    "freescalekinetis": "nxp",
    "ststm32": "stm32",
    "siliconlabsefm32": "silabs",
    "nordicnrf51": "nordic",
    "nordicnrf52": "nordic",
    "nxplpc": "nxp"
}


def get_board_architecture(board_config):
    if board_config.get("build.cpu", "").lower().startswith("cortex"):
        return "arm"
    elif board_config.get("build.march", "") in ("rv64imac", "rv32imac"):
        return "riscv"
    elif board_config.get("build.mcu") == "esp32":
        return "xtensa32"

    sys.stderr.write(
        "Error: Cannot configure Zephyr environment for %s\n"
        % env.subst("$PIOPLATFORM")
    )
    env.Exit(1)


def populate_zephyr_env_vars(zephyr_env, board_config):
    toolchain_variant = "UNKNOWN"
    arch = get_board_architecture(board_config)
    if arch == "arm":
        toolchain_variant = "gnuarmemb"
        zephyr_env["GNUARMEMB_TOOLCHAIN_PATH"] = platform.get_package_dir(
            "toolchain-gccarmnoneeabi"
        )
    elif arch == "riscv":
        toolchain_variant = "cross-compile"
        zephyr_env["CROSS_COMPILE"] = os.path.join(
            platform.get_package_dir("toolchain-riscv"), "bin", "riscv64-unknown-elf-"
        )
    elif arch == "xtensa32":
        toolchain_variant = "espressif"
        zephyr_env["ESPRESSIF_TOOLCHAIN_PATH"] = platform.get_package_dir(
            "toolchain-xtensa32"
        )

    zephyr_env["ZEPHYR_TOOLCHAIN_VARIANT"] = toolchain_variant
    zephyr_env["ZEPHYR_BASE"] = FRAMEWORK_DIR

    additional_packages = [
        platform.get_package_dir("tool-dtc"),
        platform.get_package_dir("tool-ninja"),
    ]

    if "windows" not in get_systype():
        additional_packages.append(platform.get_package_dir("tool-gperf"))

    zephyr_env["PATH"] = (
        str(env["ENV"]["PATH"]) + os.pathsep + os.pathsep.join(additional_packages)
    )


def is_proper_zephyr_project():
    project_dir = env.subst("$PROJECT_DIR")
    cmakelist_present = os.path.isfile(os.path.join(
        project_dir, "zephyr", "CMakeLists.txt"))
    return cmakelist_present


def create_default_project_files():
    cmake_tpl = """cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(%s)

FILE(GLOB app_sources ../src/*.c*)
target_sources(app PRIVATE ${app_sources})
"""

    project_dir = env.subst("$PROJECT_DIR")
    cmake_txt_file = os.path.join(project_dir, "zephyr", "CMakeLists.txt")
    if not os.path.isfile(cmake_txt_file):
        os.makedirs(os.path.dirname(cmake_txt_file))
        with open(cmake_txt_file, "w") as fp:
            fp.write(cmake_tpl % os.path.basename(project_dir))

    if not os.listdir(os.path.join(env.subst("$PROJECT_SRC_DIR"))):
        # create an empty file to make CMake happy during first init
        open(os.path.join(env.subst("$PROJECT_SRC_DIR"), "empty.c"), "a").close()


def is_cmake_reconfigure_required():
    cmake_cache_file = os.path.join(BUILD_DIR, "CMakeCache.txt")
    cmake_txt_file = os.path.join(env.subst("$PROJECT_DIR"), "zephyr", "CMakeLists.txt")
    cmake_preconf_dir = os.path.join(BUILD_DIR, "zephyr", "include", "generated")

    for d in (CMAKE_API_REPLY_DIR, cmake_preconf_dir):
        if not os.path.isdir(d) or not os.listdir(d):
            return True
    if not os.path.isfile(cmake_cache_file):
        return True
    if not os.path.isfile(os.path.join(BUILD_DIR, "build.ninja")):
        return True
    if os.path.getmtime(cmake_txt_file) > os.path.getmtime(cmake_cache_file):
        return True

    return False


def run_cmake():
    print("Reading CMake configuration...")

    cmake_cmd = [
        os.path.join(platform.get_package_dir("tool-cmake") or "", "bin", "cmake"),
        "-S",
        os.path.join(env.subst("$PROJECT_DIR"), "zephyr"),
        "-B",
        BUILD_DIR,
        "-G",
        "Ninja",
        "-DBOARD=%s" % get_zephyr_target(board),
        "-DPYTHON_EXECUTABLE:FILEPATH=%s" % env.subst("$PYTHONEXE"),
    ]

    platform_name = env.subst("$PIOPLATFORM")
    if platform_name in PLATFORMS_WITH_EXTERNAL_HAL.keys():
        cmake_cmd.extend(
            [
                "-D",
                "ZEPHYR_MODULES="
                + platform.get_package_dir(
                    "framework-zephyr-hal-" + PLATFORMS_WITH_EXTERNAL_HAL[platform_name]),
            ]
        )

    zephyr_env = os.environ.copy()
    populate_zephyr_env_vars(zephyr_env, board)

    result = exec_command(cmake_cmd, env=zephyr_env)
    if result["returncode"] != 0:
        sys.stderr.write(result["out"] + "\n")
        sys.stderr.write(result["err"])
        env.Exit(1)

    if int(ARGUMENTS.get("PIOVERBOSE", 0)):
        print(result["out"])
        print(result["err"])


def get_cmake_code_model():
    query_file = os.path.join(CMAKE_API_QUERY_DIR, "codemodel-v2")
    if not os.path.isfile(query_file):
        os.makedirs(os.path.dirname(query_file))
        open(query_file, "a").close()  # create an empty file

    if not is_proper_zephyr_project():
        create_default_project_files()

    if is_cmake_reconfigure_required():
        run_cmake()

    if (
        not os.path.isdir(CMAKE_API_REPLY_DIR)
        or not os.listdir(CMAKE_API_REPLY_DIR)
    ):
        sys.stderr.write("Error: Couldn't find CMake API response file\n")
        env.Exit(1)

    codemodel = {}
    for target in os.listdir(CMAKE_API_REPLY_DIR):
        if target.startswith("codemodel-v2"):
            with open(os.path.join(CMAKE_API_REPLY_DIR, target), "r") as fp:
                codemodel = json.load(fp)

    assert codemodel["version"]["major"] == 2
    return codemodel


def get_zephyr_target(board_config):
    return board_config.get("build.zephyr.variant", env.subst("$BOARD").lower())


def get_target_elf_arch(board_config):
    architecture = get_board_architecture(board_config)
    if architecture == "arm":
        return "elf32-littlearm"
    if architecture == "riscv":
        if board.get("build.march", "") == "rv32imac":
            return "elf32-littleriscv"
        return "elf64-littleriscv"
    if architecture == "xtensa32":
        return "elf32-xtensa-le"

    sys.stderr.write(
        "Error: Cannot find correct elf architecture for %s\n"
        % env.subst("$PIOPLATFORM")
    )
    env.Exit(1)


def build_library(lib_config):
    lib_objects = compile_source_files(lib_config)
    lib_name = lib_config["name"]
    if lib_name.startswith("..__"):
        lib_name = lib_name.replace("..__", "")

    return env.Library(target=os.path.join("$BUILD_DIR", lib_name), source=lib_objects)


def get_target_config(project_configs, target_index):
    target_json = project_configs.get("targets")[target_index].get("jsonFile", "")
    target_config_file = os.path.join(CMAKE_API_REPLY_DIR, target_json)
    if not os.path.isfile(target_config_file):
        sys.stderr.write("Error: Couldn't find target config %s\n" % target_json)
        env.Exit(1)

    with open(target_config_file) as fp:
        return json.load(fp)


def generate_kobject_files():
    cmd = [
        "$PYTHONEXE",
        '"%s"' % os.path.join(FRAMEWORK_DIR, "scripts", "gen_kobject_list.py"),
        "--kobj-types-output",
        "${TARGETS[0]}",
        "--kobj-otype-output",
        "${TARGETS[1]}",
        "--kobj-size-output",
        "${TARGETS[2]}",
    ]

    return env.Command(
        [
            os.path.join(
                "$BUILD_DIR", "zephyr", "include", "generated", "kobj-types-enum.h"
            ),
            os.path.join(
                "$BUILD_DIR", "zephyr", "include", "generated", "otype-to-str.h"
            ),
            os.path.join(
                "$BUILD_DIR", "zephyr", "include", "generated", "otype-to-size.h"
            ),
        ],
        None,
        env.VerboseAction(" ".join(cmd), "Generating KObject files $TARGETS"),
    )


def validate_driver_cmd():

    cmd = [
        "$PYTHONEXE",
        '"%s"' % os.path.join(FRAMEWORK_DIR, "scripts", "gen_kobject_list.py"),
        "--validation-output",
        "$TARGET",
    ]

    return env.Command(
        os.path.join(
            "$BUILD_DIR", "zephyr", "include", "generated", "driver-validation.h"
        ),
        None,
        env.VerboseAction(" ".join(cmd), "Validating drivers $TARGET"),
    )


def generate_syscall_macro_header():
    cmd = [
        "$PYTHONEXE",
        '"%s"' % os.path.join(FRAMEWORK_DIR, "scripts", "gen_syscall_header.py"),
        ">",
        "$TARGET",
    ]

    return env.Command(
        os.path.join(
            "$BUILD_DIR", "zephyr", "include", "generated", "syscall_macros.h"
        ),
        None,
        env.VerboseAction(" ".join(cmd), "Generating syscall macro header $TARGET"),
    )


def parse_syscalls():
    cmd = [
        "$PYTHONEXE",
        '"%s"' % os.path.join(FRAMEWORK_DIR, "scripts", "parse_syscalls.py"),
        "--include",
        '"%s"' % os.path.join(FRAMEWORK_DIR, "include"),
        "--json-file",
        "$TARGET",
    ]

    return env.Command(
        os.path.join("$BUILD_DIR", "zephyr", "include", "generated", "syscalls.json"),
        None,
        env.VerboseAction(" ".join(cmd), "Parsing system calls from $TARGET"),
    )


def generate_syscall_files(syscalls_json):
    cmd = [
        "$PYTHONEXE",
        '"%s"' % os.path.join(FRAMEWORK_DIR, "scripts", "gen_syscalls.py"),
        "--json-file",
        "$SOURCE",
        "--base-output",
        os.path.join("$BUILD_DIR", "zephyr", "include", "generated", "syscalls"),
        "--syscall-dispatch",
        "${TARGETS[0]}",
        "--syscall-list",
        "${TARGETS[1]}",
    ]

    return env.Command(
        [
            os.path.join(
                BUILD_DIR, "zephyr", "include", "generated", "syscall_dispatch.c"
            ),
            os.path.join(BUILD_DIR, "zephyr", "include", "generated", "syscall_list.h"),
            os.path.join(
                "$BUILD_DIR",
                "zephyr",
                "include",
                "generated",
                "syscalls",
                "errno_private.h",
            ),
            os.path.join(
                "$BUILD_DIR", "zephyr", "include", "generated", "syscalls", "atomic.h"
            ),
            os.path.join(
                "$BUILD_DIR", "zephyr", "include", "generated", "syscalls", "device.h"
            ),
            os.path.join(
                "$BUILD_DIR", "zephyr", "include", "generated", "syscalls", "kernel.h"
            ),
            os.path.join(
                "$BUILD_DIR",
                "zephyr",
                "include",
                "generated",
                "syscalls",
                "sys_clock.h",
            ),
        ],
        syscalls_json,
        env.VerboseAction(" ".join(cmd), "Generating syscall files"),
    )


def extract_link_flags(target_config):
    def _split_flags(flags_str):
        i = 0
        result = []
        flag = []
        while i < len(flags_str.strip()):
            if flags_str[i] == " " and flags_str[i + 1] == "-":
                result.append("".join(flag).strip())
                flag.clear()
            flag.append(flags_str[i])
            i += 1

        return result

    link_flags = []
    for f in (
        target_config.get("link", {})
        .get("commandFragments", [])[0]
        .get("fragment")
        .split()
    ):
        if f.endswith((".a", ".o", ".obj")) or f.startswith(
            ("-Wl,--no-whole-archive", "-Wl,--whole-archive")
        ):
            continue

        # With -Wl,-T GCC is not able to pick ld script that's not in CWD
        if f == "-Wl,-T":
            f = "-T"
        link_flags.append(f)

    return link_flags


def get_linkerscript_final_cmd(app_includes, base_ld_script):
    cmd = [
        "$CC",
        "-x",
        "assembler-with-cpp",
        "-undef",
        "-MD",
        "-MF",
        "${TARGET}.dep",
        "-MT",
        "$TARGET",
        "-D__GCC_LINKER_CMD__",
        "-DLINKER_PASS2",
        "-E",
        "$SOURCE",
        "-P",
        "-o",
        "$TARGET",
    ]

    cmd.extend(['-I"%s"' % inc for inc in app_includes])

    return env.Command(
        os.path.join("$BUILD_DIR", "zephyr", "linker_pass_final.cmd"),
        base_ld_script,
        env.VerboseAction(" ".join(cmd), "Generating final linker script $TARGET"),
    )


def find_base_ldscript(app_includes):
    # A temporary solution since there is no an easy way to find linker script
    for inc in app_includes:
        for f in os.listdir(inc):
            if f == "linker.ld" and os.path.isfile(os.path.join(inc, f)):
                return os.path.join(inc, f)

    sys.stderr.write("Error: Couldn't find a base linker script!\n")
    env.Exit(1)


def get_linkerscript_cmd(app_includes, base_ld_script):
    cmd = [
        "$CC",
        "-x",
        "assembler-with-cpp",
        "-undef",
        "-MD",
        "-MF",
        "${TARGET}.dep",
        "-MT",
        "$TARGET",
        "-D__GCC_LINKER_CMD__",
        "-E",
        "$SOURCE",
        "-P",
        "-o",
        "$TARGET",
    ]

    cmd.extend(['-I"%s"' % inc for inc in app_includes])

    return env.Command(
        os.path.join("$BUILD_DIR", "zephyr", "linker.cmd"),
        base_ld_script,
        env.VerboseAction(" ".join(cmd), "Generating linker script $TARGET"),
    )


def load_target_configurations(cmake_codemodel):
    configs = {}
    project_configs = cmake_codemodel.get("configurations")[0]
    for config in project_configs.get("projects", []):
        for target_index in config.get("targetIndexes", []):
            target_config = get_target_config(project_configs, target_index)
            configs[target_config["name"]] = target_config

    return configs


def prepare_build_envs(config):
    build_envs = []
    target_compile_groups = config.get("compileGroups")
    is_build_type_debug = (
        set(["debug", "sizedata"]) & set(COMMAND_LINE_TARGETS)
        or env.GetProjectOption("build_type") == "debug"
    )

    for cg in target_compile_groups:
        includes = []
        sys_includes = []
        for inc in cg.get("includes", []):
            inc_path = inc["path"]
            if inc.get("isSystem", False):
                sys_includes.append(inc_path)
            else:
                includes.append(inc_path)

        defines = [inc.get("define") for inc in cg.get("defines", [])]
        compile_commands = cg.get("compileCommandFragments", [])
        for cc in compile_commands:
            build_env = env.Clone()
            build_flags = cc.get("fragment")
            build_env.AppendUnique(**build_env.ParseFlags(build_flags))
            build_env.Append(
                CPPDEFINES=defines,
                CPPPATH=includes,
            )

            if sys_includes:
                build_env.Append(CCFLAGS=[("-isystem", inc) for inc in sys_includes])

            build_env.Append(ASFLAGS=build_env.get("CCFLAGS", [])[:])

            build_env.ProcessUnFlags(env.get("BUILD_UNFLAGS"))
            if is_build_type_debug:
                build_env.ConfigureDebugFlags()
            build_envs.append(build_env)

    return build_envs


def compile_source_files(config):
    build_envs = prepare_build_envs(config)
    config_name = config["name"]
    if config_name.startswith("..__"):
        config_name = config_name.replace("..__", "")
    objects = []
    for source in config.get("sources", []):
        if source["path"].endswith(".rule"):
            continue
        compile_group_idx = source.get("compileGroupIndex")
        if compile_group_idx is not None:
            src_path = source.get("path")
            if not os.path.isabs(source.get("path")):
                # For cases when sources are located near CMakeLists.txt
                src_path = os.path.join(env.subst("$PROJECT_DIR"), "zephyr", src_path)
            objects.append(
                build_envs[compile_group_idx].StaticObject(
                    target=os.path.join(
                        "$BUILD_DIR",
                        config_name,
                        os.path.basename(os.path.dirname(src_path)),
                        os.path.basename(src_path) + ".o",
                    ),
                    source=os.path.realpath(src_path),
                )
            )

    return objects


def get_app_includes(app_config):
    plain_includes = []
    sys_includes = []
    cg = app_config["compileGroups"][0]
    for inc in cg.get("includes", []):
        inc_path = inc["path"]
        if inc.get("isSystem", False):
            sys_includes.append(inc_path)
        else:
            plain_includes.append(inc_path)

    plain_includes.append(os.path.join(
        env.subst("$BUILD_DIR"), "zephyr", "include", "generated"))

    return {
        "plain_includes": plain_includes,
        "sys_includes": sys_includes
    }


def get_app_defines(app_config):
    return [
        inc.get("define") for inc in app_config["compileGroups"][0].get("defines", [])
    ]


def get_firmware_flags(app_config, main_config):
    # Use the first compile commands group
    result = {}
    app_flags = {}
    for cg in app_config["compileGroups"]:
        app_flags[cg["language"]] = env.ParseFlags(
            cg["compileCommandFragments"][0]["fragment"])

    if "C" in app_flags.keys():
        result.update(app_flags["C"])
    if "CXX" in app_flags.keys():
        cxx_section = app_flags["CXX"]
        if not result.get("CCFLAGS", []):
            result.update(cxx_section)
        else:
            # Flags that are not present in CC and CXX sections
            cflags = set(result["CCFLAGS"]) - set(cxx_section["CCFLAGS"])
            # Flags that are not present in C and CC sections
            cxx_flags = set(cxx_section["CCFLAGS"]) - set(result["CCFLAGS"])
            # Common flags for both C and CXX sections
            ccflags = set(result["CCFLAGS"]) - cxx_flags - cflags
            result["CFLAGS"].extend(list(cflags))
            result["CXXFLAGS"] = list(cxx_flags) + cxx_section["CXXFLAGS"]
            result["CCFLAGS"] = list(ccflags)
    if "ASM" in app_flags.keys():
        result["ASFLAGS"] = list(app_flags["CCFLAGS"])

    if not result:
        sys.stderr.write("Error: No build flags found for app target\n")
        env.Exit(1)

    standard_libs = ("-lgcc", "-lc", "-lm")
    app_link_flags = extract_link_flags(main_config)

    # Ignore ld script and standard libraries as they'll be specified in a special place
    result["LINKFLAGS"] = [
        f
        for f in app_link_flags
        if not f.endswith(".cmd") and f != "-T" and f not in standard_libs
    ]

    return result


def generate_isr_list_binary(preliminary_elf, board):
    cmd = [
        "$OBJCOPY",
        "--input-target=" + get_target_elf_arch(board),
        "--output-target=binary",
        "--only-section=.intList",
        "$SOURCE",
        "$TARGET",
    ]

    return env.Command(
        os.path.join("$BUILD_DIR", "zephyr", "isrList.bin"),
        preliminary_elf,
        env.VerboseAction(" ".join(cmd), "Generating ISR list $TARGET"),
    )


def generate_isr_table_file_cmd(preliminary_elf, board_config):
    cmd = [
        "$PYTHONEXE",
        '"%s"' % os.path.join(FRAMEWORK_DIR, "arch", "common", "gen_isr_tables.py"),
        "--output-source",
        "$TARGET",
        "--kernel",
        "${SOURCES[0]}",
        "--intlist",
        "${SOURCES[1]}",
        "--debug",
    ]

    config_file = os.path.join(env.subst("$BUILD_DIR"), "zephyr", ".config")

    if os.path.isfile(config_file):
        with open(config_file) as fp:
            data = fp.read()
        if "CONFIG_GEN_ISR_TABLES=y" in data:
            cmd.append("--sw-isr-table")
        if "CONFIG_GEN_IRQ_VECTOR_TABLE=y" in data:
            cmd.append("--vector-table")

    cmd = env.Command(
        os.path.join("$BUILD_DIR", "zephyr", "isr_tables.c"),
        [preliminary_elf, os.path.join("$BUILD_DIR", "zephyr", "isrList.bin")],
        env.VerboseAction(" ".join(cmd), "Generating ISR table $TARGET"),
    )

    env.Requires(cmd, generate_isr_list_binary(preliminary_elf, board_config))

    return cmd


def generate_offset_header_file_cmd():
    cmd = [
        "$PYTHONEXE",
        '"%s"' % os.path.join(FRAMEWORK_DIR, "scripts", "gen_offset_header.py"),
        "-i",
        "$SOURCE",
        "-o",
        "$TARGET",
    ]

    return env.Command(
        os.path.join("$BUILD_DIR", "zephyr", "include", "generated", "offsets.h"),
        os.path.join("$BUILD_DIR", "offsets", "offsets", "offsets.c.o"),
        env.VerboseAction(
            " ".join(cmd), "Generating header file with offsets $TARGET",
        ),
    )


def build_offsets_lib(target_configs):
    lib = build_library(target_configs["offsets"])

    env.Depends(
        lib[0].sources[0],
        generate_syscall_files(parse_syscalls())
        + generate_syscall_macro_header()
        + generate_kobject_files(),
    )

    return lib


#
# Current build script limitations
#

env.EnsurePythonVersion(3, 4)

if " " in FRAMEWORK_DIR:
    sys.stderr.write("Error: Detected a whitespace character in framework path\n")
    env.Exit(1)

#
# Initial targets loading
#

codemodel = get_cmake_code_model()
if not codemodel:
    sys.stderr.write("Error: Couldn't find code model generated by CMake\n")
    env.Exit(1)

target_configs = load_target_configurations(codemodel)

app_config = target_configs.get("app")
prebuilt_config = target_configs.get("zephyr_prebuilt")

if not app_config or not prebuilt_config:
    sys.stderr.write("Error: Couldn't find main Zephyr target in the code model\n")
    env.Exit(1)

offset_header_file = generate_offset_header_file_cmd()

#
# LD scripts processing
#

app_includes = get_app_includes(app_config)
base_ld_script = find_base_ldscript(app_includes["plain_includes"])
final_ld_script = get_linkerscript_final_cmd(
    app_includes["plain_includes"], base_ld_script)
preliminary_ld_script = get_linkerscript_cmd(app_includes["plain_includes"], base_ld_script)

env.Depends(final_ld_script, offset_header_file)
env.Depends(preliminary_ld_script, offset_header_file)

#
# Libraries processing
#

framework_libs_map = {}
for target, target_config in target_configs.items():
    lib_name = target_config["name"]
    if target_config["type"] not in (
        "STATIC_LIBRARY",
        "OBJECT_LIBRARY",
    ) or lib_name in ("app", "offsets"):
        continue

    lib = build_library(target_config)
    framework_libs_map[target_config["id"]] = lib

    if any(
        d.get("id", "").startswith("offsets_h")
        for d in target_config.get("dependencies", [])
    ):
        env.Depends(lib[0].sources, offset_header_file)

# Offsets library compiled separately as it requires additional dependencies
offsets_lib = build_offsets_lib(target_configs)

#
# Preliminary elf and subsequent targets
#

preliminary_elf_path = os.path.join("$BUILD_DIR", "firmware-pre.elf")
env.Depends(preliminary_elf_path, os.path.join(BUILD_DIR, "libkernel.a"))

for dep in (offsets_lib, preliminary_ld_script):
    env.Depends(preliminary_elf_path, dep)

isr_table_file = generate_isr_table_file_cmd(preliminary_elf_path, board)

#
# Final elf target
#

env["_EXTRA_ZEPHYR_PIOBUILDFILES"] = compile_source_files(
    target_configs["zephyr_final"]
)

for dep in (isr_table_file, final_ld_script):
    env.Depends("$PROG_PATH", dep)

libs = [
    framework_libs_map[d["id"]]
    for d in prebuilt_config.get("dependencies", [])
    if framework_libs_map.get(d["id"], {}) and not d["id"].startswith(("kernel", "app"))
]

env.Replace(ARFLAGS=["qc"])
env.Prepend(_LIBFLAGS="-Wl,--whole-archive ")

# Note: libc and kernel libraries must be placed explicitly after zephyr libraries
# outside of whole-archive flag
env.Append(
    CPPPATH=app_includes["plain_includes"],
    CPPDEFINES=get_app_defines(app_config),
    CCFLAGS=[("-isystem", inc) for inc in app_includes.get("sys_includes", [])],
    PIOBUILDFILES=compile_source_files(prebuilt_config),
    LIBS=sorted(libs) + [offsets_lib],
    _LIBFLAGS=" -Wl,--no-whole-archive -lkernel -lc -lm -lgcc",
    BUILDERS=dict(
        ElfToBin=Builder(
            action=env.VerboseAction(
                " ".join(
                    [
                        "$OBJCOPY",
                        "--gap-fill",
                        "0xff",
                        "--remove-section=.comment",
                        "--remove-section=COMMON",
                        "--remove-section=.eh_frame",
                        "-O",
                        "binary",
                        "$SOURCES",
                        "$TARGET",
                    ]
                ),
                "Building $TARGET",
            ),
            suffix=".bin",
        ),
        ElfToHex=Builder(
            action=env.VerboseAction(
                " ".join(
                    [
                        "$OBJCOPY",
                        "-O",
                        "ihex",
                        "--remove-section=.comment",
                        "--remove-section=COMMON",
                        "--remove-section=.eh_frame",
                        "$SOURCES",
                        "$TARGET",
                    ]
                ),
                "Building $TARGET",
            ),
            suffix=".hex",
        ),
    ),
)

env.AppendUnique(**get_firmware_flags(app_config, prebuilt_config))

if get_board_architecture(board) == "arm":
    env.Replace(
        SIZEPROGREGEXP=r"^(?:text|_TEXT_SECTION_NAME_2|sw_isr_table|devconfig|rodata|\.ARM.exidx)\s+(\d+).*",
        SIZEDATAREGEXP=r"^(?:datas|bss|noinit|initlevel|_k_mutex_area|_k_stack_area)\s+(\d+).*",
    )
