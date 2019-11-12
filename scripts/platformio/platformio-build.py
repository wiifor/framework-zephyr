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


from os import environ, listdir, makedirs, pathsep
from os.path import abspath, basename, dirname, isdir, isfile, join, getmtime
import sys
import json

from SCons.Script import DefaultEnvironment

from platformio import util
from platformio.proc import exec_command

Import("env")


try:
    import yaml
    import pykwalify
except ImportError:
    env.Execute("$PYTHONEXE -m pip install pyyaml pykwalify")

platform = env.PioPlatform()
board = env.BoardConfig()

FRAMEWORK_DIR = platform.get_package_dir("framework-zephyr")
assert isdir(FRAMEWORK_DIR)

BUILD_DIR = env.subst("$BUILD_DIR")
CMAKE_API_DIR = join(BUILD_DIR, ".cmake", "api", "v1")
CMAKE_API_QUERY_DIR = join(CMAKE_API_DIR, "query")
CMAKE_API_REPLY_DIR = join(CMAKE_API_DIR, "reply")
PLATFORMS_WITH_EXTERNAL_HAL = ("ststm32", "nordicnrf52")


def add_ordered_dependency(target, dependency):
    env.Depends(target, dependency)
    env.Requires(target, dependency)


def populate_zephyr_env_vars(zephyr_env, platform_name, board_config):
    toolchain_variant = "UKNOWN"
    if board_config.get("build.cpu", "").lower().startswith("cortex"):
        toolchain_variant = "gnuarmemb"
        zephyr_env["GNUARMEMB_TOOLCHAIN_PATH"] = platform.get_package_dir(
            "toolchain-gccarmnoneeabi"
        )
    elif platform_name == "sifive":
        toolchain_variant = "cross-compile"
        zephyr_env["CROSS_COMPILE"] = join(
            platform.get_package_dir("toolchain-riscv"), "bin", "riscv64-unknown-elf-"
        )
    elif board_config.get("build.mcu", "") == "esp32":
        toolchain_variant = "espressif"
        zephyr_env["ESPRESSIF_TOOLCHAIN_PATH"] = platform.get_package_dir(
            "toolchain-xtensa32"
        )
    else:
        sys.stderr.write(
            "Error: Cannot configure Zephyr environment for %s\n" % platform_name
        )
        env.Exit(1)

    zephyr_env["ZEPHYR_TOOLCHAIN_VARIANT"] = toolchain_variant
    zephyr_env["ZEPHYR_BASE"] = FRAMEWORK_DIR
    zephyr_env["PATH"] = (
        str(env["ENV"]["PATH"])
        + pathsep
        + pathsep.join(
            [
                platform.get_package_dir("tool-dtc"),
                join(platform.get_package_dir("tool-ninja")),
            ]
        )
    )


def is_proper_zephyr_project():
    project_dir = env.subst("$PROJECT_DIR")
    cmakelist_present = isfile(join(project_dir, "CMakeLists.txt"))
    return cmakelist_present


def create_default_project_files():
    cmake_tpl = """cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(%s)

FILE(GLOB app_sources src/*.c*)
target_sources(app PRIVATE ${app_sources})
"""

    project_dir = env.subst("$PROJECT_DIR")
    cmake_txt_file = join(project_dir, "CMakeLists.txt")
    if not isfile(cmake_txt_file):
        with open(cmake_txt_file, "w") as fp:
            fp.write(cmake_tpl % basename(project_dir))

    if len(listdir(join(env.subst("$PROJECT_SRC_DIR")))) == 0:
        # create an empty file to make CMake happy during first init
        open(join(env.subst("$PROJECT_SRC_DIR"), "empty.c"), "a").close()


def is_cmake_reconfigure_required():
    cmake_cache_file = join(BUILD_DIR, "CMakeCache.txt")
    cmake_txt_file = join(env.subst("$PROJECT_DIR"), "CMakeLists.txt")
    cmake_preconf_dir = join(BUILD_DIR, "zephyr", "include", "generated")

    for d in (CMAKE_API_REPLY_DIR, cmake_preconf_dir):
        if not isdir(d) or len(listdir(d)) == 0:
            return True
    if not isfile(cmake_cache_file):
        return True
    if not isfile(join(BUILD_DIR, "build.ninja")):
        return True
    if getmtime(cmake_txt_file) > getmtime(cmake_cache_file):
        return True

    return False


def run_cmake():
    print("Generating CMake files")

    cmake_cmd = [
        join(platform.get_package_dir("tool-cmake") or "", "bin", "cmake"),
        "-S",
        env.subst("$PROJECT_DIR"),
        "-B",
        BUILD_DIR,
        "-G",
        "Ninja",
        "-DBOARD=%s" % get_zephyr_target(board),
        "-DPYTHON_EXECUTABLE:FILEPATH=%s" % env.subst("$PYTHONEXE"),
    ]

    platform_name = env.subst("$PIOPLATFORM")
    if platform_name in PLATFORMS_WITH_EXTERNAL_HAL:
        cmake_cmd.extend(
            [
                "-D",
                "ZEPHYR_MODULES="
                + platform.get_package_dir("framework-zephyr-hal-" + platform_name),
            ]
        )

    zephyr_env = environ.copy()
    populate_zephyr_env_vars(zephyr_env, platform_name, board)

    result = exec_command(cmake_cmd, env=zephyr_env)
    if result["returncode"] != 0:
        sys.stderr.write(result["out"] + "\n")
        sys.stderr.write(result["err"])
        env.Exit(1)

    if int(ARGUMENTS.get("PIOVERBOSE", 0)):
        print(result["out"])


def get_cmake_code_model():
    query_file = join(CMAKE_API_QUERY_DIR, "codemodel-v2")
    if not isfile(query_file):
        makedirs(dirname(query_file))
        open(query_file, "a").close()  # create an empty file

    if not is_proper_zephyr_project():
        create_default_project_files()

    if is_cmake_reconfigure_required():
        run_cmake()

    if not isdir(CMAKE_API_REPLY_DIR) or len(listdir(CMAKE_API_REPLY_DIR)) == 0:
        sys.stderr.write("Error: Couldn't find CMake API response file\n")
        env.Exit(1)

    codemodel = {}
    for target in listdir(CMAKE_API_REPLY_DIR):
        if target.startswith("codemodel-v2"):
            with open(join(CMAKE_API_REPLY_DIR, target), "r") as fp:
                codemodel = json.load(fp)

    assert codemodel["version"]["major"] == 2
    return codemodel


def get_zephyr_target(board_config):
    return board_config.get("build.zephyr.variant", env.subst("$BOARD").lower())


def get_target_elf_arch(board_config, platform_name):
    if board_config.get("build.cpu", "").lower().startswith("cortex"):
        return "elf32-littlearm"
    if platform_name == "sifive":
        return "elf64-littleriscv"
    if platform_name == "espressif32":
        return "elf32-xtensa-le"

    sys.stderr.write(
        "Error: Cannot find correct elf architecture for  %s\n" % platform_name
    )
    env.Exit(1)


def build_library(lib_config):
    lib_objects = compile_source_files(lib_config)
    lib_name = lib_config["name"]
    if lib_name.startswith("..__"):
        lib_name = lib_name.replace("..__", "")

    return env.Library(target=join("$BUILD_DIR", lib_name), source=lib_objects)


def get_target_config(project_configs, target_index):
    target_json = project_configs.get("targets")[target_index].get("jsonFile", "")
    target_config_file = join(CMAKE_API_REPLY_DIR, target_json)
    if not isfile(target_config_file):
        sys.stderr.write("Error: Couldn't find target config %s\n" % target_json)
        env.Exit(1)

    with open(target_config_file) as fp:
        return json.load(fp)


def generate_kobject_files():
    gen_kobject_list_cmd = [
        "$PYTHONEXE",
        '"%s"' % join(FRAMEWORK_DIR, "scripts", "gen_kobject_list.py"),
        "--kobj-types-output",
        "${TARGETS[0]}",
        "--kobj-otype-output",
        "${TARGETS[1]}",
        "--kobj-size-output",
        "${TARGETS[2]}",
    ]

    return env.Command(
        [
            join("$BUILD_DIR", "zephyr", "include", "generated", "kobj-types-enum.h"),
            join("$BUILD_DIR", "zephyr", "include", "generated", "otype-to-str.h"),
            join("$BUILD_DIR", "zephyr", "include", "generated", "otype-to-size.h"),
        ],
        None,
        env.VerboseAction(
            " ".join(gen_kobject_list_cmd), "Generating KObject files $TARGETS"
        ),
    )


def validate_driver_cmd():

    validate_cmd = [
        "$PYTHONEXE",
        '"%s"' % join(FRAMEWORK_DIR, "scripts", "gen_kobject_list.py"),
        "--validation-output",
        "$TARGET",
    ]

    return env.Command(
        join("$BUILD_DIR", "zephyr", "include", "generated", "driver-validation.h"),
        None,
        env.VerboseAction(" ".join(validate_cmd), "Validating drivers $TARGET"),
    )


def generate_syscall_macro_header():
    gen_syscall_header_cmd = [
        "$PYTHONEXE",
        '"%s"' % join(FRAMEWORK_DIR, "scripts", "gen_syscall_header.py"),
        ">",
        "$TARGET",
    ]

    return env.Command(
        join("$BUILD_DIR", "zephyr", "include", "generated", "syscall_macros.h"),
        None,
        env.VerboseAction(
            " ".join(gen_syscall_header_cmd), "Generating syscall macro header $TARGET"
        ),
    )


def parse_syscalls():
    parse_syscalls_cmd = [
        "$PYTHONEXE",
        '"%s"' % join(FRAMEWORK_DIR, "scripts", "parse_syscalls.py"),
        "--include",
        '"%s"' % join(FRAMEWORK_DIR, "include"),
        "--json-file",
        "$TARGET",
    ]

    return env.Command(
        join("$BUILD_DIR", "zephyr", "include", "generated", "syscalls.json"),
        None,
        env.VerboseAction(
            " ".join(parse_syscalls_cmd), "Parsing system calls from $TARGET"
        ),
    )


def generate_syscall_files(syscalls_json):
    gen_syscalls_cmd = [
        "$PYTHONEXE",
        '"%s"' % join(FRAMEWORK_DIR, "scripts", "gen_syscalls.py"),
        "--json-file",
        "$SOURCE",
        "--base-output",
        join("$BUILD_DIR", "zephyr", "include", "generated", "syscalls"),
        "--syscall-dispatch",
        "${TARGETS[0]}",
        "--syscall-list",
        "${TARGETS[1]}",
    ]

    return env.Command(
        [
            join(BUILD_DIR, "zephyr", "include", "generated", "syscall_dispatch.c"),
            join(BUILD_DIR, "zephyr", "include", "generated", "syscall_list.h"),
            join(
                "$BUILD_DIR",
                "zephyr",
                "include",
                "generated",
                "syscalls",
                "errno_private.h",
            ),
            join(
                "$BUILD_DIR", "zephyr", "include", "generated", "syscalls", "atomic.h"
            ),
            join(
                "$BUILD_DIR", "zephyr", "include", "generated", "syscalls", "device.h"
            ),
            join(
                "$BUILD_DIR", "zephyr", "include", "generated", "syscalls", "kernel.h"
            ),
            join(
                "$BUILD_DIR",
                "zephyr",
                "include",
                "generated",
                "syscalls",
                "sys_clock.h",
            ),
        ],
        syscalls_json,
        env.VerboseAction(" ".join(gen_syscalls_cmd), "Generating syscall files"),
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
    ldscript_cmd = [
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

    ldscript_cmd.extend(['-I"%s"' % inc for inc in app_includes])

    return env.Command(
        join("$BUILD_DIR", "zephyr", "linker_pass_final.cmd"),
        base_ld_script,
        env.VerboseAction(
            " ".join(ldscript_cmd), "Generating final linker script $TARGET"
        ),
    )


def find_base_ldscript(app_includes):
    # A temporary solution since there is no an easy way to find linker script
    for inc in app_includes:
        for f in listdir(inc):
            if f == "linker.ld" and isfile(join(inc, f)):
                return join(inc, f)

    sys.stderr.write("Error: Couldn't find a base linker script!\n")
    env.Exit(1)


def get_linkerscript_cmd(app_includes, base_ld_script):
    ldscript_cmd = [
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

    ldscript_cmd.extend(['-I"%s"' % inc for inc in app_includes])

    return env.Command(
        join("$BUILD_DIR", "zephyr", "linker.cmd"),
        base_ld_script,
        env.VerboseAction(" ".join(ldscript_cmd), "Generating linker script $TARGET"),
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
        includes = [inc.get("path") for inc in cg.get("includes", [])]
        defines = [inc.get("define") for inc in cg.get("defines", [])]

        compile_commands = cg.get("compileCommandFragments", [])
        for cc in compile_commands:
            build_env = env.Clone()
            build_flags = cc.get("fragment")
            build_env.AppendUnique(**build_env.ParseFlags(build_flags))
            build_env.Append(
                ASFLAGS=build_env.get("CCFLAGS", [])[:],
                CPPDEFINES=defines,
                CPPPATH=includes,
            )
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
        compile_group_idx = source.get("compileGroupIndex")
        if compile_group_idx is not None:
            objects.append(
                build_envs[compile_group_idx].StaticObject(
                    target=join(
                        "$BUILD_DIR",
                        "zephyr",
                        config_name,
                        basename(dirname(source.get("path"))),
                        basename(source.get("path")) + ".o",
                    ),
                    source=abspath(source.get("path")),
                )
            )

    return objects


def get_app_includes(app_config):
    return [
        inc.get("path") for inc in app_config["compileGroups"][0].get("includes", [])
    ]


def get_app_defines(app_config):
    return [
        inc.get("define") for inc in app_config["compileGroups"][0].get("defines", [])
    ]


def get_firmware_flags(app_config, main_config):
    # Use the first compile commands group
    app_flags = {}
    for cg in app_config["compileGroups"]:
        app_flags.update(env.ParseFlags(cg["compileCommandFragments"][0]["fragment"]))

    # Ignore ld script flags as they'll be specified in a special place
    app_flags["LINKFLAGS"] = [
        f
        for f in extract_link_flags(main_config)
        if not f.endswith(".cmd") and f != "-T" and f != "-lgcc"
    ]

    return app_flags


def generate_offset_header_file(source, target, env):
    print("Generating header file with offsets")
    for f in source:
        filepath = f.get_abspath()
        if not filepath.endswith("offsets.c.o"):
            continue

        gen_offsets_header_cmd = [
            "$PYTHONEXE",
            '"%s"' % join(FRAMEWORK_DIR, "scripts", "gen_offset_header.py"),
            "-i",
            filepath,
            "-o",
            join(BUILD_DIR, "zephyr", "include", "generated", "offsets.h"),
        ]

        env.Execute(" ".join(gen_offsets_header_cmd))


def generate_isr_list_binary(preliminary_elf, board, platform):
    generate_isr_binary_cmd = [
        "$OBJCOPY",
        "--input-target=" + get_target_elf_arch(board, platform),
        "--output-target=binary",
        "--only-section=.intList",
        "$SOURCE",
        "$TARGET",
    ]

    return env.Command(
        join("$BUILD_DIR", "zephyr", "isrList.bin"),
        preliminary_elf,
        env.VerboseAction(
            " ".join(generate_isr_binary_cmd), "Generating ISR list $TARGET"
        ),
    )


def generate_isr_table_file_cmd(preliminary_elf, board_config, platform):
    generate_isr_src_file_cmd = [
        "$PYTHONEXE",
        '"%s"' % join(FRAMEWORK_DIR, "arch", "common", "gen_isr_tables.py"),
        "--output-source",
        "$TARGET",
        "--kernel",
        "${SOURCES[0]}",
        "--intlist",
        "${SOURCES[1]}",
        "--debug",
        "--sw-isr-table",
        "--vector-table",
    ]

    cmd = env.Command(
        join("$BUILD_DIR", "zephyr", "isr_tables.c"),
        [preliminary_elf, join("$BUILD_DIR", "zephyr", "isrList.bin")],
        env.VerboseAction(
            " ".join(generate_isr_src_file_cmd), "Generating ISR table $TARGET"
        ),
    )

    env.Requires(cmd, generate_isr_list_binary(preliminary_elf, board_config, platform))

    return cmd


def build_offsets_lib(target_configs):
    lib = build_library(target_configs["offsets"])
    env.AddPostAction(lib, generate_offset_header_file)
    env.SideEffect(join(BUILD_DIR, "zephyr", "include", "generated", "offsets.h"), lib)

    return lib


env.EnsurePythonVersion(3, 4)

if " " in FRAMEWORK_DIR:
    sys.stderr.write("Error: Detected a whitespace character in framework path\n")
    env.Exit(1)

codemodel = get_cmake_code_model()
if not codemodel:
    sys.stderr.write("Error: Couldn't find code model generated by CMake\n")
    env.Exit(1)

target_configs = load_target_configurations(codemodel)

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


app_config = target_configs.get("app")
prebuilt_config = target_configs.get("zephyr_prebuilt")

if not app_config or not prebuilt_config:
    sys.stderr.write("Error: Couldn't find main Zephyr target in the code model\n")
    env.Exit(1)

app_includes = get_app_includes(app_config)
base_ld_script = find_base_ldscript(app_includes)

final_ld_script = get_linkerscript_final_cmd(app_includes, base_ld_script)
preliminary_ld_script = get_linkerscript_cmd(app_includes, base_ld_script)

offsets_lib = build_offsets_lib(target_configs)
add_ordered_dependency(final_ld_script, offsets_lib)
add_ordered_dependency(preliminary_ld_script, offsets_lib)

preliminary_elf_path = join("$BUILD_DIR", "firmware-pre.elf")
env.Depends(preliminary_elf_path, join(BUILD_DIR, "libkernel.a"))

pre_elf_targets = (
    generate_syscall_files(parse_syscalls()),
    generate_syscall_macro_header(),
    generate_kobject_files(),
    preliminary_ld_script,
)

for dep in pre_elf_targets:
    add_ordered_dependency(preliminary_elf_path, dep)

final_elf_targets = (
    final_ld_script,
    generate_isr_table_file_cmd(preliminary_elf_path, board, env.subst("$PIOPLATFORM")),
    compile_source_files(target_configs["zephyr_final"]),
)

for dep in final_elf_targets:
    add_ordered_dependency("$PROG_PATH", dep)

libs = [
    framework_libs_map[d["id"]]
    for d in prebuilt_config.get("dependencies", [])
    if framework_libs_map.get(d["id"], {}) and not d["id"].startswith(("kernel", "app"))
]

libs.append(offsets_lib)

env.Replace(ARFLAGS=["qc"])
env.Prepend(_LIBFLAGS="-Wl,--whole-archive ")

# Note: libc and kernel libraries must be placed explicitly after zephyr libraries
# outside of whole-archive flag
env.Append(
    PIOBUILDFILES=compile_source_files(prebuilt_config),
    CPPPATH=app_includes,
    CPPDEFINES=get_app_defines(app_config),
    LIBS=sorted(libs),
    _LIBFLAGS=" -Wl,--no-whole-archive -lkernel -lgcc",
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

if board.get("build.cpu", "").lower().startswith("cortex"):
    env.Replace(
        SIZEPROGREGEXP=r"^(?:text|_TEXT_SECTION_NAME_2|sw_isr_table|devconfig|devconfig|rodata|\.ARM.exidx)\s+(\d+).*",
        SIZEDATAREGEXP=r"^(?:datas|bss|noinit|initlevel|_k_mutex_area|_k_stack_area)\s+(\d+).*",
    )
