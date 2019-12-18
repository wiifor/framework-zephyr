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

import os

from SCons import Util
from SCons.Script import AlwaysBuild

from platformio import VERSION
from platformio.util import pioversion_to_intstr


Import("env")


def ZephyrBuildProgram(env):
    env["LDSCRIPT_PATH"] = None
    env.ProcessProgramDeps()
    env.ProcessProjectDeps()

    # append into the beginning a main LD script
    env.Prepend(LINKFLAGS=["-T", "$LDSCRIPT_PATH"])

    # enable "cyclic reference" for linker
    if env.get("LIBS") and env.GetCompilerType() == "gcc":
        env.Prepend(_LIBFLAGS="-Wl,--start-group ")
        env.Append(_LIBFLAGS=" -Wl,--end-group")

    program_pre = env.Program(
        os.path.join("$BUILD_DIR", "firmware-pre"), env["PIOBUILDFILES"],
        LDSCRIPT_PATH=os.path.join("$BUILD_DIR", "zephyr", "linker.cmd")
    )
    
    program = env.Program(
        os.path.join("$BUILD_DIR", env.subst("$PROGNAME")),
        env["PIOBUILDFILES"] + env["_EXTRA_ZEPHYR_PIOBUILDFILES"],
        LDSCRIPT_PATH=os.path.join("$BUILD_DIR", "zephyr", "linker_pass_final.cmd")
    )

    env.Replace(PIOMAINPROG=program)

    AlwaysBuild(
        env.Alias(
            "checkprogsize",
            program,
            env.VerboseAction(env.CheckUploadSize, "Checking size $PIOMAINPROG"),
        )
    )

    print("Building in %s mode" % env.GetBuildType())

    return program


def LegacyZephyrBuildProgram(env):
    def _append_pio_macros():
        env.AppendUnique(
            CPPDEFINES=[
                (
                    "PLATFORMIO",
                    int("{0:02d}{1:02d}{2:02d}".format(*pioversion_to_intstr())),
                )
            ]
        )

    _append_pio_macros()

    env.PrintConfiguration()

    # fix ASM handling under non case-sensitive OS
    if not Util.case_sensitive_suffixes(".s", ".S"):
        env.Replace(AS="$CC", ASCOM="$ASPPCOM")

    # process extra flags from board
    if "BOARD" in env and "build.extra_flags" in env.BoardConfig():
        env.ProcessFlags(env.BoardConfig().get("build.extra_flags"))

    # apply user flags
    env.ProcessFlags(env.get("BUILD_FLAGS"))

    # process framework scripts
    env.BuildFrameworks(env.get("PIOFRAMEWORK"))

    is_build_type_debug = (
        set(["debug", "sizedata"]) & set(COMMAND_LINE_TARGETS)
        or env.GetProjectOption("build_type") == "debug"
    )
    if is_build_type_debug:
        env.ConfigureDebugFlags()

    # remove specified flags
    env.ProcessUnFlags(env.get("BUILD_UNFLAGS"))

    if "__test" in COMMAND_LINE_TARGETS:
        env.ConfigureTestTarget()

    # enable "cyclic reference" for linker
    if env.get("LIBS") and env.GetCompilerType() == "gcc":
        env.Prepend(_LIBFLAGS="-Wl,--start-group ")
        env.Append(_LIBFLAGS=" -Wl,--end-group")


    # build project with dependencies
    from platformio.builder.tools.platformio import _build_project_deps
    _build_project_deps(env)

    # append into the beginning a main LD script
    env.Prepend(LINKFLAGS=["-T", "$LDSCRIPT_PATH"])

    program_pre = env.Program(
        os.path.join("$BUILD_DIR", "firmware-pre"), env["PIOBUILDFILES"],
        LDSCRIPT_PATH=os.path.join("$BUILD_DIR", "zephyr", "linker.cmd")
    )

    program = env.Program(
        os.path.join("$BUILD_DIR", env.subst("$PROGNAME")),
        env["PIOBUILDFILES"] + env["_EXTRA_ZEPHYR_PIOBUILDFILES"],
        LDSCRIPT_PATH=os.path.join("$BUILD_DIR", "zephyr", "linker_pass_final.cmd")
    )

    env.Replace(PIOMAINPROG=program)

    AlwaysBuild(
        env.Alias(
            "checkprogsize",
            program,
            env.VerboseAction(env.CheckUploadSize, "Checking size $PIOMAINPROG"),
        )
    )

    print("Building in %s mode" % ("debug" if is_build_type_debug else "release"))

    return program


if pioversion_to_intstr() < [4, 1, 1]:
    env.AddMethod(LegacyZephyrBuildProgram, "BuildProgram")
else:
    env.AddMethod(ZephyrBuildProgram, "BuildProgram")
