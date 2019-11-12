import os
from SCons.Script import AlwaysBuild


Import("env")

def ZephyrBuildProgram(env):
    env["LDSCRIPT_PATH"] = None
    env.ProcessProgramDeps()
    env.ProcessProjectDeps()

    # append into the beginning a main LD script
    env.Prepend(LINKFLAGS=["-T", "$LDSCRIPT_PATH"])

    program_pre = env.Program(
        os.path.join("$BUILD_DIR", "firmware-pre"), env["PIOBUILDFILES"],
        LDSCRIPT_PATH=os.path.join("$BUILD_DIR", "zephyr", "linker.cmd")
    )
    
    program = env.Program(
        os.path.join("$BUILD_DIR", env.subst("$PROGNAME")),
        env["PIOBUILDFILES"] + [os.path.join("$BUILD_DIR", "zephyr", "zephyr_final", "zephyr", "isr_tables.c.o")],
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

env.AddMethod(ZephyrBuildProgram, "BuildProgram")
