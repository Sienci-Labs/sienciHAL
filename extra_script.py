Import("env")

env.Replace(PROGNAME="%s" % env.GetProjectOption("custom_prog_version")+"_"+env.GetProjectOption("grblhal_driver_version"))
env.Replace(custom_board_name="%s" % env.GetProjectOption("custom_prog_version")+"_"+env.GetProjectOption("grblhal_driver_version"))

# Custom HEX from ELF
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex", "-R", ".eeprom", 
        '"$BUILD_DIR/${PROGNAME}.elf"', '"$BUILD_DIR/${PROGNAME}.hex"'
    ]), "Building $BUILD_DIR/${PROGNAME}.hex")
)

