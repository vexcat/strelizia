# SuperHot Loader

This project is far from complete. As of now, SuperHot can be built & uploaded with your program by executing `superhot`, and patches can be sent with `superpatch`, with behavior not quite inline with what this .md says. Think of this page as the *goals* for the project rather than what it can actually do.

The SuperHot Loader is a program launcher for VEX V5. It receives patches to your program over serial input, then stops your program and reloads it with the changes applied. In more advanced usage, it reduces patches to just a few bytes and can be used to selectively update parts of the program while leaving everything else running. All while avoiding the main caveat of normal Hot/Cold linking, where the cold section can't directly make references to the hot section. (aside from PROS' internal HOT_TABLE, otherwise cold wouldn't be able to reference a hot opcontrol/auto/init)

## Usage

When installed correctly, you will have a new `superhot` command in your path. This uses the same syntax as `prosv5 upload`, which is called internally. What's different is that executing this will rebuild the current project with SuperHot's cold section instead of the normal one. (This works by compiling your cold libraries and cold objects with our custom v5.ld, and compiling your hot code with our custom v5-hot.ld.) The daemon continues running, automatically building & uploading as files in the `src/` change. Because SuperHot's Makefile makes use of ccache, there is no need to do anything special when you edit a header file. The dependent C++ source files will be automatically rebuilt.

There are three main caveats to the usage of SuperHot.

  - The terminal/serial is reserved for tabu messages (see vexcat/strelizia and vexcat/tabicat). This mechanism is used to send patches, with SuperHot listening on tabu topic "patch". SuperHot prints two fake ptys when you run it, the first being "Direct I/O", and the second being "Tabu Passthrough". If you don't want to use tabu in your code, then use Direct I/O and call `recv_ser_char()`/`xmit_ser_char()` to get/send a character in your code. These functions are line-buffered and build on top of the tabu mechanism (topic "line"). "Tabu Passthrough" goes directly to the serial when SuperHot isn't busy with it. This is meant for a program using tabu.js (vexcat/tabicat) to communicate with.
  - All task creations must call a `superhot_create_task` macro defined in superhot_compat.hpp, which takes the same arguments as `pros::Task()`. This macro is completely harmless outside of `superhot`, compiling to `pros::Task()` directly if `SUPERHOT_ENABLED` isn't defined (a normal, non-superhot build). Tasks not added by this method will not be stopped on patch receival. If you have additional program destruction, like removing things from the heap, then handle them in a global destructor.
  - Build management is handled entirely by SuperHot. Changes to the Makefile will not change anything in your project build. If you want to change how your project builds, edit the Makefile in the superhot directory and be careful to only change what you need to. One key setting you may have to change is the `-j` parameter in superhot/build.sh, which should be changed to the number of cores on your CPU for optimal build performance. Do not attempt to use `make -j4 all`, this is a race condition as one thread will delete files while others create files.

## Advanced Usage

### Modules

Even with SuperHot, your project might just be too *thicc* to upload quickly. If this is the case, you can use SuperHot Modules. Normally, compiled code is packed together. Functions jammed side by side in a binary. This causes issues when you're trying to make a patch, because the addresses of all the functions change! This limits the effectiveness of patching. To get around this, you can specify a list of modules.

Here is an example modules.json file, which compiles into a hot linker script.

```
{
  "autonomous": {
    "weight": 1,
    "objects": ["autonomous.cpp.o", "auto_util.cpp.o"]
  },
  "opcontrol": {
    "weight": 1,
    "objects": ["opcontrol.cpp.o"]
  },
  "default": {
    "weight": 8
  }
}
```

As you can see, there are three modules defined. One for autonomous, one for operator control, and one for everything else. The "weight" is how much you expect the module to contribute to your program's size. As only a total of 16MiB is given for you to use, 1/(1+1+8) of that is being allocated for auto & opcontrol (1.6MiB), and 8/(1+1+8) of that is being allocated for everything else (12.8MiB). If your code for a module is too big for the given weight, you'll get an error like `section '.text.autonomous_MEMORY' will not fit in region 'autonomous_MEMORY'` during the link step. Non-integer values are allowed.
 

# Disclaimer

8301E is not responsible for any instability caused in your program by use of this launcher. We do not claim that this program is perfect or that it is competition-ready, and *we strongly recommend to not use it while competing*.