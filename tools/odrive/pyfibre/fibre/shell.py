
import sys
import platform
import threading
import fibre

async def discovered_device(device,
                        interactive_variables, discovered_devices,
                        mount, shutdown_token, logger):
    """
    Handles the discovery of new devices by displaying a
    message and making the device available to the interactive
    console
    """
    mount_result = await mount(device)
    if mount_result is None:
        logger.debug("ignoring device")
        return

    display_name, var_name = mount_result

    if display_name in discovered_devices:
        verb = "Reconnected"
        index = discovered_devices.index(display_name)
    else:
        verb = "Connected"
        discovered_devices.append(display_name)
        index = len(discovered_devices) - 1

    var_name = var_name + str(index)

    # Publish new device to interactive console
    interactive_variables[var_name] = device
    globals()[var_name] = device # Add to globals so tab complete works
    logger.notify("{} to {} as {}".format(verb, display_name, var_name))

    # Subscribe to disappearance of the device
    device._on_lost.add_done_callback(lambda x: lost_device(var_name, shutdown_token, logger))

def lost_device(interactive_name, shutdown_token, logger):
    """
    Handles the disappearance of a device by displaying
    a message.
    """
    if not shutdown_token[0]:
        logger.warn("Oh no {} disappeared".format(interactive_name))

def get_user_name(interactive_variables, obj):
    queue = [(k, v) for k, v in interactive_variables.items() if isinstance(v, fibre.libfibre.RemoteObject)]

    if not isinstance(obj, fibre.libfibre.RemoteObject):
        return None

    while len(queue):
        k, v = queue.pop(0)
        if v == obj:
            return k
        for key in dir(v.__class__):
            class_member = getattr(v.__class__, key)
            if not key.startswith('_') and isinstance(class_member, fibre.libfibre.RemoteAttribute):
                queue.append((k + "." + (key if not class_member._magic_getter else "_" + key + "_property"), class_member._get_obj(v)))

    return "anonymous_remote_object_" + str(self._obj_handle)

def launch_shell(args, mount,
                interactive_variables,
                print_banner, print_help,
                logger):
    """
    Launches an interactive python or IPython command line
    interface.
    As devices are connected they are made available as
    "dev0", "dev1", ...
    The names of the variables can be customized by setting branding_short.
    """

    discovered_devices = []
    shutdown_token = [False]
    globals().update(interactive_variables)

    fibre.libfibre.get_user_name = lambda obj: get_user_name(interactive_variables, obj)

    # Connect to device
    with fibre.Domain(args.path) as domain:
        on_discovery = lambda dev: discovered_device(dev, interactive_variables, discovered_devices, mount, shutdown_token, logger)
        discovery = domain.run_discovery(on_discovery)

        # Check if IPython is installed
        if args.no_ipython:
            use_ipython = False
        else:
            try:
                import IPython
                use_ipython = True
            except:
                print("Warning: you don't have IPython installed.")
                print("If you want to have an improved interactive console with pretty colors,")
                print("you should install IPython\n")
                use_ipython = False

        interactive_variables["help"] = lambda: print_help(args, len(discovered_devices) > 0)

        # If IPython is installed, embed IPython shell, otherwise embed regular shell
        if use_ipython:
            # Override help function # pylint: disable=W0612
            help = lambda: print_help(args, len(discovered_devices) > 0) 
            # to fix broken "%run -i script.py"
            locals()['__name__'] = globals()['__name__'] 
            console = IPython.terminal.embed.InteractiveShellEmbed(banner1='')

            # hack to make IPython look like the regular console
            console.runcode = console.run_cell 
            interact = console

            # Catch ObjectLostError (since disconnect is not always an error)
            default_exception_hook = console._showtraceback
            def filtered_exception_hook(ex_class, ex, trace):
                if(ex_class.__module__+'.'+ex_class.__name__ != 'fibre.libfibre.ObjectLostError'):
                    default_exception_hook(ex_class,ex,trace)
                
            console._showtraceback = filtered_exception_hook
        else:
            # Enable tab complete if possible
            try:
                import readline # Works only on Unix
                readline.parse_and_bind("tab: complete")
            except:
                sudo_prefix = "" if platform.system() == "Windows" else "sudo "
                print("Warning: could not enable tab-complete. User experience will suffer.\n"
                    "Run `{}pip install readline` and then restart this script to fix this."
                    .format(sudo_prefix))

            import code
            console = code.InteractiveConsole(locals=interactive_variables)
            interact = lambda: console.interact(banner='')

            # Catch ObjectLostError (since disconnect is not alway an error)
            console.runcode("import sys")
            console.runcode("default_exception_hook = sys.excepthook")
            console.runcode("def filtered_exception_hook(ex_class, ex, trace):\n"
                            "  if ex_class.__module__ + '.' + ex_class.__name__ != 'fibre.libfibre.ObjectLostError':\n"
                            "    default_exception_hook(ex_class,ex,trace)")
            console.runcode("sys.excepthook=filtered_exception_hook")

        # Launch shell
        print_banner()
        logger._skip_bottom_line = True
        interact()

        shutdown_token[0] = True
        discovery.stop()
