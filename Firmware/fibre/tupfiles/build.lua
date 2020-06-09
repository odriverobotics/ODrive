

function GCCToolchain(prefix, builddir, compiler_flags, linker_flags)

    -- add some default compiler flags
    compiler_flags += '-fstack-usage'

    local gcc_generic_compiler = function(compiler, compiler_flags, gen_su_file, src, flags, includes, outputs)
        -- resolve source path
        src = tostring(src)

        -- convert include list to flags
        inc_flags = {}
        for _,inc in pairs(includes) do
            inc_flags += "-I"..tostring(inc)
        end

        obj_file = builddir.."/"..src:gsub("/","_")..".o"
        outputs.object_files += obj_file
        if gen_su_file then
            su_file = builddir.."/"..src:gsub("/","_")..".su"
            extra_outputs = { su_file }
            outputs.su_files += su_file
        else
            extra_outputs = {}
        end
        tup.frule{
            inputs= { tup.getcwd()..'/'..src },
            command=compiler..' -c %f '..
                    tostring(compiler_flags)..' '.. -- CFLAGS for this compiler
                    tostring(inc_flags)..' '.. -- CFLAGS for this translation unit
                    tostring(flags).. -- CFLAGS for this translation unit
                    ' -o %o',
            outputs={obj_file,extra_outputs=extra_outputs}
        }
    end
    return {
        compile_c = function(src, flags, includes, outputs) gcc_generic_compiler(prefix..'gcc -std=c99', compiler_flags, true, src, flags, includes, outputs) end,
        compile_cpp = function(src, flags, includes, outputs) gcc_generic_compiler(prefix..'g++ -std=c++11', compiler_flags, true, src, flags, includes, outputs) end,
        compile_asm = function(src, flags, includes, outputs) gcc_generic_compiler(prefix..'gcc -x assembler-with-cpp', compiler_flags, false, src, flags, includes, outputs) end,
        link = function(objects, libs, output_name)
            -- convert lib list to flags
            lib_flags = {}
            for _,inc in pairs(libs) do
                lib_flags += "-l"..tostring(inc)
            end

            output_name = builddir..'/'..output_name
            tup.frule{
                inputs=objects,
                command=prefix..'g++ %f '..
                        tostring(linker_flags)..' '..
                        tostring(lib_flags)..' '..
                        '-Wl,-Map=%O.map'..
                        ' -o %o',
                outputs={output_name..'.elf', extra_outputs={output_name..'.map'}}
            }
            -- display the size
            tup.frule{inputs={output_name..'.elf'}, command=prefix..'size %f'}
            -- create *.hex and *.bin output formats
            tup.frule{inputs={output_name..'.elf'}, command=prefix..'objcopy -O ihex %f %o', outputs={output_name..'.hex'}}
            tup.frule{inputs={output_name..'.elf'}, command=prefix..'objcopy -O binary -S %f %o', outputs={output_name..'.bin'}}
        end
    }
end


function LLVMToolchain(arch, compiler_flags, linker_flags)

    -- add some default compiler flags
    --compiler_flags += '-march='..arch
    compiler_flags += '-std=c++14'
    
    clang_generic_compiler = function(compiler, compiler_flags, src, flags, includes, outputs)
        -- add includes to CFLAGS
        for _,inc in pairs(includes) do
            flags += "-I"..inc
        end
        -- todo: vary build directory
        obj_file="build/"..src:gsub("/","_")..".o"
        tup.frule{
            inputs=src,
            command=compiler..' -c %f '..
                    tostring(compiler_flags)..' '.. -- CFLAGS for this compiler
                    tostring(flags).. -- CFLAGS for this translation unit
                    ' -o %o',
            outputs={obj_file}
        }
        outputs.object_files += obj_file
    end
    return {
        compile_c = function(src, flags, includes, outputs) clang_generic_compiler('clang', compiler_flags, src, flags, includes, outputs) end,
        compile_cpp = function(src, flags, includes, outputs) clang_generic_compiler('clang++', compiler_flags, src, flags, includes, outputs) end,
        link = function(objects, output_name)
            tup.frule{
                inputs=objects,
                command='clang++ %f '..
                        tostring(linker_flags)..
                        ' -o %o',
                outputs=output_name
            }
        end
    }
end


function get_generalized_paths(paths)
    if paths == nil then
        return {}
    else
        -- TODO: check for string
        generalized_paths = {}
        for _,path in pairs(paths) do
            table.insert(generalized_paths, tup.nodevariable(path))
        end
        return generalized_paths
    end
end


-- A package is a collection of source files and associated
-- information required to compile those source files.
--  pkg.sources:    The source files that shall be compiled as
--                  part of this package
--  pkg.private_headers: The include directories that are required
--                  to compile the source files in this package
--  pkg.headers:    The include directories that are _exported_
--                  by this package. These directories are included
--                  when compiling other packages that import
--                  this package.
--  pkg.packages:   The packages that are needed to compile and link this
--                  package. The public include directories of each imported
--                  package are passed to the compiler when compiling
--                  the source files of this package. The object files
--                  emitted by the imported packages are included when linking
--                  this package.
--  pkg.libs:       The libraries that are needed to link this
--                  package
function define_package(pkg)
    --print('defined package in '..tup.getcwd())
    pkg.sources = get_generalized_paths(pkg.sources)
    pkg.objects = get_generalized_paths(pkg.objects)
    pkg.headers = get_generalized_paths(pkg.headers)
    pkg.private_headers = get_generalized_paths(private_headers)
    if pkg.packages == nil then pkg.packages = {} end
    if pkg.libs == nil then pkg.libs = {} end
    if pkg.c_flags == nil then pkg.c_flags = {} end
    if pkg.cpp_flags == nil then pkg.cpp_flags = {} end
    if pkg.asm_flags == nil then pkg.asm_flags = {} end
    return pkg
end


-- Builds object files from the source files in the specified package
function build_objects(pkg, toolchain)
    all_headers = {}
    tup.append_table(all_headers, pkg.private_headers)
    tup.append_table(all_headers, pkg.headers)

    -- add exported header directories of each imported package
    for _,imported_pkg in pairs(pkg.packages) do
        tup.append_table(all_headers, imported_pkg.headers)
    end

    -- compile
    outputs = {
        object_files = {}
    }
    tup.append_table(outputs.object_files, pkg.objects)

    for _,src in pairs(pkg.sources) do
        --print("compile "..src)
        ext = tup.ext(tostring(src))
        if ext == 'c' then
            toolchain.compile_c(src, pkg.c_flags, all_headers, outputs)
        elseif ext == 'cpp' then
            toolchain.compile_cpp(src, pkg.cpp_flags, all_headers, outputs)
        elseif ext == 's' or tup.ext(src) == 'asm' then
            toolchain.compile_asm(src, pkg.asm_flags, all_headers, outputs)
        else
            error('unrecognized file ending')
        end
    end

    return outputs.object_files
end

function build_executable(name, pkg, toolchain)
    all_object_files = {}
    all_libs = {}

    -- TODO: flatten the import hierarchy prior to compiling

    -- build current package
    tup.append_table(all_object_files, build_objects(pkg, toolchain))
    tup.append_table(all_libs, pkg.libs)

    -- build imported packages
    for _,imported_pkg in pairs(pkg.packages) do
        objects = build_objects(imported_pkg, toolchain)
        tup.append_table(all_object_files, objects)
        tup.append_table(all_libs, imported_pkg.libs)
    end

    -- link
    --tup.append_table(args.linker_objects, outputs.object_files)
    print('link objects ')
    print(all_object_files)
    print(all_libs)
    toolchain.link(all_object_files, all_libs, name)
end
