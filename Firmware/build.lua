
-- This file contains support functions for Tupfile.lua

function trim(s)
    return (s:gsub("^%s*(.-)%s*$", "%1"))
end

function string:split(sep)
    local sep, fields = sep or ":", {}
    local pattern = string.format("([^%s]+)", sep)
    self:gsub(pattern, function(c) fields[#fields+1] = c end)
    return fields
end

-- Very basic parser to retrieve variables from a Makefile
function parse_makefile_vars(makefile)
    vars = {}
    current_var = nil
    for line in io.lines(tup.getcwd()..'/'..makefile) do
        if current_var == nil then
            i,j = string.find(line, "+=")
            if not i then
                i,j = string.find(line, "=")
            end
            if i then
                current_var = trim(string.sub(line, 1, i-1))
                vars[current_var] = vars[current_var] or ''
                line = string.sub(line, j+1, -1)
                --print("varname: "..varname.." the rest: "..line)
            end
        end

        if current_var != nil then
            --print("append chunk "..trim(line).." to "..current_var)
            vars[current_var] = vars[current_var]..' '..trim(line)
            if string.sub(vars[current_var], -1) == '\\' then
                vars[current_var] = string.sub(vars[current_var], 1, -2)
            else
                current_var = nil
            end
        end
    end
    return vars
end




function GCCToolchain(prefix, builddir, compiler_flags, linker_flags)

    -- add some default compiler flags
    -- -fstack-usage gives a warning for some functions containing inline assembly (prvPortStartFirstTask in particular)
    -- so for now we just disable it
    calculate_stack_usage = false
    if calculate_stack_usage then
        compiler_flags += '-fstack-usage'
    end

    local gcc_generic_compiler = function(compiler, compiler_flags, gen_su_file, src, flags, includes, outputs)
        -- convert include list to flags
        inc_flags = {}
        for _,inc in pairs(includes) do
            inc_flags += "-I"..inc
        end
        -- todo: vary build directory
        obj_file = builddir.."/obj/"..src:gsub("/","_")..".o"
        outputs.object_files += obj_file
        if gen_su_file then
            su_file = builddir.."/"..src:gsub("/","_")..".su"
            extra_outputs = { su_file }
            outputs.su_files += su_file
        else
            extra_outputs = {}
        end
        if src == 'communication/communication.cpp' then extra_inputs = 'build/version.h' end -- TODO: fix hack
        tup.frule{
            inputs= { src, extra_inputs=extra_inputs },
            command=compiler..' -c %f '..
                    tostring(compiler_flags)..' '.. -- CFLAGS for this compiler
                    tostring(inc_flags)..' '.. -- CFLAGS for this translation unit
                    tostring(flags).. -- CFLAGS for this translation unit
                    ' -o %o',
            outputs={obj_file,extra_outputs=extra_outputs}
        }
    end
    return {
        compile_c = function(src, flags, includes, outputs) gcc_generic_compiler(prefix..'gcc -std=c99', compiler_flags, calculate_stack_usage, src, flags, includes, outputs) end,
        compile_cpp = function(src, flags, includes, outputs) gcc_generic_compiler(prefix..'g++ -std=c++14', compiler_flags, calculate_stack_usage, src, flags, includes, outputs) end,
        compile_asm = function(src, flags, includes, outputs) gcc_generic_compiler(prefix..'gcc -x assembler-with-cpp', compiler_flags, false, src, flags, includes, outputs) end,
        link = function(objects, output_name)
            output_name = builddir..'/'..output_name
            tup.frule{
                inputs=objects,
                command=prefix..'g++ %f '..
                        tostring(linker_flags)..' '..
                        '-Wl,-Map=%O.map'..
                        ' -o %o',
                outputs={output_name..'.elf', extra_outputs={output_name..'.map'}}
            }
            -- display the size
            tup.frule{inputs={output_name..'.elf'}, command=prefix..'size %f'}
            -- generate disassembly
            tup.frule{inputs={output_name..'.elf'}, command=prefix..'objdump %f -dSC > %o', outputs={output_name..'.asm'}}
            -- create *.hex and *.bin output formats
            tup.frule{inputs={output_name..'.elf'}, command=prefix..'objcopy -O ihex %f %o', outputs={output_name..'.hex'}}
            tup.frule{inputs={output_name..'.elf'}, command=prefix..'objcopy -O binary -S %f %o', outputs={output_name..'.bin'}}
        end
    }
end

all_packages = {}

-- toolchains: Each element of this list is a collection of functions, such as compile_c, link, ...
--              You can create a new toolchain object for each platform you want to build for.
function build(args)
    if args.toolchain == nil then args.toolchain = {} end
    if args.sources == nil then args.sources = {} end
    if args.includes == nil then args.includes = {} end
    if args.packages == nil then args.packages = {} end
    if args.c_flags == nil then args.c_flags = {} end
    if args.cpp_flags == nil then args.cpp_flags = {} end
    if args.asm_flags == nil then args.asm_flags = {} end
    if args.ld_flags == nil then args.ld_flags = {} end
    if args.linker_objects == nil then args.linker_objects = {} end
    
    -- add includes of other packages
    for _,pkg_name in pairs(args.packages) do
        --print('depend on package '..pkg_name)
        pkg = all_packages[pkg_name]
        if pkg == nil then
            error("unknown package "..pkg_name)
        end
        -- add path of each include
        for _,inc in pairs(pkg.includes or {}) do
            args.includes += tostring(inc)
        end
        tup.append_table(args.linker_objects, pkg.object_files)
    end

    -- run everything once for every toolchain
    for _,toolchain in pairs(args.toolchains) do
        -- compile
        outputs = {}
        for _,src in pairs(args.sources) do
            --print("compile "..src)
            if tup.ext(src) == 'c' then
                toolchain.compile_c(src, args.c_flags, args.includes, outputs)
            elseif tup.ext(src) == 'cpp' then
                toolchain.compile_cpp(src, args.cpp_flags, args.includes, outputs)
            elseif tup.ext(src) == 's' or tup.ext(src) == 'asm' then
                toolchain.compile_asm(src, args.asm_flags, args.includes, outputs)
            else
                error('unrecognized file ending')
            end
        end

        -- link
        if outputs.object_files != nil and args.type != 'objects' then
            tup.append_table(args.linker_objects, outputs.object_files)
            toolchain.link(args.linker_objects, args.name)
        end

        outputs.includes = {}
        for _,inc in pairs(args.includes) do
            table.insert(outputs.includes, inc)
        end
        if args.name != nil then
            all_packages[args.name] = outputs
        end
    end

    --for k,v in pairs(all_packages) do
    --    print('have package '..k)
    --end
end

