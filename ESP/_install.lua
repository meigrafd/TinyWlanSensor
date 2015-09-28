--
-- Execute this on first setup to compile all needed files. 
-- This file will deletes itself after done :)
--

skipFiles={ 'init.lua', '_install.lua' }

for FILE,SIZE in pairs(file.list()) do
    local skipIt=false
    for i=1, #skipFiles do
        if FILE == skipFiles[i] then
            skipIt=true
         end
    end
    if skipIt == false then
        print("compiling: "..FILE)
        node.compile(FILE)
        file.remove(FILE)
    end
end

print("Done")
--delete variables to cleanup memory
FILE,SIZE,skipFiles,skipIt=nil
--delete file
file.remove("_install.lua")
