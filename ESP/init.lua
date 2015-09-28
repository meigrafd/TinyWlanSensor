-- ----------------------------------
--  NodeMCU     TinySensors     v0.03
-- ----------------------------------

local requiredFiles = { 'config', 'functions' }

-- Check for first startup
for FILE,SIZE in pairs(file.list()) do
    if FILE == "_install.lua" then
        dofile("_install.lua")
        break
    end
end
FILE,SIZE=nil

-- Load required files
for i=1, #requiredFiles do
    success = require(requiredFiles[i])
    if success == false then
        error("ERROR: Missing required file: "..requiredFiles[i])
    end
end
i,requiredFiles,success = nil

print("READY")
