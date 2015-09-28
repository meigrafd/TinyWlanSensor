-- Connect to WLAN
wlanConnect = function(ssid, passwd)
    local ssid = ssid or SSID
    local passwd = passwd or PASSWD
    wifi.setmode(wifi.STATION)
    wifi.sta.config(ssid, passwd)
    wifi.sta.connect()
    tmr.alarm(0, 1000, 1, function()
        if wifi.sta.getip() == nil then
            --print("Connecting to AP...")
            local dummy=1
        else
            IP = wifi.sta.getip()
            --print("-> Got IP: ".. IP)
            tmr.stop(0)
            print("DONE")
        end
    end)
end

-- Disconnect from WLAN
wlanDisconnect = function()
    wifi.sta.disconnect()
    print("DONE")
end

-- Send data to ip..
wlanSend = function(data, ip, port, page)
    if data == nil then
        print("ERROR: Missing data")
        do return end
    end
    local ip = ip or SERVERIP
    local port = port or SERVERPORT
    local page = page or PAGE
    local page = page.."?"..data
    local client=net.createConnection(net.TCP, false)
    client:connect(port, ip)
    --print('Sending http request.')
    client:send("GET /" .. page .. " HTTP/1.1\r\nHost: " .. ip .. "\r\nConnection: keep-alive\r\nAccept: */*\r\n\r\n")
    print("DONE")
    page,client=nil
end

-- Return MAC of ESP8266 Module
getMAC = function()
    print('MAC:'..wifi.sta.getmac())
end

-- Return WLAN IP
getIP = function()
    if IP then
        print('IP:'..IP)
    else
        print('IP:NONE')
    end
end
