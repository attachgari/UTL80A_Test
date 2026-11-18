param([switch]$NoBrowser)

$ErrorActionPreference = 'Stop'
$webRoot = Join-Path $PSScriptRoot 'web'
$pages = @{
    '/' = @{ File = 'index.html'; Type = 'text/html; charset=utf-8' }
    '/index.html' = @{ File = 'index.html'; Type = 'text/html; charset=utf-8' }
    '/app.js' = @{ File = 'app.js'; Type = 'text/javascript; charset=utf-8' }
}
foreach ($file in @('index.html', 'app.js')) {
    if (-not (Test-Path -LiteralPath (Join-Path $webRoot $file) -PathType Leaf)) {
        throw "Missing web/$file. Keep the launcher and web folder together."
    }
}

$listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, 8765)
$listener.ExclusiveAddressUse = $true
try {
    $listener.Start()
} catch [System.Net.Sockets.SocketException] {
    if ($_.Exception.SocketErrorCode -ne [System.Net.Sockets.SocketError]::AddressAlreadyInUse) { throw }
    $listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, 0)
    $listener.Start()
}

try {
    $url = 'http://localhost:{0}/' -f $listener.LocalEndpoint.Port
    Write-Host "UTL80A RAW Monitor: $url"
    Write-Host 'Keep this window open. Press Ctrl+C to stop.'
    if (-not $NoBrowser) {
        try {
            Start-Process $url
        } catch {
            Write-Host 'Open the URL above in Chrome or Edge.'
        }
    }

    while ($true) {
        if (-not $listener.Pending()) {
            Start-Sleep -Milliseconds 50
            continue
        }
        $client = $listener.AcceptTcpClient()
        try {
            $client.ReceiveTimeout = 1500
            $client.SendTimeout = 1500
            $stream = $client.GetStream()
            $reader = [System.IO.StreamReader]::new($stream, [System.Text.Encoding]::ASCII)
            $request = $reader.ReadLine()
            if ($request -notmatch '^(GET|HEAD) (/[^ ]*) HTTP/1\.[01]$') { continue }
            $method = $Matches[1]
            $path = $Matches[2].Split('?')[0]
            $page = $pages[$path]
            if ($page) {
                $status = '200 OK'
                $type = $page.Type
                $body = [System.IO.File]::ReadAllBytes((Join-Path $webRoot $page.File))
            } else {
                $status = '404 Not Found'
                $type = 'text/plain; charset=utf-8'
                $body = [System.Text.Encoding]::UTF8.GetBytes('Not found')
            }
            $head = "HTTP/1.0 $status`r`nContent-Type: $type`r`nContent-Length: $($body.Length)`r`nCache-Control: no-store`r`nConnection: close`r`n`r`n"
            $headerBytes = [System.Text.Encoding]::ASCII.GetBytes($head)
            $stream.Write($headerBytes, 0, $headerBytes.Length)
            if ($method -eq 'GET') { $stream.Write($body, 0, $body.Length) }
        } catch [System.IO.IOException] {
            continue
        } finally {
            $client.Dispose()
        }
    }
} finally {
    $listener.Stop()
}
