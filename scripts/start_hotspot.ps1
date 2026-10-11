Add-Type -AssemblyName System.Runtime.WindowsRuntime
$asTaskGeneric = ([System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object { $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' })[0]

Function Await($WinRtTask, $ResultType) {
    $asTask = $asTaskGeneric.MakeGenericMethod($ResultType)
    $netTask = $asTask.Invoke($null, @($WinRtTask))
    $netTask.Wait(5000) | Out-Null
    $netTask.Result
}

try {
    $profile = [Windows.Networking.Connectivity.NetworkInformation,Windows.Networking.Connectivity,ContentType=WindowsRuntime]::GetInternetConnectionProfile()
    if ($null -ne $profile) {
        $tetheringManager = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]::CreateFromConnectionProfile($profile)
        if ($tetheringManager.TetheringOperationalState -ne "On") {
            Write-Host "Enabling Mobile Hotspot..."
            $asyncOp = $tetheringManager.StartTetheringAsync()
            $res = Await $asyncOp ([Windows.Networking.NetworkOperators.NetworkOperatorTetheringOperationResult])
            Write-Host "Result: $($res.Status)"
        } else {
            Write-Host "Hotspot already On."
        }
    }
} catch {
    Write-Host "Error: $_"
}
