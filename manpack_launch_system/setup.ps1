# Get the ID and security principal of the current user account
$myWindowsID=[System.Security.Principal.WindowsIdentity]::GetCurrent()
$myWindowsPrincipal=new-object System.Security.Principal.WindowsPrincipal($myWindowsID)

# Get the security principal for the Administrator role
$adminRole=[System.Security.Principal.WindowsBuiltInRole]::Administrator

# Check to see if we are currently running "as Administrator"
if ($myWindowsPrincipal.IsInRole($adminRole))
   {
   # We are running "as Administrator" - so change the title and background color to indicate this
   $Host.UI.RawUI.WindowTitle = $myInvocation.MyCommand.Definition + "(Elevated)"
   $Host.UI.RawUI.BackgroundColor = "DarkBlue"
   clear-host
   }
else
   {
   # We are not running "as Administrator" - so relaunch as administrator
   
   # Create a new process object that starts PowerShell
   $newProcess = new-object System.Diagnostics.ProcessStartInfo "PowerShell";
   
   # Specify the current script path and name as a parameter
   $newProcess.Arguments = $myInvocation.MyCommand.Definition;
   
   # Indicate that the process should be elevated
   $newProcess.Verb = "runas";
   
   # Start the new process
   [System.Diagnostics.Process]::Start($newProcess);
   
   # Exit from the current, unelevated, process
   exit
   }

echo ""
echo ""
echo "Enabling WSL and Virtual Machine Platform"
echo ""
echo ""

dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart

Write-Host -NoNewLine "Press any key to continue..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")

echo ""
echo ""
echo "Installing WSL"
echo ""
echo ""
# create staging directory if it does not exists
if (-Not (Test-Path -Path .\temp)) { $dir = mkdir .\temp }

.\wsl_packages\wsl_update_x64.msi /quiet
wsl.exe --set-default-version 2

Write-Host -NoNewLine "Press any key to continue..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")

echo ""
echo ""
echo "Installing Ubuntu in WSL"
echo ""
echo ""

$wslName = "ManPackLaunchSystem"
$wslInstallationPath = "C:\WSL2\ManPackLaunchSystem"
$username = "pnt"
$installAllSoftware = "false"

# create temp directory if it does not exists
if (-Not (Test-Path -Path .\temp)) { 
    $dir = mkdir .\temp 
}

Copy-Item .\wsl_packages\ubuntuWSL.appx .\temp\ubuntuWSL.zip
Expand-Archive .\temp\ubuntuWSL.zip .\temp\ubuntuWSL

if (-Not (Test-Path -Path $wslInstallationPath)) {
    mkdir $wslInstallationPath
}
wsl.exe --import $wslName $wslInstallationPath .\temp\ubuntuWSL\install.tar.gz

Remove-Item .\temp\ubuntuWSL.zip
Remove-Item -r .\temp

Write-Host -NoNewLine "Press any key to continue..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")

# Update the system
wsl.exe -d $wslName -u root bash -ic "apt update; apt upgrade -y"

# create your user and add it to sudoers
wsl.exe -d $wslName -u root bash -ic "./scripts/config/system/createUser.sh $username ubuntu"

# ensure WSL Distro is restarted when first used with user account
wsl.exe -t $wslName

# if ($installAllSoftware -ieq $true) {
#     wsl.exe -d $wslName -u root bash -ic "./scripts/config/system/sudoNoPasswd.sh $username"
#     wsl.exe -d $wslName -u root bash -ic ./scripts/install/installBasePackages.sh
#     wsl.exe -d $wslName -u $username bash -ic ./scripts/install/installAllSoftware.sh
#     wsl.exe -d $wslName -u root bash -ic "./scripts/config/system/sudoWithPasswd.sh $username"
# }

Write-Host -NoNewLine "Press any key to continue..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")