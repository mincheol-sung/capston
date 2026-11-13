"""Launch-time CAN setup. Does not open a CAN socket or send motor commands."""
import subprocess


def prepare_can(script, interface, device=''):
    command = ['bash', str(script), '--non-interactive', '--interface', interface]
    if device:
        command.append(device)
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f'CAN setup failed for {interface}: {exc}. Motor nodes were not started.') from exc
    output = (result.stdout + result.stderr).strip()
    if result.returncode:
        raise RuntimeError(
            f'CAN setup failed for {interface}; motor nodes were not started.\n{output}\n'
            'If sudo authorization is required, run sudo -v in this terminal and relaunch. '
            'Alternatively prepare CAN manually and use auto_setup_can:=false.')
    return output
