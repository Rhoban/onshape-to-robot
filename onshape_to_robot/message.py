from colorama import Fore, Back, Style, just_fix_windows_console

just_fix_windows_console()

def error(text: str):
    return Fore.RED + text + Style.RESET_ALL

def bright(text: str):
    return Style.BRIGHT + text + Style.RESET_ALL

def info(text: str):
    return Fore.BLUE + text + Style.RESET_ALL

def success(text: str):
    return Fore.GREEN + text + Style.RESET_ALL

def warning(text: str):
    return Fore.YELLOW + text + Style.RESET_ALL

def dim(text: str):
    return Style.DIM + text + Style.RESET_ALL
