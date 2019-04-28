"""
    File name: ansi_color_code.py
    Maintainer: AyumiizZ
    Python Version: 2.7
    About: ansi code for printing color text
"""


class AnsiCode:
    """
    Class name: AnsiCode
    Maintainer: AyumiizZ
    About: ansi code for printing color text
    """
    def __init__(self):
        self.DEFAULT = '\033[0m'
        self.BOLD = '\033[1m'
        self.LIGHT = '\033[2m'
        self.ITALIC = '\033[3m'
        self.UNDERLINE = '\033[4m'
        self.HL = '\033[7m'
        self.INVISIBLE = '\033[8m'
        self.CROSS = '\033[9m'
        self.BLACK = '\033[30m'
        self.LIGHT_RED = '\033[31m'
        self.LIGHT_GREEN = '\033[32m'
        self.LIGHT_YELLOW = '\033[33m'
        self.LIGHT_BLUE = '\033[34m'
        self.LIGHT_PURPLE = '\033[35m'
        self.LIGHT_CYAN = '\033[36m'
        self.LIGHT_WHITE = '\033[37m'
        self.LIGHT_BLACK_HL = '\033[100m'
        self.LIGHT_RED_HL = '\033[41m'
        self.LIGHT_GREEN_HL = '\033[42m'
        self.LIGHT_YELLOW_HL = '\033[43m'
        self.LIGHT_BLUE_HL = '\033[44m'
        self.LIGHT_PURPLE_HL = '\033[45m'
        self.LIGHT_CYAN_HL = '\033[46m'
        self.LIGHT_WHITE_HL = '\033[47m'
        self.LIGHT_BLACK = '\033[90m'
        self.RED = '\033[91m'
        self.GREEN = '\033[92m'
        self.YELLOW = '\033[93m'
        self.BLUE = '\033[94m'
        self.PURPLE = '\033[95m'
        self.CYAN = '\033[96m'
        self.WHITE = '\033[97m'
        self.BLACK_HL = '\033[40m'
        self.RED_HL = '\033[101m'
        self.GREEN_HL = '\033[102m'
        self.YELLOW_HL = '\033[103m'
        self.BLUE_HL = '\033[104m'
        self.PURPLE_HL = '\033[105m'
        self.CYAN_HL = '\033[106m'
        self.WHITE_HL = '\033[107m'
