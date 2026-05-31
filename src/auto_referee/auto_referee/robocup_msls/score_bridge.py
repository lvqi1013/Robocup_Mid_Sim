from .qt_compat import QObject, pyqtSignal


class ScoreBridge(QObject):
    blackScoreChanged = pyqtSignal(int)
    redScoreChanged = pyqtSignal(int)
