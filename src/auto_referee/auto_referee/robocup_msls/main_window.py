from .pages.match_page import MatchPage
from .pages.start_page import StartPage
from .qt_compat import QIcon, QMainWindow, QStackedWidget
from .ros_subscriber import RosScoreSubscriber
from .score_bridge import ScoreBridge
from .styles import STYLE


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robocup MSLS UI")
        self.setWindowIcon(QIcon())
        self.resize(980, 620)
        self.setMinimumSize(760, 520)

        self.bridge = ScoreBridge()
        self.ros = RosScoreSubscriber(self.bridge)

        self.stack = QStackedWidget()
        self.start_page = StartPage()
        self.match_page = MatchPage(self.bridge)
        self.stack.addWidget(self.start_page)
        self.stack.addWidget(self.match_page)
        self.setCentralWidget(self.stack)

        self.start_page.createMatch.connect(self.open_match)
        self.setStyleSheet(STYLE)

        if self.ros.available:
            self.match_page.topic_status.setText("ROS2 score subscription active")

    def open_match(self, match):
        self.match_page.load_match(match)
        self.stack.setCurrentWidget(self.match_page)

    def closeEvent(self, event):
        self.ros.shutdown()
        super().closeEvent(event)
