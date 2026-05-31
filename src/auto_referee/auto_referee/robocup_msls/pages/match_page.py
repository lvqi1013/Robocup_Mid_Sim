from ..constants import APP_FOOTER, ROUND_LIMITS
from ..qt_compat import QFrame, QGridLayout, QLabel, QTimer, QVBoxLayout, QWidget, align_center
from ..score_bridge import ScoreBridge


class MatchPage(QWidget):
    def __init__(self, bridge: ScoreBridge):
        super().__init__()
        self.bridge = bridge
        self.remaining_seconds = 0
        self.countdown = QTimer(self)
        self.countdown.timeout.connect(self._tick)
        self._build_ui()
        self.bridge.blackScoreChanged.connect(self.set_black_score)
        self.bridge.redScoreChanged.connect(self.set_red_score)

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(44, 34, 44, 26)
        root.setSpacing(24)

        self.header = QLabel()
        self.header.setObjectName("matchHeader")
        self.header.setAlignment(align_center())
        root.addWidget(self.header)

        scoreboard = QFrame()
        scoreboard.setObjectName("scoreboard")
        board = QGridLayout(scoreboard)
        board.setContentsMargins(34, 28, 34, 28)
        board.setHorizontalSpacing(18)
        board.setVerticalSpacing(12)

        self.black_name = QLabel()
        self.black_name.setObjectName("blackName")
        self.black_name.setAlignment(align_center())
        self.red_name = QLabel()
        self.red_name.setObjectName("redName")
        self.red_name.setAlignment(align_center())

        self.black_score = QLabel("0")
        self.black_score.setObjectName("blackScore")
        self.black_score.setAlignment(align_center())
        self.red_score = QLabel("0")
        self.red_score.setObjectName("redScore")
        self.red_score.setAlignment(align_center())

        colon = QLabel(":")
        colon.setObjectName("scoreColon")
        colon.setAlignment(align_center())

        self.timer_label = QLabel("10:00")
        self.timer_label.setObjectName("timer")
        self.timer_label.setAlignment(align_center())

        board.addWidget(self.black_name, 0, 0)
        board.addWidget(self.red_name, 0, 2)
        board.addWidget(self.black_score, 1, 0)
        board.addWidget(colon, 1, 1)
        board.addWidget(self.red_score, 1, 2)
        board.addWidget(self.timer_label, 2, 0, 1, 3)
        board.setColumnStretch(0, 1)
        board.setColumnStretch(1, 0)
        board.setColumnStretch(2, 1)
        root.addWidget(scoreboard)

        self.topic_status = QLabel("Score topics: black_team_score / red_team_score")
        self.topic_status.setObjectName("topicStatus")
        self.topic_status.setAlignment(align_center())
        root.addWidget(self.topic_status)
        root.addStretch(1)

        footer = QLabel(APP_FOOTER)
        footer.setObjectName("footer")
        footer.setAlignment(align_center())
        root.addWidget(footer)

    def load_match(self, match):
        self.header.setText(f"{match.stage}  |  {match.black_team} VS {match.red_team}")
        self.black_name.setText(match.black_team)
        self.red_name.setText(match.red_team)
        self.black_score.setText("0")
        self.red_score.setText("0")
        self.remaining_seconds = ROUND_LIMITS[match.stage]
        self._render_timer()
        self.countdown.start(1000)

    def set_black_score(self, score: int):
        self.black_score.setText(str(score))

    def set_red_score(self, score: int):
        self.red_score.setText(str(score))

    def _tick(self):
        if self.remaining_seconds > 0:
            self.remaining_seconds -= 1
            self._render_timer()
        else:
            self.countdown.stop()
            self.timer_label.setText("TIME UP")

    def _render_timer(self):
        minutes, seconds = divmod(self.remaining_seconds, 60)
        self.timer_label.setText(f"{minutes:02d}:{seconds:02d}")
