from datetime import datetime

from ..config_writer import write_match_config
from ..constants import APP_FOOTER, ROUND_LIMITS
from ..models import MatchInfo
from ..qt_compat import (
    QButtonGroup,
    QCheckBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
    align_center,
    pyqtSignal,
)


class StartPage(QWidget):
    createMatch = pyqtSignal(MatchInfo)

    def __init__(self):
        super().__init__()
        self.stage_group = QButtonGroup(self)
        self.stage_group.setExclusive(True)
        self._build_ui()

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(44, 34, 44, 26)
        root.setSpacing(28)

        title = QLabel("Robocup MSLS Match Center")
        title.setObjectName("title")
        title.setAlignment(align_center())
        root.addWidget(title)

        stages = QFrame()
        stages.setObjectName("glassPanel")
        stages_layout = QHBoxLayout(stages)
        stages_layout.setContentsMargins(18, 14, 18, 14)
        stages_layout.setSpacing(16)
        for index, stage in enumerate(ROUND_LIMITS):
            checkbox = QCheckBox(stage)
            checkbox.setObjectName("stageCheck")
            checkbox.setChecked(index == 0)
            self.stage_group.addButton(checkbox)
            stages_layout.addWidget(checkbox)
        root.addWidget(stages)

        versus = QGridLayout()
        versus.setHorizontalSpacing(28)
        versus.setVerticalSpacing(10)

        black_label = QLabel("BLACK TEAM")
        black_label.setObjectName("teamLabel")
        red_label = QLabel("RED TEAM")
        red_label.setObjectName("teamLabelRed")

        self.black_input = QLineEdit()
        self.black_input.setObjectName("blackInput")
        self.black_input.setPlaceholderText("输入黑队名称")
        self.black_input.setMinimumHeight(58)

        self.red_input = QLineEdit()
        self.red_input.setObjectName("redInput")
        self.red_input.setPlaceholderText("输入红队名称")
        self.red_input.setMinimumHeight(58)

        vs = QLabel("VS")
        vs.setObjectName("vs")
        vs.setAlignment(align_center())

        versus.addWidget(black_label, 0, 0)
        versus.addWidget(red_label, 0, 2)
        versus.addWidget(self.black_input, 1, 0)
        versus.addWidget(vs, 1, 1)
        versus.addWidget(self.red_input, 1, 2)
        versus.setColumnStretch(0, 1)
        versus.setColumnStretch(1, 0)
        versus.setColumnStretch(2, 1)
        root.addLayout(versus)

        self.create_button = QPushButton("创建比赛")
        self.create_button.setObjectName("primaryButton")
        self.create_button.setMinimumHeight(58)
        self.create_button.clicked.connect(self._create_match)
        root.addWidget(self.create_button)
        root.addStretch(1)

        footer = QLabel(APP_FOOTER)
        footer.setObjectName("footer")
        footer.setAlignment(align_center())
        root.addWidget(footer)

    def _create_match(self):
        selected = self.stage_group.checkedButton()
        black_team = self.black_input.text().strip()
        red_team = self.red_input.text().strip()
        if not selected or not black_team or not red_team:
            QMessageBox.warning(self, "信息不完整", "请选择赛制，并填写黑队和红队名称。")
            return
        if black_team == red_team:
            QMessageBox.warning(self, "队伍重复", "黑队和红队名称不能相同。")
            return

        match = MatchInfo(
            stage=selected.text(),
            black_team=black_team,
            red_team=red_team,
            timestamp=datetime.now().strftime("%Y-%m-%d %H:%M"),
        )
        write_match_config(match)
        self.createMatch.emit(match)
