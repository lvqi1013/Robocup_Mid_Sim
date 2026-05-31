STYLE = """
QMainWindow, QWidget {
    background: #091018;
    color: #f4fbff;
    font-family: "Microsoft YaHei", "Segoe UI", Arial, sans-serif;
}
QWidget {
    font-size: 16px;
}
#title {
    font-size: 34px;
    font-weight: 800;
    color: #eefaff;
}
#glassPanel, #scoreboard {
    background: rgba(17, 31, 46, 0.92);
    border: 1px solid rgba(102, 224, 255, 0.35);
    border-radius: 8px;
}
#stageCheck {
    spacing: 10px;
    color: #d9f8ff;
    font-size: 18px;
    font-weight: 600;
    padding: 8px 10px;
}
#stageCheck::indicator {
    width: 22px;
    height: 22px;
    border-radius: 5px;
    border: 2px solid #66e0ff;
    background: #0d1a26;
}
#stageCheck::indicator:checked {
    background: #66e0ff;
    border: 2px solid #ff3d6e;
}
#teamLabel, #teamLabelRed {
    font-size: 14px;
    font-weight: 800;
    letter-spacing: 0px;
}
#teamLabel {
    color: #7fe7ff;
}
#teamLabelRed {
    color: #ff6b86;
}
QLineEdit {
    border-radius: 8px;
    padding: 0 18px;
    font-size: 22px;
    font-weight: 700;
    color: #ffffff;
    background: #0f1d2a;
    border: 2px solid rgba(255, 255, 255, 0.12);
    selection-background-color: #66e0ff;
}
QLineEdit:focus {
    border: 2px solid #66e0ff;
    background: #12283a;
}
#blackInput {
    border-left: 5px solid #66e0ff;
}
#redInput {
    border-right: 5px solid #ff3d6e;
}
#vs {
    min-width: 96px;
    color: #ffffff;
    font-size: 42px;
    font-weight: 900;
}
#primaryButton {
    border: 0;
    border-radius: 8px;
    color: #061018;
    font-size: 21px;
    font-weight: 900;
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #66e0ff, stop:1 #ff3d6e);
}
#primaryButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #92eeff, stop:1 #ff6f91);
}
#primaryButton:pressed {
    background: #ffffff;
}
#footer {
    color: rgba(232, 250, 255, 0.78);
    font-size: 18px;
    font-weight: 700;
}
#matchHeader {
    color: #eefaff;
    font-size: 29px;
    font-weight: 900;
}
#blackName, #redName {
    font-size: 24px;
    font-weight: 900;
}
#blackName {
    color: #7fe7ff;
}
#redName {
    color: #ff6b86;
}
#blackScore, #redScore {
    font-size: 118px;
    font-weight: 900;
}
#blackScore {
    color: #8becff;
}
#redScore {
    color: #ff4c78;
}
#scoreColon {
    font-size: 92px;
    font-weight: 900;
    color: #ffffff;
}
#timer {
    margin-top: 8px;
    padding: 14px;
    border-radius: 8px;
    background: #0b1722;
    border: 1px solid rgba(255, 255, 255, 0.16);
    color: #fff4b8;
    font-size: 52px;
    font-weight: 900;
}
#topicStatus {
    color: rgba(232, 250, 255, 0.68);
    font-size: 14px;
}
"""
