from .qt_compat import QT6, QApplication, QFont


def create_app(argv):
    QApplication.setFont(QFont("Microsoft YaHei", 10))
    return QApplication(argv)


def run_app(app):
    return app.exec() if QT6 else app.exec_()
