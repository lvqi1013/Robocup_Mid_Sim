import sys

from robocup_msls.app import create_app, run_app
from robocup_msls.main_window import MainWindow


def main():
    app = create_app(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(run_app(app))


if __name__ == "__main__":
    main()
