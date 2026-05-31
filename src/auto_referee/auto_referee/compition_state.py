
class CompitionState:
    def __init__(self):
        self.black_team_name = '' 
        self.red_team_name = ''

        self.black_team_score = 0
        self.red_team_score = 0

class FieldConfig:
    BALL_RADIUS = 11.0  
    """足球的半径 m"""

    FIELD_LENGTH = 2200.0
    """球场的长度 x方向  cm"""

    GOALPOST_WIDTH = 75.0
    """球门立柱沿 X 方向的宽度/纵深 X轴进球判定（判断球是否进入球门纵深范围） cm"""

    GOAL_HEIGHT = 101.0
    """球门横梁下沿到地面的高度 cm(z方向)"""

    GOALPOST_LEN = 240.0
    """球门的长度 cm (y方向)"""

    def m2cm(self, x:float):
        """单位换算: m->cm"""
        return x * 100.0