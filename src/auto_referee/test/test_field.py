from auto_referee.field import Field, Point


def test_goal_requires_mouth_height_and_depth():
    field = Field()
    assert field.is_goal(Point(1110.0, 0.0), 12.0)
    assert not field.is_goal(Point(1110.0, 110.0), 12.0)
    assert not field.is_goal(Point(1110.0, 0.0), 95.0)
    assert not field.is_goal(Point(1200.0, 0.0), 12.0)


def test_restart_point_is_moved_out_of_penalty_area():
    field = Field()
    assert field.restart_outside_penalty(Point(1000.0, 20.0)) == Point(
        740.0, 350.0
    )
    assert field.restart_outside_penalty(Point(-1000.0, -20.0)) == Point(
        -740.0, -350.0
    )
    point = Point(100.0, 200.0)
    assert field.restart_outside_penalty(point) == point


def test_touchline_and_goal_line_are_distinct():
    field = Field()
    assert field.is_out_up(Point(0.0, 701.0))
    assert not field.is_out_right(Point(0.0, 701.0))
    assert field.is_out_right(Point(1101.0, 0.0))
