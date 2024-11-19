from ball_detection.transforms import camera_view_to_plate_view


def get_ball_position_camera_view():
    raise NotImplementedError


def get_ball_position_plate_view():
    ball_position_bottom_view = get_ball_position_camera_view()
    ball_position_top_view = camera_view_to_plate_view(ball_position_bottom_view)
    return ball_position_top_view
