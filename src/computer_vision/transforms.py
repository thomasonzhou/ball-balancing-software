import numpy as np

PLATE_RADIUS_CM = 15.0


def scale_pixels_to_centimeters(circle_position_pixels, pixels_h):
    """Rough transform from pixels to distance"""
    return tuple(
        np.clip(
            pixel * (PLATE_RADIUS_CM / (pixels_h / 2.0)), min=-PLATE_RADIUS_CM, max=PLATE_RADIUS_CM
        )
        for pixel in circle_position_pixels
    )


def camera_view_to_plate_view(ball_position_bottom_view):
    return (-ball_position_bottom_view[0], ball_position_bottom_view[1])
