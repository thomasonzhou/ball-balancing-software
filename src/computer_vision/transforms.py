import numpy as np

PLATE_RADIUS_CM = 13.0


def scale_pixels_to_centimeters(circle_position_pixels, pixels_h):
    """
    Transforms pixel positions to distances in centimeters.

    Parameters:
        circle_position_pixels (tuple[float, float]): A sequence of pixel positions to be transformed.
        pixels_h (float): The height of the pixel grid used for scaling.

    Returns:
        tuple[float, float]: A tuple of pixel positions scaled to centimeters, each clipped within [-PLATE_RADIUS_CM, PLATE_RADIUS_CM].
    """
    return tuple(
        np.clip(
            pixel * (PLATE_RADIUS_CM / (pixels_h / 2.0)), min=-PLATE_RADIUS_CM, max=PLATE_RADIUS_CM
        )
        for pixel in circle_position_pixels
    )


def camera_view_to_plate_view(ball_position_bottom_view):
    return (ball_position_bottom_view[0], ball_position_bottom_view[1])
