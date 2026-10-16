"""
Test-only exact ray/sphere intersections for current depth localization.

No perception fit or estimator is used to generate these known surfaces.
"""
import numpy as np

WIDTH, HEIGHT, FOV, RADIUS = 640, 480, 1.74, .25


def render(center):
    fx, fy, cx, cy = (WIDTH / (2 * np.tan(FOV / 2)),) * 2 + (WIDTH / 2, HEIGHT / 2)
    rows, cols = np.indices((HEIGHT, WIDTH))
    rays = np.stack((np.ones((HEIGHT, WIDTH)),
                     -(cols - cx) / fx, -(rows - cy) / fy), axis=-1)
    center = np.asarray(center, dtype=float)
    dot = rays @ center
    ray_norm2 = np.sum(rays * rays, axis=-1)
    disc = dot * dot - ray_norm2 * (center @ center - RADIUS ** 2)
    mask = (disc >= 0) & (dot > 0)
    depth = np.full((HEIGHT, WIDTH), np.inf)
    depth[mask] = (dot[mask] - np.sqrt(disc[mask])) / ray_norm2[mask]
    return mask, depth, rays
