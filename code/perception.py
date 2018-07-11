import numpy as np
import cv2
import time


def navigable_thresh(img, rgb_thresh=(160, 160, 160)):

    color_select = np.zeros_like(img[:,:,0])
    above_thresh = ((img[:,:,0] > rgb_thresh[0]) &
                    (img[:,:,1] > rgb_thresh[1]) &
                    (img[:,:,2] > rgb_thresh[2]))
    color_select[above_thresh] = 1
    return color_select

def obstacle_thresh(img, rgb_thresh=(160, 160, 160)):

    color_select = np.zeros_like(img[:,:,0])
    below_thresh = ((img[:,:,0] < rgb_thresh[0]) &
                    (img[:,:,1] < rgb_thresh[1]) &
                    (img[:,:,2] < rgb_thresh[2]))
    color_select[below_thresh] = 1
    return color_select


def rock_thresh(img):

    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV, 3)
    lower_yellow = np.array([20, 150, 100], dtype='uint8')
    upper_yellow = np.array([50, 255, 255], dtype='uint8')
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    return mask 

def rover_coords(binary_img):

    ypos, xpos = binary_img.nonzero()
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel

def to_polar_coords(x_pixel, y_pixel):

    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

def rotate_pix(xpix, ypix, yaw):

    yaw_rad = yaw * np.pi / 180
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 

    xpix_translated = (xpos + (xpix_rot / scale))
    ypix_translated = (ypos + (ypix_rot / scale))
    return xpix_translated, ypix_translated

def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale=10):

    xpix_rot, ypix_rot = rotate_pix(xpix=xpix, ypix=ypix, yaw=yaw)
    xpix_tran, ypix_tran = translate_pix(xpix_rot=xpix_rot, ypix_rot=ypix_rot,
                                         xpos=xpos, ypos=ypos, scale=scale)
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    return x_pix_world, y_pix_world

def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    return warped

def perception_step(Rover):

    img = Rover.img
    dst_size = 5
    bottom_offset = 6
    src = np.float32([[14, 140], [300, 140], [200, 96], [118, 96]])
    dst = np.float32([
        [img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
        [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
        [img.shape[1]/2 + dst_size, img.shape[0] - 2 * dst_size - bottom_offset],
        [img.shape[1]/2 - dst_size, img.shape[0] - 2 * dst_size - bottom_offset]])

    warped = perspect_transform(img=img, src=src, dst=dst)
    navigable = navigable_thresh(img=warped, rgb_thresh=(160, 160, 160)) 
    obstacles = obstacle_thresh(img=warped, rgb_thresh=(140, 140, 140))
    rock_samples = rock_thresh(img=warped)
    Rover.vision_image[:,:,0] = obstacles * 255
    Rover.vision_image[:,:,1] = rock_samples * 255
    Rover.vision_image[:,:,2] = navigable * 255
    navigable_xpix, navigable_ypix = rover_coords(binary_img=navigable)
    obstacles_xpix, obstacles_ypix = rover_coords(binary_img=obstacles)
    rocks_xpix, rocks_ypix = rover_coords(binary_img=rock_samples)

    scale = dst_size * 2
    xpos, ypos = Rover.pos
    yaw = Rover.yaw
    worldmap_size = Rover.worldmap.shape[0]

    navigable_x_world, navigable_y_world = pix_to_world(
        xpix=navigable_xpix, ypix=navigable_ypix,
        xpos=xpos, ypos=ypos, yaw=yaw, world_size=worldmap_size, scale=scale)
    obstacles_x_world, obstacles_y_world = pix_to_world(
        xpix=obstacles_xpix, ypix=obstacles_ypix,
        xpos=xpos, ypos=ypos, yaw=yaw, world_size=worldmap_size, scale=scale)
    rocks_x_world, rocks_y_world = pix_to_world(
        xpix=rocks_xpix, ypix=rocks_ypix,
        xpos=xpos, ypos=ypos, yaw=yaw, world_size=worldmap_size, scale=scale)

    if (Rover.pitch < 0.5 or Rover.pitch > 359.5) and (Rover.roll < 0.5 or Rover.roll > 359.5):
        Rover.worldmap[obstacles_y_world, obstacles_x_world, 0] += 1
        Rover.worldmap[rocks_y_world, rocks_x_world, 1] = 255
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    distances, angles = to_polar_coords(x_pixel=navigable_xpix,
                                        y_pixel=navigable_ypix)
    Rover.nav_dists = distances
    Rover.nav_angles = angles

    if len(rocks_xpix) > 5:
        rock_distance, rock_angle = to_polar_coords(x_pixel=rocks_xpix,
                                                    y_pixel=rocks_ypix)
        Rover.rock_dist = rock_distance
        Rover.rock_angle = rock_angle 
        if not Rover.sample_seen:
            Rover.sample_timer = time.time()
        Rover.sample_seen = True

    if Rover.start_pos is None:
        Rover.start_pos = (Rover.pos[0], Rover.pos[1])
        print('Start_Pos ', Rover.start_pos)

    return Rover
