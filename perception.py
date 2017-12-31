import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                 & (img[:,:,1] > rgb_thresh[1]) \
                 & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def border_thresh(img, rgb_thresh=(0, 255, 255)):
    # Create an array of zeros same xy size as img, but single channel
    border_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    on_thresh = (img[:,:,0] == rgb_thresh[0]) \
              & (img[:,:,1] == rgb_thresh[1]) \
              & (img[:,:,2] == rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    border_select[on_thresh] = 1
    border_select = border_select == 1
    # Return the binary image
    return border_select

def find_rocks(img, rgb_thresh_rock=(100,100,50)):
    # Create an array of zeros same xy size as img, but single channel
    rocks_select = np.zeros_like(img[:,:,0])
    
    rocks_thresh = (img[:,:,0] > rgb_thresh_rock[0]) \
                 & (img[:,:,1] > rgb_thresh_rock[1]) \
                 & (img[:,:,2] < rgb_thresh_rock[2]) \
    
    rocks_select[rocks_thresh] = 1
    # Return the binary image
    return rocks_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dists = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dists, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(
        img, M, (img.shape[1], img.shape[0]), borderValue = [0,255,255])# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed_navigable = color_thresh(warped)
    threshed_border = border_thresh(warped)
    threshed_rocks = find_rocks(warped)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = 255*(1-threshed_navigable)
    Rover.vision_image[:,:,1] = 255*threshed_rocks
    Rover.vision_image[:,:,2] = 255*threshed_navigable
    Rover.vision_image[threshed_border,:] = 0

    # 5) Convert map image pixel values to rover-centric coords
    xpix_navigable, ypix_navigable = rover_coords(threshed_navigable)
    
    # 6) Convert rover-centric pixel values to world coordinates
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    world_size = Rover.worldmap.shape[0]
    scale = 10
    # Navigable terrain
    xpix_navigable_world, ypix_navigable_world = pix_to_world(
        xpix_navigable, ypix_navigable, xpos, ypos, yaw, world_size, scale) 
    # Rock samples
    if threshed_rocks.any():
        xpix_rock, ypix_rock = rover_coords(threshed_rocks)
        xpix_rock_world, ypix_rock_world = pix_to_world(
            xpix_rock, ypix_rock, xpos, ypos, yaw, world_size, scale)
        # Calcualte the distances and angles of the rock
        dists_rock, angles_rock = to_polar_coords(xpix_rock, ypix_rock)
        Rover.rock_dists = dists_rock
        Rover.rock_angles = angles_rock
        
    # Obstacles
    threshed_obstacle = np.zeros_like(threshed_navigable)
    threshed_obstacle[threshed_navigable==0] = 1
    xpix_obstacle, ypix_obstacle = rover_coords(threshed_obstacle)
    xpix_obstacle_world, ypix_obstacle_world = pix_to_world(
        xpix_obstacle, ypix_obstacle, xpos, ypos, yaw, world_size, scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen) and the countermap.
    if (Rover.roll < 0.3 or Rover.roll > 359.7) and (Rover.pitch < 0.3 or Rover.pitch > 359.7) and not Rover.picking_up:
        # Obstacles
        Rover.worldmap[ypix_obstacle_world, xpix_obstacle_world, 0] = 255
        # Rock samples
        if threshed_rocks.any():
            Rover.worldmap[ypix_rock_world, xpix_rock_world, 1] = 255
        # Navigable terrain
        Rover.worldmap[ypix_navigable_world, xpix_navigable_world, 2] = 255
        Rover.worldmap[ypix_navigable_world, xpix_navigable_world, 0] = 0
        # Countermap
        Rover.countermap[ypix_navigable_world, xpix_navigable_world] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    dists, angles = to_polar_coords(xpix_navigable, ypix_navigable)
    # Update Rover pixel distances and angles
    Rover.nav_weights = np.exp(-Rover.countermap[ypix_navigable_world, xpix_navigable_world]/100.)
    Rover.nav_weights = Rover.nav_weights/Rover.nav_weights.sum()
    Rover.nav_dists = dists
    Rover.nav_angles = angles
    # Update navigation weights for return
    # Calculate the x and y distance from all the current navigable pixels to the starting position
    dist_ys = np.abs(ypix_navigable_world-Rover.start_pos[1])
    dist_xs = np.abs(xpix_navigable_world-Rover.start_pos[0])
    dist_to_starts = np.sqrt(dist_ys**2+dist_xs**2)
    Rover.return_weights = np.exp(-dist_to_starts/2.)
    Rover.return_weights = Rover.return_weights/Rover.return_weights.sum()
    
    return Rover