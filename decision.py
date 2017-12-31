import numpy as np
import sys


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    # Check if the rover is stuck or not
    if np.abs(Rover.vel_history).mean() <= 0.2:
        index_stuck = Rover.frame_counter2 % 160
        # Do something and try to get rid of the stuck state
        if index_stuck <= 40:
            Rover.throttle = -4*Rover.throttle_set
        elif index_stuck > 40 and index_stuck <= 80:
            Rover.throttle = 0
            Rover.steer = -15
        elif index_stuck > 80 and index_stuck <= 120:
            Rover.throttle = 4*Rover.throttle_set
        else:
            Rover.throttle = 0
            Rover.steer = 15
    # If the rover is not stuck and can move freely, do the following.
    else:
        # Check if all the rock sample has been collected and rover is at the starting position.
        if Rover.samples_collected == 6:
            # Check if rover is at the starting position.
            dist_x = np.abs(Rover.pos[0]-Rover.start_pos[0])
            dist_y = np.abs(Rover.pos[1]-Rover.start_pos[1])
            dist_to_start = np.sqrt(dist_x**2+dist_y**2)
            if dist_to_start < 3:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                print('Mission Completed!')
            else:
                # Here, I just repeat the given code for mapping, the only change is that the steer direction is weighted
                # toward the starting position.
                # Check if we have vision data to make decisions with
                if Rover.nav_angles is not None:
                    # Check for Rover.mode status
                    if Rover.mode == 'sample':
                        Rover.mode = 'forward'
                    elif Rover.mode == 'forward': 
                        # Check the extent of navigable terrain
                        if len(Rover.nav_angles) >= Rover.stop_forward:
                            # If mode is forward, navigable terrain looks good 
                            # and velocity is below max, then throttle 
                            if Rover.vel < Rover.max_vel:
                                # Set throttle value to throttle setting
                                Rover.throttle = Rover.throttle_set
                            else: # Else coast
                                Rover.throttle = 0
                            Rover.brake = 0
                            # Set steering to average angle clipped to the range +/- 15
                            Rover.steer = np.clip((Rover.return_weights * Rover.nav_angles * 180/np.pi).sum(), -15, 15)
                        # If there's a lack of navigable terrain pixels then go to 'stop' mode
                        elif len(Rover.nav_angles) < Rover.stop_forward:
                                # Set mode to "stop" and hit the brakes!
                                Rover.throttle = 0
                                # Set brake to stored brake value
                                Rover.brake = Rover.brake_set
                                Rover.steer = 0
                                Rover.mode = 'stop'
                    # If we're already in "stop" mode then make different decisions
                    elif Rover.mode == 'stop':
                        # If we're in stop mode but still moving keep braking
                        if Rover.vel > 0.2:
                            Rover.throttle = 0
                            Rover.brake = Rover.brake_set
                            Rover.steer = 0
                        # If we're not moving (vel < 0.2) then do something else
                        elif Rover.vel <= 0.2:
                            # Now we're stopped and we have vision data to see if there's a path forward
                            if len(Rover.nav_angles) < Rover.go_forward:
                                Rover.throttle = 0
                                # Release the brake to allow turning
                                Rover.brake = 0
                                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                                Rover.steer = -15 # Could be more clever here about which way to turn
                            # If we're stopped but see sufficient navigable terrain in front then go!
                            if len(Rover.nav_angles) >= Rover.go_forward:
                                # Set throttle back to stored value
                                Rover.throttle = Rover.throttle_set
                                # Release the brake
                                Rover.brake = 0
                                # Set steer to mean angle
                                Rover.steer = np.clip((Rover.return_weights * Rover.nav_angles * 180/np.pi).sum(), -15, 15)
                                Rover.mode = 'forward'
                # Just to make the rover do something 
                # even if no modifications have been made to the code
                else:
                    Rover.throttle = 0
                    Rover.steer = -15
                    Rover.brake = 0
        # If there are still some rock samples to be collected, do the following.
        else:
            # Check if a rock sample is found.
            if (Rover.rock_angles is not None and len(Rover.rock_angles) >= Rover.toward_rock) or Rover.near_sample:
                # If in a state where want to pickup a rock send pickup command
                if Rover.near_sample and not Rover.picking_up:
                    Rover.mode = 'sample'
                    # Reset rock status
                    Rover.rock_angles = None
                    Rover.rock_dists = 0
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    if Rover.vel == 0 and not Rover.picking_up:
                        Rover.send_pickup = True
                elif np.mean(Rover.rock_dists) >= Rover.stop_dist_rock and Rover.near_sample == 0:
                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                    # Reset rock status
                    Rover.rock_angles = None
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                # If the distance to the rock sample is not large, do this.
                elif np.mean(Rover.rock_dists) < Rover.stop_dist_rock and Rover.near_sample == 0:
                    Rover.mode = 'sample'
                    if np.abs(Rover.steer-np.mean(Rover.rock_angles * 180/np.pi)) <= 1:
                        if Rover.vel > 1:
                            Rover.brake = Rover.brake_set
                            Rover.throttle = 0
                        else:
                            Rover.brake = 0
                            Rover.throttle = Rover.throttle_set
                    else:
                        Rover.brake = Rover.brake_set
                        if Rover.vel > 0:
                            Rover.brake = Rover.brake_set
                        else:
                            Rover.throttle = 0
                            Rover.brake = 0
                            Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                    # Reset rock status
                    Rover.rock_angles = None   
            # If no rock sample is found, just navigate and explore.
            else:
                # Check if we have vision data to make decisions with
                if Rover.nav_angles is not None:
                    # Check for Rover.mode status
                    if Rover.mode == 'sample':
                        Rover.mode = 'forward'
                    elif Rover.mode == 'forward': 
                        # Check the extent of navigable terrain
                        if len(Rover.nav_angles) >= Rover.stop_forward:  
                            # For every 3000 time steps, stop and do some turning for 100 frames
                            if Rover.frame_counter3 >= 2900:
                                Rover.throttle = 0
                                if Rover.vel > 0:
                                    Rover.brake = 0.1*Rover.brake_set
                                else:
                                    Rover.brake = 0
                                    Rover.steer = Rover.sign_steer * 15
                            else:
                                # If mode is forward, navigable terrain looks good 
                                # and velocity is below max, then throttle 
                                if Rover.vel < Rover.max_vel:
                                    # Set throttle value to throttle setting
                                    Rover.throttle = Rover.throttle_set
                                else: # Else coast
                                    Rover.throttle = 0
                                Rover.brake = 0
                                # Set steering to average angle clipped to the range +/- 15
                                average_angle = (Rover.nav_weights * Rover.nav_angles * 180/np.pi).sum()
                                bias_angle = 0.3*np.abs(average_angle)
                                Rover.steer = np.clip(average_angle + bias_angle*np.random.normal(), -15, 15)
                        # If there's a lack of navigable terrain pixels then go to 'stop' mode
                        elif len(Rover.nav_angles) < Rover.stop_forward:
                                # Set mode to "stop" and hit the brakes!
                                Rover.throttle = 0
                                # Set brake to stored brake value
                                Rover.brake = Rover.brake_set
                                Rover.steer = 0
                                Rover.mode = 'stop'
                    # If we're already in "stop" mode then make different decisions
                    elif Rover.mode == 'stop':
                        # If we're in stop mode but still moving keep braking
                        if Rover.vel > 0.2:
                            Rover.throttle = 0
                            Rover.brake = Rover.brake_set
                            Rover.steer = 0
                        # If we're not moving (vel < 0.2) then do something else
                        elif Rover.vel <= 0.2:
                            # Now we're stopped and we have vision data to see if there's a path forward
                            if len(Rover.nav_angles) < Rover.go_forward:
                                Rover.throttle = 0
                                # Release the brake to allow turning
                                Rover.brake = 0
                                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                                Rover.steer = -15 # Could be more clever here about which way to turn
                            # If we're stopped but see sufficient navigable terrain in front then go!
                            if len(Rover.nav_angles) >= Rover.go_forward:
                                # Set throttle back to stored value
                                Rover.throttle = Rover.throttle_set
                                # Release the brake
                                Rover.brake = 0
                                # Set steer to mean angle
                                average_angle = (Rover.nav_weights * Rover.nav_angles * 180/np.pi).sum()
                                bias_angle = 0.3*np.abs(average_angle)
                                Rover.steer = np.clip(average_angle + bias_angle*np.random.normal(), -15, 15)
                                Rover.mode = 'forward'
                # Just to make the rover do something 
                # even if no modifications have been made to the code
                else:
                    Rover.throttle = 0
                    Rover.steer = -15
                    Rover.brake = 0
        
    return Rover

