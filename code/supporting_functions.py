import numpy as np
import cv2
from PIL import Image
from io import BytesIO, StringIO
import base64
import time


def convert_to_float(string_to_convert):
    if ',' in string_to_convert:
        float_value = np.float(string_to_convert.replace(',', '.'))
    else:
        float_value = np.float(string_to_convert)
    return float_value


def update_rover(Rover, data):
      if Rover.start_time == None:
            Rover.start_time = time.time()
            Rover.total_time = 0
            samples_xpos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_x"].split(';')])
            samples_ypos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_y"].split(';')])
            Rover.samples_pos = (samples_xpos, samples_ypos)
            Rover.samples_to_find = np.int(data['sample_count'])
      else:
            tot_time = time.time() - Rover.start_time
            if np.isfinite(tot_time):
                  Rover.total_time = tot_time
      Rover.vel = convert_to_float(data["speed"])
      Rover.pos = [convert_to_float(pos.strip()) for pos in data["position"].split(';')]
      Rover.yaw = convert_to_float(data["yaw"])
      Rover.pitch = convert_to_float(data["pitch"])
      Rover.roll = convert_to_float(data["roll"])
      Rover.throttle = convert_to_float(data["throttle"])
      Rover.steer = convert_to_float(data["steering_angle"])
      Rover.near_sample = np.int(data["near_sample"])
      Rover.picking_up = np.int(data["picking_up"])
      Rover.samples_found = Rover.samples_to_find - np.int(data['sample_count'])
      
      imgString = data["image"]
      image = Image.open(BytesIO(base64.b64decode(imgString)))
      Rover.img = np.asarray(image)

      return Rover, image


def create_output_images(Rover):

      if np.max(Rover.worldmap[:,:,2]) > 0:
            nav_pix = Rover.worldmap[:,:,2] > 0
            navigable = Rover.worldmap[:,:,2] * (255 / np.mean(Rover.worldmap[nav_pix, 2]))
      else: 
            navigable = Rover.worldmap[:,:,2]
      if np.max(Rover.worldmap[:,:,0]) > 0:
            obs_pix = Rover.worldmap[:,:,0] > 0
            obstacle = Rover.worldmap[:,:,0] * (255 / np.mean(Rover.worldmap[obs_pix, 0]))
      else:
            obstacle = Rover.worldmap[:,:,0]

      likely_nav = navigable >= obstacle
      obstacle[likely_nav] = 0
      plotmap = np.zeros_like(Rover.worldmap)
      plotmap[:, :, 0] = obstacle
      plotmap[:, :, 2] = navigable
      plotmap = plotmap.clip(0, 255)
      map_add = cv2.addWeighted(plotmap, 1, Rover.ground_truth, 0.5, 0)

      rock_world_pos = Rover.worldmap[:,:,1].nonzero()

      if rock_world_pos[0].any():
            rock_size = 2
            for idx in range(len(Rover.samples_pos[0])):
                  test_rock_x = Rover.samples_pos[0][idx]
                  test_rock_y = Rover.samples_pos[1][idx]
                  rock_sample_dists = np.sqrt((test_rock_x - rock_world_pos[1])**2 + \
                                        (test_rock_y - rock_world_pos[0])**2)
                  if np.min(rock_sample_dists) < 3:
                        map_add[test_rock_y-rock_size:test_rock_y+rock_size, 
                        test_rock_x-rock_size:test_rock_x+rock_size, :] = 255

      tot_nav_pix = np.float(len((plotmap[:,:,2].nonzero()[0])))
      good_nav_pix = np.float(len(((plotmap[:,:,2] > 0) & (Rover.ground_truth[:,:,1] > 0)).nonzero()[0]))
      bad_nav_pix = np.float(len(((plotmap[:,:,2] > 0) & (Rover.ground_truth[:,:,1] == 0)).nonzero()[0]))
      tot_map_pix = np.float(len((Rover.ground_truth[:,:,1].nonzero()[0])))
      perc_mapped = round(100*good_nav_pix/tot_map_pix, 1)
      if tot_nav_pix > 0:
            fidelity = round(100*good_nav_pix/(tot_nav_pix), 1)
      else:
            fidelity = 0
      map_add = np.flipud(map_add).astype(np.float32)

      cv2.putText(map_add,"Time: "+str(np.round(Rover.total_time, 1))+' s', (0, 10), 
                  cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
      cv2.putText(map_add,"Mapped: "+str(perc_mapped)+'%', (0, 25), 
                  cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
      cv2.putText(map_add,"Fidelity: "+str(fidelity)+'%', (0, 40), 
                  cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
      cv2.putText(map_add,"Rocks Found: "+str(Rover.samples_found), (0, 55), 
                  cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)

      pil_img = Image.fromarray(map_add.astype(np.uint8))
      buff = BytesIO()
      pil_img.save(buff, format="JPEG")
      encoded_string1 = base64.b64encode(buff.getvalue()).decode("utf-8")
      
      pil_img = Image.fromarray(Rover.vision_image.astype(np.uint8))
      buff = BytesIO()
      pil_img.save(buff, format="JPEG")
      encoded_string2 = base64.b64encode(buff.getvalue()).decode("utf-8")

      return encoded_string1, encoded_string2
