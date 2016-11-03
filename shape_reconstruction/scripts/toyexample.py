#/usr/bin/python

import numpy as np

class Transformation:
  def __init__(self, value):
    self.value = value
    
  def apply(self, image, value=None):
    """Apply transformation to a image, i.e. shift the entire image"""
    if value==None:
      value = self.value
    
    image_new = [None]*len(image)
    for i, val in enumerate(image):
      idx = i + value
      if idx >= len(image_new):
        break
      if idx < 0:
        continue
      image_new[idx] = val

    return image_new

  def applyInverse(self, image):
    return self.apply(image, value=-self.value)
    
  def applyToScene(self, scene, obj_id):
    """Apply transformation to a scene, i.e. move one object"""
    scene.moveObject(obj_id, self.value)
    
  
class Scene:
  def __init__(self, image_size):
    self.image_size = image_size
    self.objects = {}
    #self.z_current = 0
    
  def addObject(self, id, object_pattern, color_pattern, offset):
    # order of adding object matters  
    #z = self.z_current
    #self.z_current += 1
    
    self.objects[id] = [offset, object_pattern, color_pattern]
  
  def moveObject(self, obj_id, value):
    assert obj_id in self.objects.keys()
    self.objects[obj_id][0] += value

  def generateImage(self, shift=0):
    image = [0] * self.image_size
    color_image = ['-'] * self.image_size
    for id, obj in self.objects.items():
      offset, pattern, color_pattern, = obj
      offset += shift
      rlim = min(self.image_size, offset + len(pattern))
      for i in range(offset, rlim):
        if pattern[i-offset] == image[i]:
          print "Warn: obj %s collides" % id
        image[i] = pattern[i-offset]
        color_image[i] = color_pattern[i-offset]
    return image, color_image
    
  def transformAndGenerateImage(self, value):
    # non-destructive
    image, color_image = self.generateImage(value)
    return image, color_image
  
  def display(self):
    image, color_image = self.generateImage()
    return " ".join(map(str, image)), " ".join(map(str, color_image))
    
    
class Segmenter:
  def __init__(self, image_size):
    self.image_size = image_size

    # model of the object to be tracked in current frame
    self.model_object = [None]*image_size
    # model of the object in color; only depth info is used to compute this
    self.model_color = [None]*image_size
    # motion detected in current frame; 
    # >0 means has occluded, <0 has revealed, 0 no change
    self.model_motion_map = [None]*image_size

    # these are candidates that could be considered as part
    # of the model by using other priors
    self.model_candidates = [None]*image_size
    self.model_color_candidates = [None]*image_size
    
    self.image_last = None
    self.color_image_last = None
    self.T_last = None
    
  #def projectModelToImage(self):
    #image = [None] * image_size
    #for i in enumerate(range(offset, image_size)):
      
  def accumulate(self, vals_to_add, vals_to_add_color):
    for i, (val, cval) in enumerate(zip(vals_to_add, vals_to_add_color)):
      if val != None:
        self.model_object[i] = val
        self.model_color[i] = cval
    
  def step(self, image, color_image, T=None):
    if self.image_last == None:
      assert (T==None)
      self.image_last = image
      self.color_image_last = color_image
      return
  
    assert (T!=None)
    
    # clean-up model by removing points that are inconsistent in the model
    # (and which are not occluded)
    # caveat: with sensor noise we might actually remove too many points
    for idx, (m_i, i_i) in enumerate(zip(T.apply(self.model_object), image)):
      if m_i > i_i:
        shifted_idx = idx - T.value # hacky
        self.model_object[shifted_idx] = None
        self.model_color[shifted_idx] = None
    
    self.model_object = T.apply(self.model_object)
    self.model_color = T.apply(self.model_color)
    
    # detect motion and update motion map
    motion_map = self.detectMotion(self.image_last, image)
    print "raw motion map"
    print motion_map
    self.model_motion_map = self.mergeMotionMap(motion_map)
    
    # detect color changes
    color_change_map = []
    for i,j in zip(self.color_image_last, color_image):
      if i != j:
        color_change_map.append (1)
      else:
        color_change_map.append (0)
    print "color_change_map"
    print color_change_map

    self.model_candidates = []
    self.model_color_candidates = []
    for img_last, climg_last, img_cur, climg_cur, mm, cm in zip(T.apply(self.image_last), T.apply(self.color_image_last), \
      image, color_image, T.apply(self.model_motion_map), T.apply(color_change_map)):    
      if mm == 0 and img_last == img_cur and cm != 0:  # no change in depth, T-consistent, but color changed
        self.model_candidates.append(img_cur)
        self.model_color_candidates.append(climg_cur)
      else:
        self.model_candidates.append(None)
        self.model_color_candidates.append(None)
    
    # forward prediction and comparison
    # compute T*image_last 
    # keep all points with -1 motion that match in current
    model_add = []
    model_add_color = []
    for img_last, climg_last, img_cur, climg_cur, mm in zip(T.apply(self.image_last), T.apply(self.color_image_last), \
      image, color_image, T.apply(self.model_motion_map)):
      if mm == None or mm >= 0:
        model_add.append(None)
        model_add_color.append(None)
        continue
      # mm < 0:
      if img_last == img_cur:
        model_add.append(img_cur)
        model_add_color.append(climg_cur)
      else:
        model_add.append(None)
        model_add_color.append(None)

    #model_add = T.applyInverse(model_add)
    #print "object -"
    #print model_add
    self.accumulate(model_add, model_add_color)
    
    # backward prediction and comparison
    # compute T^-1*image
    # keep all points with + motion that match in last
    model_add = []
    model_add_color = []
    #print "T.applyInverse(image)"
    #print self.image_last
    #print T.applyInverse(image)
    #print self.model_motion_map
    for img_last, climg_last, img_cur, climg_cur, mm in zip(self.image_last, self.color_image_last, \
      T.applyInverse(image), T.applyInverse(color_image), T.applyInverse(self.model_motion_map)):
      if mm == None or mm <= 0:
        model_add.append(None)
        model_add_color.append(None)
        continue
      # mm > 0:
      if img_last == img_cur:
        model_add.append(img_cur)
        model_add_color.append(climg_cur)
      else:
        model_add.append(None)
        model_add_color.append(None)

    #print "object +"
    #print model_add
    model_add = T.apply(model_add)
    model_add_color = T.apply(model_add_color)

    self.accumulate(model_add, model_add_color)

    # log
    self.image_last = image
    self.color_image_last = color_image
  
  def detectMotion(self, ot1, ot2):
    assert len(ot1)==len(ot2)
    
    res = []
    for i,j in zip(ot1, ot2):
      res.append (j - i)
  
    return res
    
  def mergeMotionMap(self, motion_map):
    res = []
    for i, j in zip(self.model_motion_map, motion_map):
      if i != None and j != None and i*j < 0:
        res.append(0)
      elif j > 0: # always take new map if map is + 
        res.append(j)
      elif i < 0: # always take new map if old map is -
        res.append(j)
      else: # otherwise take bigger value
        res.append (max ([i,j]))
    return res
    
if __name__ == "__main__":
  #obj_main = [ 3, 2, 3 ], [ 'a', 'a', 'b' ]
  #obj_behind = [5,5], ['a', 'b']
  #obj_infront = [4],  ['b']

  obj_main = [ 3, 3, 3 ], [ 'a', 'b', 'c' ]
  obj_behind = [5,5], ['a', 'b']
  obj_infront = [4],  ['b']
  
  image_size = 15
  
  scene = Scene(image_size)
  scene.addObject("main", obj_main[0], obj_main[1], 6)
  #scene.addObject("behind", obj_behind[0], obj_behind[1], 2)
  #scene.addObject("infront", obj_infront[0], obj_infront[1], 9)

  segmenter = Segmenter(image_size)
  segmenter.step(* scene.generateImage())
  
  print "========================="

  print scene.display()

  main_transforms =   [ 1, 2, -4, 3, 2 ]
  behind_transforms = [ 0, 0,  1, 3, 0 ]
  #behind_transforms = [0]*5

  for i, (Tm_val, Tb_val) in enumerate(zip(main_transforms, behind_transforms)):
    print "=================================================="
    print "t=%d; T=%d" % (i+1, Tm_val)
    Tm = Transformation(Tm_val)
    Tm.applyToScene(scene, "main")
    
    #Tb = Transformation(Tb_val)
    #Tb.applyToScene(scene, "behind")
    
    print scene.display()
    print "--------------------------------------------------"
    
    segmenter.step(* scene.generateImage(), T=Tm)
    print "motion map"
    print segmenter.model_motion_map
    print "object"
    print segmenter.model_object, segmenter.model_color
    print "candidates"
    print segmenter.model_candidates, segmenter.model_color_candidates
    
