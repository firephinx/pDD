import cv2
import numpy
import struct
import math

def rescale_depth_image(im, out_path = None):
    '''
    Rescales a depth image to be between 0-255 for visualization purposes
    '''
    max_value = 2**13
    
    if isinstance(im, basestring):
        im = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    
    out_im = numpy.zeros(im.shape, dtype='uint8')
    print out_im.shape
    for i in xrange(im.shape[0]):
        for j in xrange(im.shape[1]):
            value = im[i,j]
            out_im[i,j] = int(float(value)/max_value * 255)
    
    if out_path is not None:
        cv2.imwrite(out_path, out_im)
    
    return out_im

def get_color_depth_mapping():
    '''
    Captures a live depth, color and mapping image from the Kinect
    '''
    import rospy
    import cv_bridge
    import sensor_msgs.msg as sm
    import time
    rospy.init_node('stupid_node', anonymous=True)
    time.sleep(1)
    
    topics = ['/head/kinect2/rgb/image_color',
              '/head/kinect2/depth/image_depth',
              '/head/kinect2/mapping/depth_mapping']
    images = []
    for topic in topics:
        message = rospy.wait_for_message(
                topic, sm.Image)
        print 'Got topic', topic
        bridge = cv_bridge.CvBridge()
        images.append(numpy.asarray(bridge.imgmsg_to_cv(message)[:]))
    
    return images

def process_mapping(mapping):
    
    if isinstance(mapping, basestring):
        mapping = cv2.imread(mapping)
    
    result = numpy.zeros((mapping.shape[0], mapping.shape[1], 2), dtype=float)
    for y in xrange(mapping.shape[0]):
        for x in xrange(mapping.shape[1]):
            rgb_x_bytes = struct.pack('HH', mapping[y,x,0], mapping[y,x,1])
            rgb_y_bytes = struct.pack('HH', mapping[y,x,2], mapping[y,x,3])
            result[y,x,0] = struct.unpack('f', rgb_x_bytes)
            result[y,x,1] = struct.unpack('f', rgb_y_bytes)
    
    return result

def align_depth_from_mapping(color,
                             depth,
                             mapping,
                             scale_factor,
                             out_color_path,
                             out_depth_path):
    
    if isinstance(color, basestring):
        color = cv2.imread(color, cv2.IMREAD_UNCHANGED)
    
    if isinstance(depth, basestring):
        depth = cv2.imread(depth, cv2.IMREAD_UNCHANGED)
    
    if isinstance(mapping, basestring):
        mapping = cv2.imread(mapping, cv2.IMREAD_UNCHANGED)
    
    if mapping.dtype == 'uint8':
        mapping = process_mapping(mapping);
    
    output = numpy.zeros((1080*scale_factor,
                          1920*scale_factor,
                          1), dtype='uint16')
    
    for y in xrange(depth.shape[0]):
        for x in xrange(depth.shape[1]):
            raw = depth[y,x]
            if raw == 0:
                continue
            
            if math.isinf(mapping[y,x,0]) or math.isinf(mapping[y,x,0]):
                continue
            
            address = [int(mapping[y,x,0]*scale_factor),
                       int(mapping[y,x,0]*scale_factor)];
            
            if (address[0] >= 0 and address[0] < output.shape[0] and
                address[1] >= 0 and address[1] < output.shape[1]):
                
                if (output[address[0], address[1]] == 0 or
                    output[address[0], address[1]] > raw):
                    output[address[0], address[1]] = raw
    
    '''
    rescale_depth_image(output, out_depth_path.replace('.', '_rescaled.'))
    rescale_depth_image(depth, out_depth_path.replace('.', '_orig.'))
    
    cv2.imwrite(out_color_path, color)
    cv2.imwrite(out_depth_path, output)
    cv2.imwrite('test_images/mapping.png', mapping)
    '''
    
    return output
