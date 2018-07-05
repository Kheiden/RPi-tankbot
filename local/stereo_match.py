from camera import Camera

import cv2

class StereoMatch():

    def __init__(self):
        self.c = Camera()

    def main(self):
        processing_time = []
        for i in range(60):
            processing_time01 = cv2.getTickCount()
            imgLeft, disparity = self.c.create_disparity_map()
            disparity = Image.fromarray(disparity)
            disparity.save("/home/pi/input_output/stream_test/{}".format(i), format='JPEG')
            print("Processing time:", cv2.getTickCount() - processing_time01)


if __name__ == '__main__':
    s = StereoMatch()
    s.main()
