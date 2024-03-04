import argparse
import Image_Processing as proc
import cv2
import time

class DetectBall():
    def __init__(self, args):
        self.image_ip = args.image_ip

        self.tuning_mode = args.tuning_mode
        self.tuning_params = {
            'x_min': args.x_min,
            'x_max': args.x_max,
            'y_min': args.y_min,
            'y_max': args.y_max
        }
        self.tuning_params['h_min'] = getattr(args, 'h_min')
        self.tuning_params['h_max'] = getattr(args, 'h_max')
        self.tuning_params['s_min'] = getattr(args, 's_min')
        self.tuning_params['s_max'] = getattr(args, 's_max')
        self.tuning_params['v_min'] = getattr(args, 'v_min')
        self.tuning_params['v_max'] = getattr(args, 'v_max')
        self.tuning_params['sz_min'] = getattr(args, 'sz_min')
        self.tuning_params['sz_max'] = getattr(args, 'sz_max')

        if self.tuning_mode:
            proc.create_tuning_window(self.tuning_params)

    def callback(self):
        
        
        cap = cv2.VideoCapture(self.image_ip)
        # time.sleep(60)

        if not cap.isOpened():
            print("Error: Could not open stream.")
        else:
            print("Stream opened successfully.")
        while cap.isOpened():
            ret, cv_image = cap.read()
            if self.tuning_mode:
                self.tuning_params = proc.get_tuning_params()
            if ret:
                # cv2.imshow('Frame', cv_image)
                keypoints_norm, out_image, tuning_image = proc.find_object(cv_image, self.tuning_params)
                # print(type(tuning_image))  # Check the type of tuning_image
                # print(tuning_image.shape) 
                cv2.imshow('tuining', tuning_image)
                cv2.imshow('out', out_image)

                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
            else:
                break

        cap.release()
        cv2.destroyAllWindows()
rtmp_url = 'rtmp://192.168.137.142/live/stream'
def main():
    parser = argparse.ArgumentParser(description="Detect Ball")
    parser.add_argument('--image_ip', type=str, default=rtmp_url, help='IP address of the image stream')
    parser.add_argument('--tuning_mode', type=bool, default=False, help='Enable tuning mode')
    parser.add_argument('--x_min', type=int, default=0, help='Minimum value of x')
    parser.add_argument('--x_max', type=int, default=100, help='Maximum value of x')
    parser.add_argument('--y_min', type=int, default=0, help='Minimum value of y')
    parser.add_argument('--y_max', type=int, default=100, help='Maximum value of y')
    parser.add_argument('--h_min', type=int, default=29, help='Minimum value of hue for object')
    parser.add_argument('--h_max', type=int, default=69, help='Maximum value of hue for object')
    parser.add_argument('--s_min', type=int, default=14, help='Minimum value of saturation for object')
    parser.add_argument('--s_max', type=int, default=160, help=f'Maximum value of saturation for object')
    parser.add_argument('--v_min', type=int, default=38, help='Minimum value of value for object')
    parser.add_argument('--v_max', type=int, default=239, help='Maximum value of value for object')
    parser.add_argument('--sz_min', type=int, default=23, help='Minimum size of object')
    parser.add_argument('--sz_max', type=int, default=54, help='Maximum size of object')

    args = parser.parse_args()
    detect_ball = DetectBall(args)
    detect_ball.callback()

if __name__ == '__main__':
    main()
