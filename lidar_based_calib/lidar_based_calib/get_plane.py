import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from skimage.measure import LineModelND, ransac
from rclpy.executors import ExternalShutdownException
import matplotlib.pyplot as plt


import cv2
import numpy as np

class PlaneExtractor(Node):
    def __init__(self, 
                 node_name, 
                 *, 
                 context = None, 
                 cli_args = None, 
                 namespace = None, 
                 use_global_arguments = True, 
                 enable_rosout = True, 
                 start_parameter_services = True, 
                 parameter_overrides = None, 
                 allow_undeclared_parameters = False, 
                 automatically_declare_parameters_from_overrides = False
        ):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.listen_callback, 2)
        # Publisher of distance to calibrating plane, the values are [A,B,C,D, Dist], where A,B,C,D are plane equation coeff's and Dist is minimal distance from lidar to plane
        self.dist_publisher = self.create_publisher(Float64MultiArray, 'dist', 2)
        self.angles: list[float] = list()

    @staticmethod
    def get_angles(angle_min: float, angle_max: float, increment: float) -> list[float]:
        curr_angle: float = angle_min
        res: list[float] = list()
        while curr_angle <= angle_max:
            res.append(curr_angle)
            curr_angle += increment
        return res

    def listen_callback(self, scan_msg: LaserScan):
        if not self.angles:
            self.angles = self.get_angles(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        x: np.array = np.array([dist*np.cos(angle) for dist, angle in zip(scan_msg.ranges, self.angles) if not np.isnan(dist) and not np.isinf(dist)]).reshape(-1,1)
        y: np.array = np.array([dist*np.sin(angle) for dist, angle in zip(scan_msg.ranges, self.angles) if not np.isnan(dist) and not np.isinf(dist)]).reshape(-1,1)
        data = np.column_stack([x, y])

        model = LineModelND()
        model.estimate(data)
        # robustly fit line only using inlier data with RANSAC algorithm
        model_robust, inliers = ransac(data, LineModelND, min_samples=10,
                                    residual_threshold=0.5, max_trials=1000)
        outliers = inliers == False

        # generate coordinates of estimated models
        line_x = np.arange(x.min(),x.max())
        line_y = model.predict_y(line_x)
        line_y_robust = model_robust.predict_y(line_x)
        # img = np.ones((len(x),len(y),3), np.uint8)
        # img *=255
        # for i, j in zip(line_x, line_y):
        #     print(i,j)
        #     img[int(i)][int(j)] = [0,0,0]
        # # line_y_robust = model_robust.predict_y(line_x)
        # # print(line_y)
        # cv2.imshow('line', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # dist = Float64MultiArray()
        # dist.data = [0.0,0.0,0.0,0.0]
        # self.dist_publisher.publish(dist)
        fig, ax = plt.subplots()
        
        ax.plot(data[outliers, 0], data[outliers, 1], '.r', alpha=0.6,
                label='Outlier data')
        ax.plot(data[inliers, 0], data[inliers, 1], '.b', alpha=0.6,
                label='Inlier data')
        print("data: ", data)
        print(data[inliers, 0], data[inliers, 1])
        ax.plot(line_x, line_y, '-k', label='Line model from all data')
        ax.plot(line_x, line_y_robust, '-b', label='Robust line model')
        ax.legend(loc='lower left')
        plt.show()

def main():
    rclpy.init(args=None)
    node = PlaneExtractor('plane_extractor0')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('')
        node.get_logger().info('Shutting Down...')
        node.destroy_node()
    finally:
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()