#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from project.srv import GetBoardState
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

def gradient_based_thresholding(image, gradient_threshold=30, intensity_threshold=100):
    '''Gradient-based thresholding'''    
    # Calculate gradient
    sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)
    gradient_magnitude = np.sqrt(sobelx**2 + sobely**2)
    
    # Normalize gradient
    gradient_magnitude = cv2.normalize(gradient_magnitude, None, 0, 255, cv2.NORM_MINMAX)
    
    # Create masks based on gradient and intensity
    gradient_mask = gradient_magnitude > gradient_threshold
    intensity_mask = image > intensity_threshold
    
    # Combine both masks
    combined_mask = np.logical_and(gradient_mask, intensity_mask).astype(np.uint8) * 255
    
    return combined_mask

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group)
        
        self.publisher_ = self.create_publisher(Image, 'image_processed', 10)
        
        self.bridge = CvBridge()
        
        self.board_state = [0] * 9
        self.service = self.create_service(GetBoardState, 'get_board_state', self.get_board_state_callback)
        
        self.board_wh_rotio = 15.4 / 20.35

        # Create a window and set mouse callback for point selection
        cv2.namedWindow("Manually select points")
        cv2.setMouseCallback("Manually select points", self.mouse_callback)
        
        # Initialize points as empty list
        self.src_pts = []
        self.selected_point_index = 0
        self.current_image = None
        self.point_selection_image = None
        self.is_auto_select = False
        
        self.get_logger().info(f'Image processing started')
        self.get_logger().info('Click on the "Manually select points" window to select points 1-4 in order')
        self.get_logger().info('Press "a" for automatic point selection')
        self.get_logger().info('Press "r" to reset')
        
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for point selection"""
        if event == cv2.EVENT_LBUTTONDOWN and self.current_image is not None:
            # Add the clicked point to the list
            if len(self.src_pts) < 4:
                self.is_auto_select = False
                self.src_pts.append([float(x), float(y)])
                self.get_logger().info(f'Added point {len(self.src_pts)} at [{x}, {y}]')
                
                # Immediately redraw the image with updated points
                self.draw_points()

                if len(self.src_pts) == 4:
                    cv2.destroyWindow("Automatically select points")
    
    def auto_select_points(self, image):
        """Automatically select points by detecting the largest black area (board)"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply adaptive threshold to detect dark areas
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        
        # Apply morphological operations to connect nearby regions
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            self.get_logger().warn("No contours found for automatic point selection")
            return
        
        # Find the largest contour (assuming it's the board)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)
        
        # If we don't have 4 points, try a different epsilon
        if len(approx) != 4:
            epsilon = 0.05 * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)
            
        if len(approx) != 4:
            self.get_logger().warn(f"Could not find 4 corners, found {len(approx)}")
            return
        
        # Convert to list of points
        points = [point[0] for point in approx]
        
        # Sort points in the order: top-left, top-right, bottom-right, bottom-left
        # First, sort by y-coordinate to separate top and bottom rows
        points = sorted(points, key=lambda x: x[1])
        
        # Top row (first two points)
        top_row = sorted(points[:2], key=lambda x: x[0])
        # Bottom row (last two points)
        bottom_row = sorted(points[2:], key=lambda x: x[0], reverse=True)
        
        # Combine in correct order
        self.src_pts = [
            [float(top_row[0][0]), float(top_row[0][1])],  # Top-left
            [float(top_row[1][0]), float(top_row[1][1])],  # Top-right
            [float(bottom_row[0][0]), float(bottom_row[0][1])],  # Bottom-right
            [float(bottom_row[1][0]), float(bottom_row[1][1])]   # Bottom-left
        ]
        
        self.get_logger().info("Automatically selected points:")
        for i, pt in enumerate(self.src_pts):
            self.get_logger().info(f"Point {i+1}: [{pt[0]:.1f}, {pt[1]:.1f}]")
        
        # Update display
        self.draw_points()
        self.is_auto_select = True
    
    def draw_points(self):
        """Draw the current points on the image"""
        if self.current_image is not None:
            # Create a copy of the current image for drawing
            self.point_selection_image = self.current_image.copy()
            
            # Draw all selected points
            for i, point in enumerate(self.src_pts):
                x, y = int(point[0]), int(point[1])
                color = (0, 255, 0)  # Green for all selected points
                cv2.circle(self.point_selection_image, (x, y), 5, color, -1)
                cv2.putText(self.point_selection_image, str(i+1), (x+15, y+5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            
            # Draw lines between selected points
            if len(self.src_pts) > 1:
                for i in range(len(self.src_pts) - 1):
                    pt1 = (int(self.src_pts[i][0]), int(self.src_pts[i][1]))
                    pt2 = (int(self.src_pts[i+1][0]), int(self.src_pts[i+1][1]))
                    cv2.line(self.point_selection_image, pt1, pt2, (255, 0, 0), 2)
            
            # If all four points are selected, connect the last to the first
            if len(self.src_pts) == 4:
                pt1 = (int(self.src_pts[3][0]), int(self.src_pts[3][1]))
                pt2 = (int(self.src_pts[0][0]), int(self.src_pts[0][1]))
                cv2.line(self.point_selection_image, pt1, pt2, (255, 0, 0), 2)
            
            # Add instructions
            if len(self.src_pts) < 4:
                cv2.putText(self.point_selection_image, f"Click to select point {len(self.src_pts) + 1}", 
                           (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(self.point_selection_image, "All points selected.", 
                           (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.putText(self.point_selection_image, "Press 'a' for automatic point selection", 
                       (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.putText(self.point_selection_image, "Press 'r' to reset", 
                       (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Manually select points", self.point_selection_image)
        
    def image_callback(self, msg):
        try:
            # Convert the ROS image message to a BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
            
            # Update point selection window
            self.draw_points()

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        pts_len = len(self.src_pts)

        if pts_len != 4:
            # Apply adaptive threshold to detect dark areas
            thresh = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
            
            # Apply morphological operations to connect nearby regions
            kernel = np.ones((5, 5), np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            cv2.imshow("Automatically select points", thresh)

        elif pts_len == 4:
            h = cv_image.shape[0]
            w = round(h * self.board_wh_rotio)

            # Define destination points for the warp transform
            dst_pts = np.array([[0, 0], [w, 0], 
                                [w, h], 
                                [0, h]], dtype=np.float32).reshape(4, 2)
            
            # Convert src_pts to numpy array
            src_pts_np = np.array(self.src_pts, dtype=np.float32).reshape(4, 2)
            
            # Calculate the perspective transform matrix
            try:
                M = cv2.getPerspectiveTransform(src_pts_np, dst_pts)
            except Exception as e:
                self.get_logger().error(f"Perspective transform calculation failed: {e}")
                return

            # Apply the perspective transform to the grayscale image
            warped_gray_image = cv2.warpPerspective(gray_image, M, (w, h))

            # Segment the image into 9 segments
            segmented_image, num_labels, labels_im, grid_bbox = self.image_segmentation(warped_gray_image)

            # cv2.imshow("processed", segmented_image)

            processed_image_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding='bgr8')
            
            self.publisher_.publish(processed_image_msg)

        # Check for keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):  # Reset point selection
            # if pts_len == 4:
            #     cv2.destroyWindow("processed")
            self.src_pts = []
            self.point_selection_image = None
            self.get_logger().info('Reset point selection')
        elif key == ord('a') and self.current_image is not None and self.src_pts == []:  # Automatic point selection
            self.auto_select_points(self.current_image)
            if len(self.src_pts) == 4:
                cv2.destroyWindow("Automatically select points")
        
    def image_segmentation(self, img):
        # Ensure the input image is a numpy array
        if not isinstance(img, np.ndarray):
            raise ValueError("Input image must be a numpy array")

        # Apply binary threshold
        binary_img = gradient_based_thresholding(img)

        # Crop the image to remove the edges
        if self.is_auto_select:
            upper_edge = int(binary_img.shape[0] * 0.01)
            lower_edge = int(binary_img.shape[0] * 0.98)
            left_edge = int(binary_img.shape[1] * 0.02)
            right_edge = int(binary_img.shape[1] * 0.98)
            binary_img = binary_img[upper_edge:lower_edge, left_edge:right_edge]

        # Apply dilation
        kernel = np.ones((3, 3), np.uint8)
        dilated_img = cv2.dilate(binary_img, kernel, iterations=1)

        # Rotate the image
        dilated_img = cv2.rotate(dilated_img, cv2.ROTATE_90_CLOCKWISE)

        # Label connected components
        num_labels, labels_im = cv2.connectedComponents(dilated_img)

        # Create an output image with colored segments
        output_image = cv2.cvtColor(dilated_img, cv2.COLOR_GRAY2BGR)

        if num_labels <= 1:
            # No components found
            return output_image, num_labels, labels_im, None

        # Area filtering: Calculate area of each connected region, filter out regions with area too small
        min_area_threshold = 400  # Set minimum area threshold, adjust as needed
        
        # Create a mask to mark regions to keep
        valid_components_mask = np.zeros_like(labels_im, dtype=bool)
        
        for label in range(1, num_labels):
            area = np.sum(labels_im == label)
            if area >= min_area_threshold:
                valid_components_mask |= (labels_im == label)
            else:
                # Remove small areas from labels
                labels_im[labels_im == label] = 0
        
        # Update label image, keep only valid regions
        labels_im[~valid_components_mask] = 0
        
        # Re-label connected regions (optional, if continuous labels are needed)
        # Here we choose not to re-label as subsequent processing will skip label 0

        # Recalculate number of valid labels
        valid_labels = np.unique(labels_im)
        num_valid_labels = len(valid_labels) - 1  # Subtract background label 0
        
        if num_valid_labels <= 0:
            return output_image, num_valid_labels, labels_im, None

        # Find the largest connected region (grid)
        # Note: We only consider regions with area above threshold
        largest_component = 0
        max_area = 0
        for label in valid_labels[1:]:  # Skip background label 0
            area = np.sum(labels_im == label)
            if area > max_area:
                max_area = area
                largest_component = label

        # Get grid bounding box
        grid_mask = (labels_im == largest_component).astype(np.uint8) * 255
        x, y, w, h = cv2.boundingRect(grid_mask)
        grid_bbox = (x, y, w, h)

        # Initialize board state
        board_state = [0] * 9

        # Color markers for components
        cross_n = 0
        circle_n = 0

        # Only process valid labels (regions with area above threshold)
        for label in valid_labels[1:]:  # Skip background label 0
            if label == largest_component:
                # Mark largest connected region (grid) in white
                output_image[labels_im == label] = [255, 255, 0]  
            else:
                # Extract component
                component_mask = (labels_im == label).astype(np.uint8) * 255
                
                # Calculate geometric center
                x, y, w, h = cv2.boundingRect(component_mask)
                cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

                middle_x = x + w // 2
                middle_y = y + h // 2
                x_1_3 = x + w // 3
                x_2_3 = x + 2 * w // 3
                y_1_3 = y + h // 3
                y_2_3 = y + 2 * h // 3

                # Draw center point
                cv2.circle(output_image, (middle_x, middle_y), 5, (0, 255, 0), -1)

                # Horizontal segments
                horizontal_left = component_mask[middle_y:middle_y + 1, x:x_1_3]
                horizontal_center = component_mask[middle_y:middle_y + 1, x_1_3:x_2_3]
                horizontal_right = component_mask[middle_y:middle_y + 1, x_2_3:(x+w)]
                
                # Vertical segments
                vertical_top = component_mask[y:y_1_3, middle_x:middle_x + 1]
                vertical_center = component_mask[y_1_3:y_2_3, middle_x:middle_x + 1]
                vertical_bottom = component_mask[y_2_3:(y+h), middle_x:middle_x + 1]

                # Condition checks
                center_empty = (np.count_nonzero(horizontal_center) == 0) and (np.count_nonzero(vertical_center) <= 100)
                margins_empty = (np.count_nonzero(horizontal_left) <= 15) and (np.count_nonzero(horizontal_right) <= 15) and \
                                (np.count_nonzero(vertical_top) <= 15) and (np.count_nonzero(vertical_bottom) <= 15)

                # Check if center has white pixels
                if center_empty:
                    output_image[labels_im == label] = [0, 255, 0]  # Green for circle
                    circle_n += 1
                    self.update_board_state(board_state, middle_x, middle_y, grid_bbox, 2)  # O
                
                elif margins_empty:
                    output_image[labels_im == label] = [0, 0, 255]  # Red for cross
                    cross_n += 1
                    self.update_board_state(board_state, middle_x, middle_y, grid_bbox, 1)  # X

                else:
                    continue

        # Add text annotations
        board_present = num_valid_labels > 0
        board_text = f"Board: {int(board_present)}"
        cross_text = f"Cross: {cross_n}"
        circle_text = f"Circle: {circle_n}"

        cv2.putText(output_image, board_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(output_image, cross_text, (110, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(output_image, circle_text, (210, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        self.board_state = board_state
        return output_image, num_valid_labels, labels_im, grid_bbox

    def update_board_state(self, board_state, x, y, grid_bbox, value):
        grid_x, grid_y, grid_w, grid_h = grid_bbox
        cell_width = grid_w // 3
        cell_height = grid_h // 3

        # Determine cell position relative to grid bounding box
        col = (x - grid_x) // cell_width
        row = (y - grid_y) // cell_height

        if 0 <= row < 3 and 0 <= col < 3:
            board_state[row * 3 + col] = value

    def get_board_state_callback(self, request, response):
        self.get_logger().info('Received request for board state.')
        response.board_state = self.board_state
        response.board_ascii = self.generate_ascii_board()
        print(response.board_ascii)
        return response
    
    def generate_ascii_board(self):
        symbols = [' ', 'X', 'O']
        ascii_board = ""
        for i in range(3):
            row = "|".join([symbols[self.board_state[i * 3 + j]] for j in range(3)])
            ascii_board += row + "\n"
            if i < 2:
                ascii_board += "-----\n"
        return ascii_board

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()