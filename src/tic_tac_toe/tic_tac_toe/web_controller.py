#!/usr/bin/env python3
import os
import yaml
import time
import rclpy
import threading
from enum import Enum
from rclpy.node import Node
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from flask import Flask, render_template, request, jsonify
from project.srv import GetBoardState, ComputeBestMove, DrawSymbol
from ament_index_python.packages import get_package_share_directory

package_share_dir = get_package_share_directory('tic_tac_toe')
template_dir = os.path.join(package_share_dir, 'templates')
config_path = os.path.join(package_share_dir, 'config', 'board_params.yaml')

app = Flask(__name__, template_folder=template_dir)

# Game state
class GameState(Enum):
    IDLE = 0
    WAITING_FOR_HUMAN = 1
    PLAYING = 2
    FINISHED = 3

class BoardRecorder:
    """Board state recorder for detecting and validating board changes"""
    def __init__(self):
        self.last_stable_state = [0] * 9    # Last stable board state
        self.current_state = [0] * 9        # Current board state
        self.change_count = 0               # Count of consecutive detected same changes
        self.last_change_position = -1      # Last changed position
        self.last_change_symbol = 0         # Last changed symbol
        self.stable_threshold = 3           # Number of consecutive same changes needed to consider stable
    
    def update(self, new_state):
        """Update board state and detect changes"""
        self.current_state = new_state.copy()
        
        # Detect changes
        changes = []
        for i in range(9):
            if self.last_stable_state[i] != new_state[i] and new_state[i] != 0:
                changes.append((i, new_state[i]))
        
        # If no changes or more than one change, reset counter
        if len(changes) != 1:
            self.change_count = 0
            self.last_change_position = -1
            self.last_change_symbol = 0
            return False
        
        # Get change information
        position, symbol = changes[0]
        
        # If same change, increment counter
        if position == self.last_change_position and symbol == self.last_change_symbol:
            self.change_count += 1
        else:
            # Different change, reset counter
            self.change_count = 1
            self.last_change_position = position
            self.last_change_symbol = symbol
        
        # If change is stable, update last stable state
        if self.change_count >= self.stable_threshold:
            self.last_stable_state = new_state.copy()
            self.change_count = 0
            return True
        
        return False
    
    def get_stable_state(self):
        """Get stable board state"""
        return self.last_stable_state.copy()
    
    def reset(self):
        """Reset recorder"""
        self.last_stable_state = [0] * 9
        self.current_state = [0] * 9
        self.change_count = 0
        self.last_change_position = -1
        self.last_change_symbol = 0

class WebController(Node):
    def __init__(self):
        super().__init__('web_controller')
        self.game_state = GameState.IDLE
        self.current_player = 1  # 1: Human, 2: Arm
        self.board = [0] * 9    # 0: Empty, 1: X, 2: O
        self.arm_first = False  # Whether the arm goes first
        self.human_symbol = 1   # Symbol used by human (1: X, 2: O)
        self.arm_symbol = 2     # Symbol used by arm (1: X, 2: O)
        
        # Physical board state
        self.physical_board_state = [0] * 9
        self.physical_board_ascii = ""
        self.board_update_thread = None
        self.stop_board_update = False
        
        # Board state recorder
        self.board_recorder = BoardRecorder()
        self.last_human_move_time = 0
        self.human_move_timeout = 60  # Timeout for human move (seconds)

        # Create a reentrant callback group to allow service callbacks to call other services without blocking
        self.callback_group = ReentrantCallbackGroup()   
        
        # Service clients
        self.draw_grid_cli = self.create_client(Empty, 'draw_grid', callback_group=self.callback_group)
        self.draw_symbol_cli = self.create_client(DrawSymbol, 'draw_symbol', callback_group=self.callback_group)
        self.press_button_cli = self.create_client(Empty, 'press_button', callback_group=self.callback_group)
        self.move_home_cli = self.create_client(Empty, 'move_home', callback_group=self.callback_group)
        self.get_board_state_cli = self.create_client(GetBoardState, 'get_board_state', callback_group=self.callback_group)
        self.compute_best_move_cli = self.create_client(ComputeBestMove, 'compute_best_move', callback_group=self.callback_group)
        self.update_params_cli = self.create_client(Empty, 'update_parameters', callback_group=self.callback_group)
        self.start_calibration_cli = self.create_client(Empty, 'start_calibration', callback_group=self.callback_group)
        self.next_calibration_step_cli = self.create_client(Empty, 'next_calibration_step', callback_group=self.callback_group)
        self.cancel_calibration_cli = self.create_client(Empty, 'cancel_calibration', callback_group=self.callback_group)
        self.finish_calibration_cli = self.create_client(Empty, 'finish_calibration', callback_group=self.callback_group)
        
        # Wait for services to become available
        while not self.draw_grid_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('draw_grid service not available, waiting again...')
        
        while not self.draw_symbol_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('draw_symbol service not available, waiting again...')
        
        while not self.press_button_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('press_button service not available, waiting again...')
        
        while not self.move_home_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_home service not available, waiting again...')
        
        while not self.get_board_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_board_state service not available, waiting again...')
        
        while not self.compute_best_move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('compute_best_move service not available, waiting again...')
        
        while not self.update_params_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('update_parameters service not available, waiting again...')
        
        while not self.start_calibration_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('start_calibration service not available, waiting again...')
        
        while not self.next_calibration_step_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('next_calibration_step service not available, waiting again...')
        
        while not self.cancel_calibration_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('cancel_calibration service not available, waiting again...')
        
        while not self.finish_calibration_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('finish_calibration service not available, waiting again...')
        
        # Load parameters
        self.load_parameters()
        
        # Start board state update thread
        self.start_board_update_thread()
        
        self.get_logger().info('Web controller initialized')
        self.initialized = True
    
    def start_board_update_thread(self):
        """Start board state update thread"""
        self.stop_board_update = False
        self.board_update_thread = threading.Thread(target=self.update_board_state_loop)
        self.board_update_thread.daemon = True
        self.board_update_thread.start()
        self.get_logger().info('Board state update thread started')
    
    def stop_board_update_thread(self):
        """Stop board state update thread"""
        self.stop_board_update = True
        if self.board_update_thread and self.board_update_thread.is_alive():
            self.board_update_thread.join(timeout=1.0)
        self.get_logger().info('Board state update thread stopped')
    
    def update_board_state_loop(self):
        """Regularly update board state loop"""
        while not self.stop_board_update and rclpy.ok():
            try:
                self.update_physical_board_state()
                
                # If waiting for human move, check board changes and timeout
                if self.game_state == GameState.WAITING_FOR_HUMAN:
                    # Check timeout
                    if time.time() - self.last_human_move_time > self.human_move_timeout:
                        self.get_logger().warn('Human move timeout, resetting game')
                        self.reset_game()
                        continue
                
                time.sleep(1)  # Update every second
            except Exception as e:
                self.get_logger().error(f'Error in board update loop: {e}')
                time.sleep(1)
    
    def update_physical_board_state(self):
        """Update physical board state"""
        if not self.get_board_state_cli.wait_for_service(timeout_sec=0.5):
            return
        
        req = GetBoardState.Request()
        future = self.get_board_state_cli.call_async(req)
        
        # Use non-blocking way to wait for result
        start_time = time.time()
        timeout_sec = 1.0
        
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.1)
        
        if future.done():
            try:
                result = future.result()
                self.physical_board_state = result.board_state.tolist()
                
                # Update board recorder
                stable_change = self.board_recorder.update(self.physical_board_state)

                # If stable change detected and game is in idle state, update internal board state
                if stable_change and self.game_state == GameState.IDLE:
                    self.board = self.board_recorder.get_stable_state()
                
                # If stable change detected and waiting for human move, process human move
                if stable_change and self.game_state == GameState.WAITING_FOR_HUMAN:
                    self.check_and_process_human_move()
                
                self.physical_board_ascii = result.board_ascii
            except Exception as e:
                self.get_logger().error(f'Error getting board state: {e}')
    
    def check_and_process_human_move(self):
        """Check and process human move"""
        # Get stable state
        stable_state = self.board_recorder.get_stable_state()
        
        # Compare with internal board state to find changes
        changes = []
        for i in range(9):
            if self.board[i] == 0 and stable_state[i] != 0:
                changes.append((i, stable_state[i]))
        
        # If only one change, process human move
        if len(changes) == 1:
            position, symbol = changes[0]
            self.process_confirmed_human_move(position, symbol)
        elif len(changes) > 1:
            # Multiple changes, possibly false detection
            self.get_logger().warn(f'Multiple board changes detected: {changes}')
    
    def process_confirmed_human_move(self, position, symbol):
        """Process confirmed human move"""
        # If human goes first and symbol not yet determined
        if not self.arm_first and self.human_symbol is None:
            self.human_symbol = symbol
            self.arm_symbol = 3 - symbol
            self.get_logger().info(f'Human chose symbol: {self.human_symbol}, arm will use: {self.arm_symbol}')
        
        # Update internal board state
        self.board[position] = self.human_symbol
        
        # Change game state
        self.game_state = GameState.PLAYING
        self.current_player = 2  # Switch to arm
        
        # Record move time
        self.last_human_move_time = time.time()
        
        # Execute arm move
        self.arm_move()
    
    def load_parameters(self):
        # Load configuration from parameter file
        try:
            with open(config_path, 'r') as file:
                self.params = yaml.safe_load(file)
                self.get_logger().info('Parameters loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load parameters: {e}')
            exit(1)
    
    def save_parameters(self):
        # Save parameters to file
        try:
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            with open(config_path, 'w') as file:
                yaml.dump(self.params, file)
            self.get_logger().info('Parameters saved successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save parameters: {e}')
            return False
    
    def update_parameter(self, key, value):
        # Update parameter
        keys = key.split('.')
        param_dict = self.params
        
        # Traverse all keys except the last one
        for k in keys[:-1]:
            # If key is a numeric string, convert to integer
            if k.isdigit():
                k = int(k)
            param_dict = param_dict[k]
        
        # Process the last key
        last_key = keys[-1]
        # If the last key is a numeric string, convert to integer
        if last_key.isdigit():
            last_key = int(last_key)
        
        # Try to convert value to appropriate type
        try:
            # If current value is numeric, try to convert new value to same type
            if isinstance(param_dict[last_key], (int, float)):
                param_dict[last_key] = type(param_dict[last_key])(value)
            else:
                param_dict[last_key] = value
        except (ValueError, TypeError):
            # If conversion fails, keep original value
            param_dict[last_key] = value
        
        return self.save_parameters()
    
    def call_service(self, client, service_name, request=None):
        # Call service - use non-blocking way
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'{service_name} service not available')
            return False
        
        if request is None:
            req = Empty.Request()
        else:
            req = request
            
        future = client.call_async(req)
        
        # Wait for future object to complete, set timeout
        start_time = time.time()
        timeout_sec = 5.0
        
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.1)
        
        if future.done():
            try:
                result = future.result()
                return True
            except Exception as e:
                self.get_logger().error(f'{service_name} service call failed: {e}')
                return False
        else:
            self.get_logger().error(f'{service_name} service call timed out')
            return False
    
    def get_board_state(self):
        # Get current board state - now using internally stored state
        return self.physical_board_state, self.physical_board_ascii
    
    def compute_best_move(self, board_state):
        # Calculate best move
        if not self.compute_best_move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('compute_best_move service not available')
            return None, None, None
        
        req = ComputeBestMove.Request()
        req.board_state = board_state
        future = self.compute_best_move_cli.call_async(req)
        
        # Wait for future object to complete, set timeout
        start_time = time.time()
        timeout_sec = 10.0  # Calculation may take longer
        
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.1)
        
        if future.done():
            try:
                result = future.result()
                self.get_logger().info(f'Best move: {result.best_move}')
                self.get_logger().info(f'Player: {result.player}')
                self.get_logger().info(f'Game status: {result.game_status}')
                return result.best_move, result.player, result.game_status
            except Exception as e:
                self.get_logger().error(f'compute_best_move service call failed: {e}')
                return None, None, None
        else:
            self.get_logger().error('compute_best_move service call timed out')
            return None, None, None
    
    def draw_grid(self):
        return self.call_service(self.draw_grid_cli, 'draw_grid')
    
    def draw_symbol(self, position, symbol):
        # Draw symbol
        req = DrawSymbol.Request()
        req.position = position
        req.symbol = symbol
        return self.call_service(self.draw_symbol_cli, 'draw_symbol', req)
    
    def press_button(self):
        return self.call_service(self.press_button_cli, 'press_button')
    
    def move_home(self):
        return self.call_service(self.move_home_cli, 'move_home')
    
    def update_parameters(self):
        return self.call_service(self.update_params_cli, 'update_parameters')
    
    def start_calibration(self):
        return self.call_service(self.start_calibration_cli, 'start_calibration')
    
    def next_calibration_step(self):
        return self.call_service(self.next_calibration_step_cli, 'next_calibration_step')
    
    def cancel_calibration(self):
        return self.call_service(self.cancel_calibration_cli, 'cancel_calibration')
    
    def finish_calibration(self):
        return self.call_service(self.finish_calibration_cli, 'finish_calibration')
    
    def start_game(self, arm_first=False, arm_symbol=None):
        self.arm_first = arm_first
        self.board = [0] * 9
        self.game_state = GameState.PLAYING if arm_first else GameState.WAITING_FOR_HUMAN
        self.board_recorder.reset()
        self.last_human_move_time = time.time()
        
        if arm_first:
            # Arm goes first, use specified symbol
            self.arm_symbol = arm_symbol if arm_symbol is not None else 1
            self.human_symbol = 3 - self.arm_symbol
            self.current_player = 2  # Arm goes first
            self.arm_move()
        else:
            # Human goes first, wait to detect first symbol
            self.human_symbol = None
            self.arm_symbol = None
            self.current_player = 1  # Human goes first
            self.get_logger().info('Waiting for human to make first move...')
        
        return True
    
    def reset_game(self):
        self.press_button()
        self.board = [0] * 9
        self.game_state = GameState.IDLE
        self.current_player = 1
        self.board_recorder.reset()
        return True
    
    def human_move(self, position, symbol=None):
        if self.game_state != GameState.PLAYING or self.current_player != 1:
            return False
        
        if self.board[position] != 0:
            return False
        
        # Update board state
        self.board[position] = self.human_symbol
        
        # Draw corresponding symbol
        success = self.draw_symbol(position, self.human_symbol)
        
        if success:
            # Call compute_best_move service to get game status
            _, _, game_status = self.compute_best_move(self.board)
            
            # Update game state based on game status
            if "wins" in game_status or "draw" in game_status:
                self.game_state = GameState.FINISHED
                # Return to home position after game ends
                self.move_home()
                return True
            
            # Switch player
            self.current_player = 2
            self.arm_move()
        
        return success
    
    def arm_move(self):
        if self.game_state != GameState.PLAYING or self.current_player != 2:
            return False
        
        # Call calculate best move service
        best_move, player, game_status = self.compute_best_move(self.board)

        if best_move is None:
            self.get_logger().error('Failed to compute best move, using fallback')
            # Use fallback strategy: randomly select an empty position
            empty_positions = [i for i, val in enumerate(self.board) if val == 0]
            if not empty_positions:
                self.game_state = GameState.FINISHED
                # Return to home position after game ends
                self.move_home()
                return False
            position = empty_positions[0]
        elif best_move == -1:
            self.game_state = GameState.FINISHED
            # Return to home position after game ends
            self.move_home()
            return False
        else:
            position = best_move
        
        self.board[position] = self.arm_symbol
        self.board_recorder.last_stable_state[position] = self.arm_symbol
        
        # Draw corresponding symbol
        success = self.draw_symbol(position, self.arm_symbol)
        
        if success:
            # Check game status
            if "wins" in game_status or "draw" in game_status:
                self.game_state = GameState.FINISHED
            else:
                # Switch player
                self.current_player = 1
                self.game_state = GameState.WAITING_FOR_HUMAN
                self.last_human_move_time = time.time()
            
            # Return to home position after game ends
            if self.game_state == GameState.FINISHED:
                self.move_home()
        
        return success

# Create ROS2 node
web_controller = None
ros_initialized = False

def init_ros_node():
    global web_controller, ros_initialized
    try:
        rclpy.init()
        web_controller = WebController()
        ros_initialized = True
        app.logger.info("ROS node initialized successfully")
        
        # Keep node running
        rclpy.spin(web_controller)
    except Exception as e:
        app.logger.error(f"Failed to initialize ROS node: {e}")
        ros_initialized = False

# Decorator to wait for ROS node initialization
def require_ros_initialized(func):
    def wrapper(*args, **kwargs):
        if not ros_initialized:
            return jsonify({'error': 'ROS node not initialized'}), 503
        return func(*args, **kwargs)
    wrapper.__name__ = func.__name__
    return wrapper

# Flask routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/start_game', methods=['POST'])
@require_ros_initialized
def api_start_game():
    data = request.json
    arm_first = data.get('arm_first', False)
    arm_symbol = data.get('arm_symbol', None)
    success = web_controller.start_game(arm_first, arm_symbol)
    return jsonify({'success': success})

@app.route('/api/reset_game', methods=['POST'])
@require_ros_initialized
def api_reset_game():
    success = web_controller.reset_game()
    return jsonify({'success': success})

@app.route('/api/human_move', methods=['POST'])
@require_ros_initialized
def api_human_move():
    data = request.json
    position = data.get('position')
    symbol = data.get('symbol', None)
    success = web_controller.human_move(position, symbol)
    return jsonify({'success': success})

@app.route('/api/draw_grid', methods=['POST'])
@require_ros_initialized
def api_draw_grid():
    success = web_controller.draw_grid()
    return jsonify({'success': success})

@app.route('/api/draw_symbol', methods=['POST'])
@require_ros_initialized
def api_draw_symbol():
    data = request.json
    position = data.get('position')
    symbol = data.get('symbol')
    success = web_controller.draw_symbol(position, symbol)
    return jsonify({'success': success})

@app.route('/api/press_button', methods=['POST'])
@require_ros_initialized
def api_press_button():
    success = web_controller.press_button()
    return jsonify({'success': success})

@app.route('/api/move_home', methods=['POST'])
@require_ros_initialized
def api_move_home():
    success = web_controller.move_home()
    return jsonify({'success': success})

@app.route('/api/update_params', methods=['POST'])
@require_ros_initialized
def api_update_params():
    data = request.json
    success = True
    errors = []
    
    for key, value in data.items():
        if not web_controller.update_parameter(key, value):
            success = False
            errors.append(f"Failed to update parameter: {key}")
    
    # Notify arm_controller to update parameters
    if success:
        if web_controller.update_params_cli.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            future = web_controller.update_params_cli.call_async(req)
            
            # Wait for future object to complete, set timeout
            start_time = time.time()
            timeout_sec = 5.0
            
            while not future.done() and (time.time() - start_time) < timeout_sec:
                time.sleep(0.1)
            
            if future.done():
                try:
                    future.result()
                    app.logger.info('Parameters updated successfully on arm controller')
                except Exception as e:
                    app.logger.error(f'Failed to update parameters on arm controller: {e}')
                    success = False
                    errors.append(f"Failed to update parameters on arm controller: {e}")
            else:
                app.logger.error('Update parameters service call timed out')
                success = False
                errors.append('Update parameters service call timed out')
        else:
            app.logger.error('update_parameters service not available')
            success = False
            errors.append('update_parameters service not available')
    
    if success:
        return jsonify({'success': True})
    else:
        return jsonify({'success': False, 'errors': errors}), 500

@app.route('/api/get_params', methods=['GET'])
@require_ros_initialized
def api_get_params():
    return jsonify(web_controller.params)

@app.route('/api/game_state', methods=['GET'])
@require_ros_initialized
def api_game_state():
    return jsonify({
        'state': web_controller.game_state.name,
        'current_player': web_controller.current_player,
        'board': web_controller.board,
        'arm_first': web_controller.arm_first,
        'human_symbol': web_controller.human_symbol,
        'arm_symbol': web_controller.arm_symbol
    })

@app.route('/api/ros_status', methods=['GET'])
def api_ros_status():
    return jsonify({'initialized': ros_initialized})

@app.route('/api/start_calibration', methods=['POST'])
@require_ros_initialized
def api_start_calibration():
    success = web_controller.start_calibration()
    return jsonify({'success': success})

@app.route('/api/next_calibration_step', methods=['POST'])
@require_ros_initialized
def api_next_calibration_step():
    success = web_controller.next_calibration_step()
    return jsonify({'success': success})

@app.route('/api/cancel_calibration', methods=['POST'])
@require_ros_initialized
def api_cancel_calibration():
    success = web_controller.cancel_calibration()
    return jsonify({'success': success})

@app.route('/api/finish_calibration', methods=['POST'])
@require_ros_initialized
def api_finish_calibration():
    success = web_controller.finish_calibration()
    return jsonify({'success': success})

def main(args=None):
    # Initialize ROS node in background thread
    ros_thread = threading.Thread(target=init_ros_node)
    ros_thread.daemon = True
    ros_thread.start()
    
    # Wait for ROS node initialization to complete
    max_wait_time = 30  # Maximum wait time (seconds)
    wait_time = 0
    while not ros_initialized and wait_time < max_wait_time:
        app.logger.info("Waiting for ROS node to initialize...")
        time.sleep(1)
        wait_time += 1
    
    if not ros_initialized:
        app.logger.error("ROS node failed to initialize within the expected time")
    
    # Start Flask application
    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == '__main__':
    main()