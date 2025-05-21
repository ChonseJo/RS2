from flask import Flask, jsonify, render_template, request, redirect, url_for, session, flash
import rclpy
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import Image
from rclpy.node import Node
import socket
from rclpy.qos import QoSProfile, ReliabilityPolicy
from threading import Thread
import time
from werkzeug.security import check_password_hash, generate_password_hash
from flask_wtf import FlaskForm
from wtforms import StringField, PasswordField
from wtforms.validators import InputRequired
from dashboard.srv import CurrentState

# Initialize the Flask app
app = Flask(__name__)
app.secret_key = 'your_secure_secret_key'  # Replace with a secure key from environment variables in production

# Simple form definition using Flask-WTF
class LoginForm(FlaskForm):
    userID = StringField('UserID', validators=[InputRequired()])
    password = PasswordField('Password', validators=[InputRequired()])

# Hashed credentials for comparison
USER_ID = 'admin'
PASSWORD_HASH = generate_password_hash('password')

@app.route('/', methods=['GET', 'POST'])
def login():
    if 'logged_in' in session:
        if session['logged_in']:
            return redirect(url_for('home'))  # Redirect if already logged in
        session.pop('logged_in', None)  # Clean up just in case

    form = LoginForm()
    if form.validate_on_submit():
        if form.userID.data == USER_ID and check_password_hash(PASSWORD_HASH, form.password.data):
            session['logged_in'] = True
            flash('You have successfully logged in.', 'success')
            return redirect(url_for('home'))
        else:
            flash('Invalid userID or password', 'error')
    return render_template('login.html', form=form)

@app.route('/home')
def home():
    if not session.get('logged_in'):
        return redirect(url_for('login'))
    data = data_store.get_data()
    return render_template('fetch.html', title='Robotics Studio 2: Big Booty Bottle Boys', 
                           warning_data=data['warning_data'], 
                           state_data=data['state_data'], 
                           number_of_bottle=data['number_of_bottle'], 
                           type_of_bottle=data['type_of_bottle'],
                           number_of_fanta_bottle=data['number_of_fanta_bottle'], 
                           number_of_cola_bottle=data['number_of_cola_bottle'],
                           number_of_sprite_bottle=data['number_of_sprite_bottle'], 
                           number_of_other_bottle=data['number_of_other_bottle'])

@app.route('/data')
def data():
    if not session.get('logged_in'):
        return jsonify({'error': 'Authentication required'}), 401
    response = jsonify(data_store.get_data())
    return response

@app.route('/e_stop', methods=['POST'])
def e_stop():
    if not session.get('logged_in'):
        return jsonify({'error': 'Authentication required'}), 401
    
    try:
        msg = Bool()
        msg.data = True
        node = rclpy.create_node('e_stop_client')
        pub = node.create_publisher(Bool, '/ur3/e_stop', 1)
        pub.publish(msg)
        node.destroy_node()
        return jsonify({'status': 'success', 'message': 'E-stop activated'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/resume', methods=['POST'])
def resume():
    if not session.get('logged_in'):
        return jsonify({'error': 'Authentication required'}), 401
    
    try:
        msg = Bool()
        msg.data = False
        node = rclpy.create_node('e_stop_client')
        pub = node.create_publisher(Bool, '/ur3/e_stop', 1)
        pub.publish(msg)
        node.destroy_node()
        return jsonify({'status': 'success', 'message': 'System resumed'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/start', methods=['POST'])
def start():
    if not session.get('logged_in'):
        return jsonify({'error': 'Authentication required'}), 401
    
    try:
        msg = Bool()
        msg.data = True
        node = rclpy.create_node('start_system_client')
        pub = node.create_publisher(Bool, '/ur3/start_system', 1)
        pub.publish(msg)
        node.destroy_node()
        return jsonify({'status': 'success', 'message': 'System started'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/logout')
def logout():
    session.pop('logged_in', None)
    flash('You have been logged out.', 'info')
    return redirect(url_for('login'))

class DataStore:
    def __init__(self):
        from threading import Lock
        self._lock = Lock()
        self._data = {'warning_data': 'NO WARNING',
                      'state_data': 'Current state:',
                      'number_of_bottle': 'Number of bottle:',
                      'type_of_bottle': 'Type of bottle',
                      'number_of_fanta_bottle': 'Number of sorted fanta bottle: 0',
                      'number_of_cola_bottle': 'Number of sorted cola bottle: 0',
                      'number_of_sprite_bottle': 'Number of sorted sprite bottle: 0',
                      'number_of_other_bottle': 'Number of sorted other bottle: 0',
                     }

    def update_data(self, message, data_type='warning_data'):
        with self._lock:
            self._data[data_type] = message

    def get_data(self):
        with self._lock:
            return self._data.copy()

data_store = DataStore()

class FlaskROS2Node(Node):
    def __init__(self):
        super().__init__('state_service_node')
        self.data_store = data_store
        
        # QoS profile for reliable communication
        self.reliable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscribe to bottle sequence results with reliable QoS
        self.bottle_sub = self.create_subscription(
            String,
            '/bottle_sequence',
            self.bottle_sequence_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE
            )
        )

        # Subscribe to system state with reliable QoS
        self.state_sub = self.create_subscription(
            String,
            '/ur3/system_state',
            self.state_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE
            )
        )

        self.get_logger().info("Dashboard node initialized and waiting for messages...")

        # Initialize bottle counters
        self.fanta_count = 0
        self.cola_count = 0
        self.sprite_count = 0
        self.other_count = 0
        self.total_count = 0
        
        # E-stop publisher
        self.e_stop_pub = self.create_publisher(
            Bool, 
            '/ur3/e_stop',
            self.reliable_qos
        )
        
        # Start system publisher
        self.start_pub = self.create_publisher(
            Bool,
            '/ur3/start_system',
            self.reliable_qos
        )

        # Services
        self.warning_srv = self.create_service(CurrentState, 'warning_data', self.warning_callback)
        self.current_state_srv = self.create_service(CurrentState, 'current_state', self.current_state_callback)
        self.number_of_bottle_srv = self.create_service(CurrentState, 'number_of_bottles', self.number_of_bottle_callback)
        self.type_of_bottle_srv = self.create_service(CurrentState, 'type_of_bottle', self.type_of_bottle_callback)
        self.fanta_bottle_srv = self.create_service(CurrentState, 'number_of_fanta_bottle', self.fanta_bottle_callback)
        self.cola_bottle_srv = self.create_service(CurrentState, 'number_of_cola_bottle', self.cola_bottle_callback)
        self.sprite_bottle_srv = self.create_service(CurrentState, 'number_of_sprite_bottle', self.sprite_bottle_callback)
        self.other_bottle_srv = self.create_service(CurrentState, 'number_of_other_bottle', self.other_bottle_callback)

        # Subscriber
        self.sub = self.create_subscription(String, 'chatter', self.ros_callback, 10)

    def warning_callback(self, request, response):
        self.data_store.update_data(f"WARNING: {request.input}", 'warning_data')
        response.output = f"WARNING: {request.input}"
        return response

    def current_state_callback(self, request, response):
        self.data_store.update_data(f"Current state: {request.input}", 'state_data')
        response.output = f"Current state: {request.input}"
        return response

    def number_of_bottle_callback(self, request, response):
        self.data_store.update_data(f"Number of bottle: {request.input}", 'number_of_bottle')
        response.output = f"Number of bottle: {request.input}"
        return response

    def type_of_bottle_callback(self, request, response):
        self.data_store.update_data(f"Type of bottle: {request.input}", 'type_of_bottle')
        response.output = f"Type of bottle: {request.input}"
        return response

    def fanta_bottle_callback(self, request, response):
        self.data_store.update_data(f"Number of sorted fanta bottle: {request.input}", 'number_of_fanta_bottle')
        response.output = f"Number of sorted fanta bottle: {request.input}"
        return response

    def cola_bottle_callback(self, request, response):
        self.data_store.update_data(f"Number of sorted cola bottle: {request.input}", 'number_of_cola_bottle')
        response.output = f"Number of sorted cola bottle: {request.input}"
        return response

    def sprite_bottle_callback(self, request, response):
        self.data_store.update_data(f"Number of sorted sprite bottle: {request.input}", 'number_of_sprite_bottle')
        response.output = f"Number of sorted sprite bottle: {request.input}"
        return response

    def other_bottle_callback(self, request, response):
        self.data_store.update_data(f"Number of sorted other bottle: {request.input}", 'number_of_other_bottle')
        response.output = f"Number of sorted other bottle: {request.input}"
        return response

    def bottle_sequence_callback(self, msg):
        try:
            # Parse the bottle sequence (format: "C S F | C S F")
            sequence = msg.data.replace(" | ", " ").split()
            self.get_logger().info(f"Received bottle sequence: {msg.data}")
            
            # Reset counts for new sequence
            self.fanta_count = 0
            self.cola_count = 0
            self.sprite_count = 0
            self.other_count = 0
            self.total_count = 0
            
            # Count bottles in sequence
            for bottle in sequence:
                if bottle == 'F':
                    self.fanta_count += 1
                elif bottle == 'C':
                    self.cola_count += 1
                elif bottle == 'S':
                    self.sprite_count += 1
                elif bottle == '?':
                    self.other_count += 1
                
                self.total_count += 1
            
            # Update the data store
            self.data_store.update_data(f"Number of sorted fanta bottle: {self.fanta_count}", 'number_of_fanta_bottle')
            self.data_store.update_data(f"Number of sorted cola bottle: {self.cola_count}", 'number_of_cola_bottle')
            self.data_store.update_data(f"Number of sorted sprite bottle: {self.sprite_count}", 'number_of_sprite_bottle')
            self.data_store.update_data(f"Number of sorted other bottle: {self.other_count}", 'number_of_other_bottle')
            self.data_store.update_data(f"Number of bottle: {self.total_count}", 'number_of_bottle')
            
            # Update current bottle type being processed
            if self.total_count > 0:
                current_type = sequence[-1]  # Get the last detected bottle
                type_map = {'F': 'Fanta', 'C': 'Cola', 'S': 'Sprite', '?': 'Unknown'}
                self.data_store.update_data(f"Type of bottle: {type_map.get(current_type, 'Unknown')}", 'type_of_bottle')
        except Exception as e:
            self.get_logger().error(f"Error processing bottle sequence: {e}")
            self.data_store.update_data("Error processing bottle sequence", 'warning_data')

    def state_callback(self, msg):
        try:
            self.data_store.update_data(f"Current state: {msg.data}", 'state_data')
            self.get_logger().info(f"System state updated: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error updating system state: {e}")

    def ros_callback(self, message):
        # Legacy callback for chatter topic
        pass

def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def run_flask():
    app.run(host='0.0.0.0', port=5002, debug=False, use_reloader=False, threaded=True)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlaskROS2Node()
        
        # Start Flask in a separate thread
        flask_thread = Thread(target=run_flask)
        flask_thread.daemon = True
        flask_thread.start()
        
        print("Dashboard is running at http://localhost:5002")
        print("Press Ctrl+C to exit")
        
        # Spin ROS2 node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    except Exception as e:
        print(f"\nError occurred: {e}")
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
