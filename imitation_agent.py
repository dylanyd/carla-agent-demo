import os
import scipy
import numpy as np

import tensorflow as tf
slim = tf.contrib.slim

import carla
from imitation_agent_network import load_imitation_learning_network
from srunner.autoagents.autonomous_agent import AutonomousAgent
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from skimage.transform import resize

class ImitationAgent(AutonomousAgent):


    def __init__(self, ego_vehicle, city_name, avoid_stopping, memory_fraction=0.25, image_cut=[115, 510]):
        super().__init__(path_to_conf_file="")
        self.ego_vehicle = ego_vehicle
        self.vehicle_control = carla.VehicleControl()
        print('vehicle_control', self.vehicle_control)
        self.agent_engaged = False
        self.prev_timestamp = 0
        self.current_data = None
        self.current_speed = None
        self.dropout_vec = [1.0] * 8 + [0.7] * 2 + [0.5] * 2 + [0.5] * 1 + [0.5, 1.] * 5

        # config_gpu = tf.ConfigProto()
        # config_gpu.gpu_options.per_process_gpu_memory_fraction = memory_fraction

        self._image_size = (88, 200, 3)
        self._avoid_stopping = avoid_stopping

        self._sess = tf.Session(config=tf.ConfigProto(
      allow_soft_placement=True, log_device_placement=True))

        with tf.device('/gpu:0'):
            self._input_images = tf.placeholder("float", shape=[None, self._image_size[0],
                                                                self._image_size[1],
                                                                self._image_size[2]],
                                                name="input_image")

            self._input_data = []

            self._input_data.append(tf.placeholder(tf.float32,
                                                   shape=[None, 4], name="input_control"))

            self._input_data.append(tf.placeholder(tf.float32,
                                                   shape=[None, 1], name="input_speed"))

            self._dout = tf.placeholder("float", shape=[len(self.dropout_vec)])

        with tf.name_scope("Network"):
            self._network_tensor = load_imitation_learning_network(self._input_images,
                                                                   self._input_data,
                                                                   self._image_size, self._dout)

        self._models_path = 'model/'

        # tf.reset_default_graph()
        self._sess.run(tf.global_variables_initializer())

        self.load_model()

        self._image_cut = image_cut

    def sensors(self):
        sensors = [{'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                    'width': 800, 'height': 600, 'fov': 100, 'id': 'Center'},
                   {'type': 'sensor.other.gnss', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'id': 'GPS'}
                   ]

        return sensors
    def load_model(self):

        variables_to_restore = tf.global_variables()

        saver = tf.train.Saver(variables_to_restore, max_to_keep=0)

        if not os.path.exists(self._models_path):
            raise RuntimeError('failed to find the models path')

        ckpt = tf.train.get_checkpoint_state(self._models_path)
        if ckpt:
            print('Restoring from ', ckpt.model_checkpoint_path)
            saver.restore(self._sess, ckpt.model_checkpoint_path)
        else:
            ckpt = 0

        return ckpt

    def run_step(self, input_data, timestamp):
        self.agent_engaged = True
        self.current_data = input_data
        control = self._compute_action(input_data['Center'][1][:, :, -2::-1],
                                       self.current_speed)
        self.prev_timestamp = timestamp
        return control


    def _compute_action(self, rgb_image, speed, direction=None):
        rgb_image = rgb_image[self._image_cut[0]:self._image_cut[1], :]

        image_input = resize(rgb_image, [self._image_size[0],
                                                      self._image_size[1]])

        image_input = image_input.astype(np.float32)
        image_input = np.multiply(image_input, 1.0 / 255.0)

        steer, acc, brake = self._control_function(image_input, speed, direction, self._sess)
        print("Prediction: steer %f, acc %f, brake %f" % (steer, acc, brake))
        # This a bit biased, but is to avoid fake breaking

        if brake < 0.1:
            brake = 0.0

        if acc > brake:
            brake = 0.0

        # We limit speed to 35 km/h to avoid
        if speed > 10.0 and brake == 0.0:
            acc = 0.0

        self.vehicle_control.steer = float(steer)
        self.vehicle_control.throttle = float(acc)
        self.vehicle_control.brake = float(brake)

        self.vehicle_control.hand_brake = 0
        self.vehicle_control.reverse = 0

        return self.vehicle_control

    def _control_function(self, image_input, speed, control_input, sess):

        branches = self._network_tensor
        x = self._input_images
        dout = self._dout
        input_speed = self._input_data[1]

        image_input = image_input.reshape(
            (1, self._image_size[0], self._image_size[1], self._image_size[2]))

        # Normalize with the maximum speed from the training set ( 90 km/h)
        speed = np.array(speed / 25.0)

        speed = speed.reshape((1, 1))

        if control_input == 2 or control_input == 0.0:
            all_net = branches[0]
        elif control_input == 3:
            all_net = branches[2]
        elif control_input == 4:
            all_net = branches[3]
        else:
            all_net = branches[1]

        feedDict = {x: image_input, input_speed: speed, dout: [1] * len(self.dropout_vec)}

        output_all = sess.run(all_net, feed_dict=feedDict)

        predicted_steers = (output_all[0][0])

        predicted_acc = (output_all[0][1])

        predicted_brake = (output_all[0][2])

        if self._avoid_stopping:
            predicted_speed = sess.run(branches[4], feed_dict=feedDict)
            predicted_speed = predicted_speed[0][0]
            real_speed = speed * 25.0

            real_predicted = predicted_speed * 25.0
            if real_speed < 2.0 and real_predicted > 3.0:
                # If (Car Stooped) and
                #  ( It should not have stopped, use the speed prediction branch for that)

                predicted_acc = 1 * (5.6 / 25.0 - speed) + predicted_acc

                predicted_brake = 0.0

                predicted_acc = predicted_acc[0][0]

        return predicted_steers, predicted_acc, predicted_brake
