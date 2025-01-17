#!/usr/bin/env python

import numpy
from GDM_Imagine_Dimensional.episodic_gwr import EpisodicGWR
from GDM_Imagine_Dimensional import gtls
from FaceChannelUtils import imageProcessingUtil, modelDictionary, modelLoader
import numpy as np
np.seterr(divide='ignore', invalid='ignore')
import re
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # or any {'0', '1', '2'}
import tensorflow as tf
tf.get_logger().setLevel('ERROR')
from PIL import Image as PILImage
import shutil

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import cv2

import os
import sys

from datetime import datetime

parentDir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(parentDir)

from log_manager import LogManager

# changing directory to the file's directory
os.chdir(os.path.dirname(__file__))

# creating/connecting the main log file
lm = LogManager('main')

saveFrames = rospy.get_param('save_frames')
os.chdir(lm.getFileDir())
# Folders to save models
# creating frames folder if not
if not os.path.isdir('saved_GWRs'):
    os.mkdir('saved_GWRs')
gwrs_path = lm.getFileDir() + "/saved_GWRs"

if saveFrames:
    # changing directory to the log files' directory to save frames
    os.chdir(lm.getFileDir())

    # creating frames folder if not
    if not os.path.isdir('frames'):
        os.mkdir('frames')

    # creating frames folder if not
    if not os.path.isdir('faces'):
        os.mkdir('faces')
    if not os.path.isdir('faces_imagined'):
        os.mkdir('faces_imagined')

# creating log file to keep the frames' dimensional outputs
lm_arousal_valence_cl = LogManager('arousal_valence_cl')
lm_arousal_valence_cl.write('ORDER: [Arousal, Valence]')
lm_arousal_valence_cl.separate(2)

# initializing the ROS node
rospy.init_node('cl_model', anonymous=True)
lm.write('cl_model_c3.py node initialized.')

# creating bridge to convert ROS image messages to processable frames
bridge = CvBridge()

config = tf.compat.v1.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.compat.v1.Session(config=config)

# input size for the dimensional model
faceSize = (96, 96)

frameSize = (480, 640)

lm.separate()
lm.write('CL MODEL FRAME OPTIONS', False)
lm.separate(1)
lm.write('Frame size: ' + str(frameSize[0]) + 'x' + str(frameSize[1]) + '\n' +
         'Face size: ' + str(faceSize[0]) + 'x' + str(faceSize[1]))

lm.separate()

trained_GWRs = (None, None)

CAAE_LoadPath = "/home/afar/catkin_ws/src/dialogue_manager/src/CLModel/checkpoint"

def sort_nicely(l):
    """ Sort the given list in the way that humans expect.
    """

    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    l.sort(key=alphanum_key)
    return l
# --------------------------------------------------------------------
# -HELPERS------------------------------------------------------------
# --------------------------------------------------------------------
def normalize_image(img, maximum):
    """
    Normalizes all pixels to values between 0 and maximum.

    @param img: numpy array
    @param maximum: scalar value

    @return numpy array of same size as img
    """
    img /= img.max() / maximum
    return img


def save_image(img_array, path):
    """
    Saves an image from a three-dimensional numpy array under path.

    @param img_array: two-dimensional numpy array
    @param path: string
    """
    img = PILImage.fromarray(img_array.astype('uint8'))
    img = img.convert('RGB')
    img.save(path)


def get_image_array(path, image_size=None):
    """
    Reads image from file and transforms it to numpy array.

    @param path: string
    @param image_size: (optional) size of the image

    @param img_array: numpy array
    """
    if image_size == None:
        return np.asarray(PILImage.open(path)).astype(np.float32)
    return np.asarray(PILImage.open(path).resize(image_size)).astype(np.float32)


def get_generated_images(path, p=96):
    """
    Cuts generated images from image saved in network output format.

    @param path: path to image to cut generated images from.

    @return: numpy array of shape 49x96x96x3
    """
    img = get_image_array(path)
    res = []
    for r in range(7):
        for c in range(7):
            single_image = img[r * p:(r + 1) * p, p * (c + 3):p * (c + 4)]
            res.append(single_image)
    return np.asarray(res)


def tile_to_square(images):
    """
    Transforms numpy array of 49x96x96 to numpy array of 672x672 by tiling.

    @param images: numpy array of shape 49x96x96

    @return: numpy array of shape 672x672
    """
    # Build final image from components
    frame = np.zeros([96 * 7, 96 * 7])
    for index, image in enumerate(images):
        index_column = index % 7
        index_row = index // 7
        frame[(index_row * 96):((index_row + 1) * 96), (index_column * 96):((index_column + 1) * 96)] = image
    return frame


def save_generated_output(inp, generated_outp, path, valence, arousal):
    """
    Save the output generated by the network.
    """
    for i in range(len(generated_outp)):
        save_path, _ = path.split('.jp')
        if not os.path.exists(save_path):
            os.makedirs(save_path)

        save_image((generated_outp[i] + 1) * 255 / 2, save_path + "/ar_" + str(arousal[i][0]) + "_val_" + str(valence[i][0]) + ".jpg")


def load_image_as_network_input(image_path, imageSize=(96, 96), normalise=True):
    """
    Load image and normalize it to pixel values in [-1,1].

    @param image_path: path to image (string)

    @return: numpy array of size 96x96x3
    """
    image = get_image_array(image_path, image_size=imageSize)
    if normalise:
        image = normalize_image(image, 2)
        return image - 1
    else:
        return image


# --------------------------------------------------------------------
# -MAIN METHODS-------------------------------------------------------
# --------------------------------------------------------------------
#
# def save_frames(path_to_dir, path_to_out_dir):
#     from utils import imageProcessingUtil
#
#     # files = os.listdir(path_to_dir)
#     files_already = os.listdir(path_to_out_dir)
#
#     files = [f for f in os.listdir(path_to_dir) if not f in files_already]
#     for file in files:
#         open_path = path_to_dir + file
#         frame = np.asarray(PILImage.open(open_path)).astype(np.float32)
#         imageProcessing = imageProcessingUtil()
#         facePoints, face = imageProcessing.detectFace(frame)
#         if not len(face) == 0:
#             # pre-process the face
#             face = imageProcessing.preProcess(face, (96, 96))
#         save_path = path_to_out_dir + file
#         face = face * 255.0
#         save_image(face, save_path)

def generate_images(loadPath, valence, arousal, path_to_dir, path_to_out_dir):
    with tf.compat.v1.Session(config=tf.compat.v1.ConfigProto(allow_soft_placement=True)) as sess:
        # restore graph
        new_saver = tf.compat.v1.train.import_meta_graph(loadPath + '/01_model.meta')
        new_saver.restore(sess, tf.compat.v1.train.latest_checkpoint(loadPath))
        graph = tf.compat.v1.get_default_graph()

        arousal_tensor = graph.get_tensor_by_name("arousal_labels:0")
        valence_tensor = graph.get_tensor_by_name("valence_labels:0")
        images_tensor = graph.get_tensor_by_name("input_images:0")

        # randomly selecting images
        files = os.listdir(path_to_dir)[-10:]

        # load input
        files_already = os.listdir(path_to_out_dir)
        files = [f for f in files if not f in files_already]

        for file in files:
            i = load_image_as_network_input(path_to_dir + "/" + file).reshape((1, 96, 96, 3))
            query_images = np.tile(i, (49, 1, 1, 1))

            # create input for net
            feed_dict = {arousal_tensor: arousal, valence_tensor: valence, images_tensor: query_images}
            op_to_restore = sess.graph.get_tensor_by_name("generator/Tanh:0")
            # run
            x = sess.run(op_to_restore, feed_dict)
            # save
            save_generated_output(i, x, path_to_out_dir + "/" + file, valence=valence, arousal=arousal)

def preProcess(image, imageSize= (64,64)):

    image = numpy.array(image)

    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    image = numpy.array(cv2.resize(image, imageSize))

    image = numpy.expand_dims(image, axis=0)

    image = image.astype('float32')

    image /= 255

    return image
def generate_encodings_facechannel(loadPath, path_to_out_dir, mode='imagine'):
    import keras.backend.tensorflow_backend as tb
    facechannel_faceSize = (64, 64)
    tb._SYMBOLIC_SCOPE.value = True

    dataX = []
    dataY_arousal = []
    dataY_valence = []
    with tf.compat.v1.Session(config=tf.compat.v1.ConfigProto(allow_soft_placement=True)) as sess:
        # load input
        modelDimensional = modelLoader.modelLoader(modelDictionary.DimensionalModel, printSummary=False)
        if mode == 'imagine':
            files = sort_nicely(os.listdir(path_to_out_dir))
            for file in files:
                encodings = []
                arousal = []
                valence = []
                imgs = os.listdir(path_to_out_dir + file)
                for img in imgs:
                    name = img.split(".jp")[0]
                    _, ar, _, val = name.split("_")
                    arousal.append(float(ar))
                    valence.append(float(val))
                    processedFace = preProcess(load_image_as_network_input(path_to_out_dir + file + "/" + img,
                                                                                           facechannel_faceSize,
                                                                           normalise=False).reshape((64, 64, 3)),
                                                               facechannel_faceSize)
                    # get encodings
                    encodings.append(numpy.array(modelDimensional.getDense(processedFace)))
                arousal = np.array(arousal).reshape((49, 1))
                valence = np.array(valence).reshape((49, 1))
                encodings = np.array(encodings).reshape((49, 200))


                if  dataX == []:
                    dataX = encodings
                    dataY_arousal = arousal
                    dataY_valence = valence
                else:
                    dataX = np.vstack([dataX, encodings])
                    dataY_arousal = np.vstack([dataY_arousal, arousal])
                    dataY_valence = np.vstack([dataY_valence, valence])
            return dataX, np.hstack([dataY_arousal, dataY_valence])
        elif mode == 'encode':
            files = sort_nicely(os.listdir(path_to_out_dir))
            if len(files) < 90:
                to_fill = 90 - len(files)
                files = numpy.concatenate([files, [files[i] for i in numpy.random.randint(0, len(files), to_fill)]])
            else:
                files = files[-90:]
            face_time_stamps = [facename.split("_frame_")[1].split('.jp')[0] for facename in files]
            encodings = []
            for file in files:
                processedFace = preProcess(
                    load_image_as_network_input(path_to_out_dir + file,facechannel_faceSize,
                                                                           normalise=False).reshape((64, 64, 3)),
                    facechannel_faceSize)
                # get encodings
                encodings.append(numpy.array(modelDimensional.getDense(processedFace)))
            encodings = np.array(encodings).reshape((len(encodings), 200))
            return encodings, face_time_stamps
        elif mode == 'input':
            encodings = []
            arousal = []
            valence = []
            files = sort_nicely(os.listdir(path_to_out_dir))[-90:]
            for file in files:
                processedFace = preProcess(
                    load_image_as_network_input(path_to_out_dir + file, facechannel_faceSize,
                                                                           normalise=False).reshape((64, 64, 3)),
                    facechannel_faceSize)
                # get encodings
                encodings.append(numpy.array(modelDimensional.getDense(processedFace)))
                dimensions = numpy.array(modelDimensional.classify(processedFace))
                arousal.append(dimensions[0])
                valence.append(dimensions[1])
            arousal = np.array(arousal).reshape((len(arousal), 1))
            valence = np.array(valence).reshape((len(valence), 1))
            encodings = np.array(encodings).reshape((len(encodings), 200))
            return encodings, np.hstack([arousal, valence])

def generate_encodings(loadPath, path_to_out_dir, test=False):
    dataX = []
    dataY_arousal = []
    dataY_valence = []
    with tf.compat.v1.Session(config=tf.compat.v1.ConfigProto(allow_soft_placement=True)) as sess:
        # restore graph
        new_saver = tf.compat.v1.train.import_meta_graph(loadPath + '/01_model.meta')
        new_saver.restore(sess, tf.compat.v1.train.latest_checkpoint(loadPath))
        graph = tf.compat.v1.get_default_graph()

        arousal_tensor = graph.get_tensor_by_name("arousal_labels:0")
        valence_tensor = graph.get_tensor_by_name("valence_labels:0")
        images_tensor = graph.get_tensor_by_name("input_images:0")

        # load input
        files = sort_nicely(os.listdir(path_to_out_dir))[0:49]
        face_time_stamps = [facename.split("_frame_")[1].split('.jp')[0] for facename in files]

        if not test:
            for file in files:
                arousal = []
                valence = []
                query_images = []
                imgs = os.listdir(path_to_out_dir + file)
                for img in imgs:
                    name = img.split(".jp")[0]
                    _, ar, _, val = name.split("_")
                    arousal.append(float(ar))
                    valence.append(float(val))
                    query_images.append(load_image_as_network_input(path_to_out_dir + file + "/" + img).reshape((1, 96, 96, 3)))
                arousal = np.array(arousal).reshape((49, 1))
                valence = np.array(valence).reshape((49, 1))
                query_images = np.array(query_images).reshape((49, 96, 96, 3))

                # create input for net
                feed_dict = {arousal_tensor: arousal, valence_tensor: valence, images_tensor: query_images}
                # Extract Encoder output
                encoder_out = sess.graph.get_tensor_by_name("encoder/Tanh:0")
                # run
                encodings = sess.run(encoder_out, feed_dict)

                if  dataX == []:
                    dataX = encodings
                    dataY_arousal = arousal
                    dataY_valence = valence
                else:
                    dataX = np.vstack([dataX, encodings])
                    dataY_arousal = np.vstack([dataY_arousal, arousal])
                    dataY_valence = np.vstack([dataY_valence, valence])
            return dataX, np.hstack([dataY_arousal, dataY_valence])
        else:
            if len(files) < 49:
                to_fill = 49 - len(files)
                files = numpy.concatenate([files, [files[i] for i in numpy.random.randint(0, len(files), to_fill)]])
            query_images = []
            for file in files:
                query_images.append(load_image_as_network_input(path_to_out_dir + file).reshape((1, 96, 96, 3)))
            query_images = np.array(query_images).reshape((len(query_images), 96, 96, 3))

            # create input for net
            feed_dict = {images_tensor: query_images}
            encoder_out = sess.graph.get_tensor_by_name("encoder/Tanh:0")
            encodings = sess.run(encoder_out, feed_dict)
            if dataX == []:
                dataX = encodings
            else:
                dataX = np.vstack([dataX, encodings])
            return dataX, face_time_stamps
def apply_network_to_images_of_dir(loadPath, path_to_dir, path_to_out_dir):
    """
    Applies the trained network to all images found in path_to_dir for 49 emotions respectively.
    Saves the output in path_to_out_dir.

    @param path_to_dir: path to existing directory (string)
    @param path_to_out_dir: path to existing directory (string)
    """

    # valence
    valence = np.arange(0.75, -0.751, -0.25)
    # valence = np.random.uniform(-0.9, 0.9, 49).reshape((49, 1))
    valence = np.repeat(valence, 7).reshape((49, 1))

    # arousal
    arousal = [np.arange(0.75, -0.751, -0.25)]
    arousal = np.repeat(arousal, 7, axis=0)
    arousal = np.asarray([item for sublist in arousal for item in sublist]).reshape((49, 1))
    # arousal = np.random.uniform(-0.9, 0.9, 49).reshape((49, 1))

    generate_images(loadPath, valence, arousal, path_to_dir + "/", path_to_out_dir + "/")
    # return generate_encodings(loadPath, path_to_out_dir + "/")
    return generate_encodings_facechannel(loadPath, path_to_out_dir + "/", mode='imagine')



def select_frames_to_imagine(path_to_dir, path_to_out_dir, size):
    files = np.random.choice(os.listdir(path_to_dir), size, replace=True)
    for file in files:
        shutil.copyfile(path_to_dir + file, path_to_out_dir + file)


def train_GDM(data, labels, trained_GWRs):
    def replay_samples(net, size):
        samples = np.zeros(size, dtype=int)
        r_weights = np.zeros((net.num_nodes, size, net.dimension))
        r_labels = np.zeros((net.num_nodes, 2, size))
        for i in range(0, net.num_nodes):
            for r in range(0, size):
                if r == 0:
                    samples[r] = i
                else:
                    samples[r] = np.argmax(net.temporal[int(samples[r - 1]), :])
                r_weights[i, r] = net.weights[int(samples[r])][0]
                r_labels[i, 0, r] = net.alabelsValence[int(samples[r])]
                r_labels[i, 1, r] = net.alabelsArousal[int(samples[r])]
        return r_weights, r_labels

    '''
    Episodic-GWR supports multi-class neurons.
    Set the number of label classes per neuron and possible labels per class
    e.g. e_labels = [50, 10]
    is two labels per neuron, one with 50 and the other with 10 classes.
    Setting the n. of classes is done for experimental control but it is not
    necessary for associative GWR learning.
    '''
    # Setting up format for Episodic and Semantic Memory labelling
    # e_labels = [labelsize, labelsize]
    # s_labels = [labelsize]
    # Training Data
    ds_vectors = data
    # Training Labels
    ds_labels = labels
    # ds_labels = np.zeros((len(e_labels), len(dataY)))
    # ds_labels[0] = dataY
    # ds_labels[1] = dataY
    # Number of context descriptors; Set to zero for frame-based evaluations.
    num_context = 0

    a_threshold = [0.6, 0.5]
    h_thresholds = [0.2, 0.2]
    beta = 0.7
    e_learning_rates = [0.087, 0.032]

    s_learning_rates = [0.087, 0.032]

    context = True

    # Initialising Episodic and Semantic Memory GWR Models.
    if trained_GWRs[0] is None:
        # Initialisint Episodic Memory
        g_episodic = EpisodicGWR()
        # Higher Max-nodes and lower age allow for faster learning with pattern-separated representations.
        g_episodic.init_network(data, ds_labels, num_context, max_nodes=len(data), age=600)
        # Initialising Semantic Memory
        g_semantic = EpisodicGWR()
        # Lower Max-nodes and higher age allow for slower learning with pattern-complete representations.
        g_semantic.init_network(data, ds_labels, num_context, max_nodes=len(data)//2, age=1200)

    else:
        # Loading trained models for subsequent training runs.
        g_episodic, g_semantic = trained_GWRs

    """ Incremental training hyper-parameters """

    # Epochs per sample for incremental learning
    epochs = 5
    epochs_replay = 1
    # Initialising experienced episodes to Zero.
    n_episodes = 0
    # Number of samples per epoch
    batch_size = 20

    # Replay parameters; With num_context = 0, RNATs size set to 1, that is, only looking at previous BMU.
    # Size of RNATs
    replay_size = (num_context * 2) + 1
    replay_weights = []
    replay_labels = []

    """##############################################################################################"""
    """ ##################################  Logging Parameters  #####################################"""
    """##############################################################################################"""

    # lm.write("GDM Parameters LOG")
    # lm.write("Number of Epochs: " + str(epochs))
    # lm.write("Number of Contexts: " + str(num_context))
    # lm.write("Activation Thresholds: [" + str(a_threshold[0]) + ", " + str(a_threshold[1]) + "]")
    # lm.write("Habituation Thresholds: [" + str(h_thresholds[0]) + ", " + str(h_thresholds[1]) + "]")
    # lm.write("Episodic lr: [" + str(e_learning_rates[0]) + ", " + str(e_learning_rates[1]) + "]")
    # lm.write("Semantic lr: [" + str(s_learning_rates[0]) + ", " + str(s_learning_rates[1]) + "]")
    # lm.write("Batch Size: " + str(batch_size))
    # lm.write("GDM Parameters LOG")


    """##############################################################################################"""
    """ ############################   Running Training of Memories  ################################"""
    """##############################################################################################"""

    for s in range(0, ds_vectors.shape[0], batch_size):
        # lm.write("Training Episodic Regular")
        g_episodic.train_egwr(ds_vectors[s:s + batch_size],
                              ds_labels[s:s + batch_size],
                              epochs, a_threshold[0], beta, e_learning_rates,
                              context, hab_threshold=h_thresholds[0], regulated=0)
        e_weights, e_labels = g_episodic.test_av(ds_vectors[s:s + batch_size], ds_labels[s:s + batch_size],
                                                         test_accuracy=False)
        # lm.write("Training Semantic Regular")
        g_semantic.train_egwr(e_weights, ds_labels[s:s + batch_size],
                              epochs, a_threshold[1], beta, s_learning_rates,
                              context=False, hab_threshold=h_thresholds[1], regulated=1)

        # # Running Pseudo-Replay  #######################################"""
        # if n_episodes > 0:
        #     # Replay pseudo-samples
        #     # lm.write("Training Episodic Replay")
        #     # Episodic Pseudo-Replay
        #     g_episodic.train_egwr(replay_weights, replay_labels,
        #                           epochs_replay, a_threshold[0], beta,
        #                           e_learning_rates, context=False, hab_threshold=h_thresholds[0], regulated=0)
        #     # lm.write("Training Semantic Replay")
        #     # Semantic Pseudo-Replay
        #     g_semantic.train_egwr(replay_weights, replay_labels,
        #                           epochs_replay, a_threshold[1], beta,
        #                               s_learning_rates, context=False, hab_threshold=h_thresholds[1], regulated=1)
        #
        #
        # # Generating Pseudo-Samples
        # replay_weights, replay_labels = replay_samples(g_episodic, replay_size)
        # replay_weights = numpy.squeeze(replay_weights)
        # replay_labels = numpy.squeeze(replay_labels)
        # n_episodes += 1

    # Evaluation with only Current Data
    e_weights, e_labels = g_episodic.test_av(ds_vectors, ds_labels, test_accuracy=False)
    s_weights, s_labels = g_semantic.test_av(e_weights, ds_labels, test_accuracy=False)

    return (g_episodic, g_semantic)


def annotate_GDM(trained_GWRs, test_data):
    g_episodic, g_semantic = trained_GWRs

    # Annotations from Episodic
    e_arousals, e_valences = g_episodic.annotate(test_data)

    # Annotations from Semantic
    s_arousals, s_valences = g_semantic.annotate(test_data)

    return numpy.array([e_arousals, e_valences]).reshape((len(e_arousals)), 2), \
           numpy.array([s_arousals, s_valences]).reshape((len(s_arousals)), 2)

def unison_shuffled_copies(a, b):
    assert len(a) == len(b)
    p = numpy.random.permutation(len(a))
    return a[p], b[p]

def callback(data):
    invalid_states = ["NONE", "SURVEY"]
    if saveFrames and str(rospy.get_param('current_state')) not in invalid_states:
        output_trained_GWRs = None
        imaginedFaceDir = os.path.join(lm.getFileDir(), 'faces_imagined/')

        if str(rospy.get_param('current_state')) not in os.listdir(imaginedFaceDir):
            if not os.path.exists(
                    os.path.join(os.path.join(lm.getFileDir(), 'faces_imagined'),
                                 str(rospy.get_param('current_state')))):
                os.mkdir(
                    os.path.join(os.path.join(lm.getFileDir(), 'faces_imagined'),
                                 str(rospy.get_param('current_state'))))

        if len(os.listdir(gwrs_path)) > 0:
            input_trained_GWRs = []
            input_trained_GWRs.append(gtls.import_network(file_name=gwrs_path + "/GDM_E", NetworkClass=EpisodicGWR))
            input_trained_GWRs.append(gtls.import_network(file_name=gwrs_path + "/GDM_S", NetworkClass=EpisodicGWR))
        else:
            input_trained_GWRs = (None, None)

        # Training with Imagined Data Every Interaction

        # Imagining Faces
        imagined_data, imagined_labels = apply_network_to_images_of_dir(loadPath=CAAE_LoadPath,
                                                                        path_to_dir=os.path.join(
                                                                            os.path.join(lm.getFileDir(), 'faces'),
                                                                            str(rospy.get_param('current_state'))),
                                                                        path_to_out_dir=os.path.join(
                                                                            os.path.join(lm.getFileDir(),
                                                                                         'faces_imagined'),
                                                                            str(rospy.get_param('current_state'))))
        # Encode Input data
        input_data, input_labels = generate_encodings_facechannel(CAAE_LoadPath,
                                                                  os.path.join(os.path.join(lm.getFileDir(), 'faces'), str(rospy.get_param('current_state'))) + "/",
                                                                  mode='input')
        training_data = numpy.vstack([imagined_data, input_data])
        training_labels = numpy.vstack([imagined_labels, input_labels])
        # training_data, training_labels = unison_shuffled_copies(training_data, training_labels)

        lm.write("Running GDM Training for " + str(rospy.get_param('current_state')), printText=True)
        output_trained_GWRs = train_GDM(data=training_data, labels=training_labels,
                                              trained_GWRs=input_trained_GWRs)
        # Annotate User Data

        # encodings, face_time_stamps = generate_encodings(CAAE_LoadPath,
        #                                os.path.join(os.path.join(lm.getFileDir(), 'faces'), str(rospy.get_param('current_state'))) + "/",
        #                                test=True)
        encodings, face_time_stamps = generate_encodings_facechannel(CAAE_LoadPath,
                                       os.path.join(os.path.join(lm.getFileDir(), 'faces'), str(rospy.get_param('current_state'))) + "/",
                                       mode="encode")
        # if output_trained_GWRs is None:
        #     output_trained_GWRs = []
        #     output_trained_GWRs.append(gtls.import_network(file_name=gwrs_path + "/GDM_E", NetworkClass=EpisodicGWR))
        #     output_trained_GWRs.append(gtls.import_network(file_name=gwrs_path + "/GDM_S", NetworkClass=EpisodicGWR))
        if str(rospy.get_param('current_state')) not in invalid_states:
            episodic_results, semantic_results = annotate_GDM(output_trained_GWRs, encodings)
            for r in range(len(face_time_stamps)):
                text = face_time_stamps[r].replace('::', '_') + " - " + str(numpy.array(episodic_results[r]).reshape((2, 1, 1))).replace('\n', '')
                lm_arousal_valence_cl.write(text, dateTime=False, printText=False)
                if rospy.get_param('arousal_valence_cl') != 'NONE':
                    rospy.set_param('arousal_valence_cl', rospy.get_param('arousal_valence_cl') + [text])
                else:
                    rospy.set_param('arousal_valence_cl', [text])
        if not os.path.exists(gwrs_path):
            os.makedirs(gwrs_path)
        gtls.export_network(gwrs_path + "/GDM_E", output_trained_GWRs[0])
        gtls.export_network(gwrs_path + "/GDM_S", output_trained_GWRs[1])
        rospy.set_param('imagination_running_flag', str(int(rospy.get_param('imagination_running_flag')) + 1))

def listener():
    rospy.Subscriber('camera_channel', Image, callback)
    lm.write('CL_Model subscriber is ready - [camera_channel] topic')

    rospy.spin()


if __name__ == '__main__':
    listener()
    lm_arousal_valence_cl.close()