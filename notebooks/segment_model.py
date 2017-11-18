import numpy as np
import cv2
import keras
from scipy.ndimage.measurements import label

import matplotlib.image as mpimg
import matplotlib.pyplot as plt

from keras.models import Model
from keras.layers.core import Layer, Dense, Dropout, Activation, Flatten, Reshape, Permute
from keras.layers.convolutional import Conv2D, MaxPooling2D, UpSampling2D, Cropping2D
from keras.layers.normalization import BatchNormalization
from keras.layers.merge import Concatenate
from keras.layers import Input, Conv2D, Conv2DTranspose
from keras import backend as K
from keras.regularizers import l2


def dice_coef(y_true, y_pred):
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)
    return (2. * intersection + 1) / (K.sum(y_true_f) + K.sum(y_pred_f) + 1)


def dice_coef_loss(y_true, y_pred):
    return -dice_coef(y_true, y_pred)


# left = minus, right = plus
def get_steering_from_prediction(prediction, original=None):
    prediction[prediction >= 0.9] = 1
    prediction[prediction <= 0.9] = 0
    labels = label(prediction[:, :, 0])
    positive_gradients = []
    negative_gradients = []

    img = None
    if original is not None:
        img = original.copy()

    for object_no in range(1, labels[1] + 1):
        # Find pixels with each car_number label value
        nonzero = (labels[0] == object_no).nonzero()
        # Identify x and y values of those pixels
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        if ((np.max(nonzeroy) - np.min(nonzeroy) > 10) & (np.max(nonzerox) - np.min(nonzerox) > 10)):
            fit = np.polyfit(nonzeroy, nonzerox, 1)
            # gradients are reversed because the image starts with 0 at the top
            if (fit[0] > 0):
                negative_gradients.append(-fit[0])
            else:
                positive_gradients.append(-fit[0])
            if original is not None:
                bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
                cv2.rectangle(img, bbox[0], bbox[1], (0, 0, 255), 1)

    # add small value to avoid nan
    if len(positive_gradients) == 0:
        positive_gradients.append(1e-3)
    if len(negative_gradients) == 0:
        negative_gradients.append(1e-3)

    if original is not None:
        print(positive_gradients, negative_gradients)
        print(np.mean(positive_gradients), np.mean(negative_gradients))
        figure = plt.figure()
        plt.imshow(img, cmap='gray')
    return np.mean(positive_gradients) + np.mean(negative_gradients)


class Tiramisu():
    def __init__(self, load=False):
        if load:
            self.load()
        else:
            self.create()

    def DenseBlock(self, x, nb_layers, filters, name="dense_block"):
        layers = [x]
        for i in range(nb_layers):
            x = BatchNormalization(axis=1, gamma_regularizer=l2(0.0001), beta_regularizer=l2(0.0001))(x)
            x = Activation('relu')(x)
            x = Conv2D(filters, kernel_size=(3, 3), padding='same',
                       name=name + "_conv_layer_" + str(i), kernel_initializer="he_uniform")(x)
            x = Dropout(0.2)(x)
            layers.append(x)
            x = Concatenate(name=name + "_concatenate_" + str(i))(layers)
        # filters += growth_rate
        return x

    def TransitionDown(self, x, filters, name="transition_down"):
        x = BatchNormalization(axis=1, gamma_regularizer=l2(0.0001), beta_regularizer=l2(0.0001))(x)
        x = Activation('relu')(x)
        x = Conv2D(filters, kernel_size=(1, 1), padding='same',
                   name=name + "_conv", kernel_initializer="he_uniform")(x)
        x = Dropout(0.2)(x)
        x = MaxPooling2D(pool_size=(2, 2), strides=(2, 2), name=name + "_pool")(x)
        return x

    def TransitionUp(self, x, filters, skip_connection, name="transition_up"):
        x = Conv2DTranspose(100, kernel_size=(3, 3), strides=(2, 2), padding='same',
                            name=name + "_conv", kernel_initializer="he_uniform")(x)
        x = Concatenate(name=name + "_concatenate")([x, skip_connection])
        return x

    def create(self):
        model_input = Input(shape=(224, 224, 3))
        first_layer = Conv2D(12, kernel_size=(3, 3), padding='same',
                             kernel_initializer="he_uniform",
                             kernel_regularizer=l2(0.0001), name="first_layer")(model_input)

        nb_layers = 2
        nb_filters = 12
        growth_rate = 12
        # total_filters = input + nb_layers*nb_filters

        filters_1 = 12 + nb_layers * nb_filters  # 12 + 2 * 12 = 36
        block_1 = self.DenseBlock(first_layer, nb_layers, nb_filters, name="block1")
        down = self.TransitionDown(block_1, filters_1, name="block1_TD")

        filters_2 = filters_1 + nb_layers * nb_filters  # 36 + 2 * 12 = 60
        block_2 = self.DenseBlock(down, nb_layers, nb_filters, name="block2")
        down = self.TransitionDown(block_2, filters_2, name="block2_TD")

        filters_3 = filters_2 + nb_layers * nb_filters  # 60 + 2 * 12 = 84
        block_3 = self.DenseBlock(down, nb_layers, nb_filters, name="block3")
        down = self.TransitionDown(block_3, filters_3, name="block3_TD")

        filters_4 = filters_3 + nb_layers * nb_filters  # 84 + 2 * 12 = 108
        block_4 = self.DenseBlock(down, nb_layers, nb_filters, name="block4")

        up = self.TransitionUp(block_4, filters_4, block_3, name="TU")  # 108 + 84  = 192

        filters_up_1 = filters_4 + filters_3 + nb_layers * nb_filters  # 192 + 2 * 12 = 216
        block_3 = self.DenseBlock(up, nb_layers, nb_filters, name="block3_up")
        up = self.TransitionUp(block_3, filters_up_1, block_2, name="block3_TU")

        filters_up_2 = filters_up_1 + filters_2 + nb_layers * nb_filters  # 216 + 60 + 2 * 12  = 300
        block_2 = self.DenseBlock(up, nb_layers, nb_filters, name="block2_up")
        up = self.TransitionUp(block_2, filters_up_2, block_1, name="block2_TU")

        filters_up_1 = filters_up_2 + filters_1 + nb_layers * nb_filters  # 300 + 36 + 2 * 12 = 360
        block_1 = self.DenseBlock(up, nb_layers, nb_filters, name="block1_up")

        last_layer = Conv2D(1, kernel_size=(3, 3), padding='same',
                            kernel_initializer="he_uniform",
                            kernel_regularizer=l2(0.0001), name="last_layer")(block_1)
        out = Activation('sigmoid')(last_layer)

        self.model = Model(inputs=[model_input], outputs=[out])
        # self.model.summary()

    def save(self):
        #         self.model.save("tiramisu_model.h5", overwrite=True)
        self.model.save_weights("tiramisu_weights.h5", overwrite=True)

    def load(self):
        self.create()
        self.model.load_weights("tiramisu_weights.h5")

    def predict(self, images):
        return self.model.predict(images)

    def predict_one(self, image):
        return self.model.predict(np.expand_dims(image, axis=0))[0]


if __name__ == "__main__":
    tiramisu = Tiramisu(load=True)

    image = cv2.cvtColor(cv2.imread("sample_dataset/image/image_182_0.4794_0.5000.png"), cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, (224, 224))

    import time
    print("First Prediction")
    t = time.time()
    prediction = tiramisu.predict_one(image)
    steering = get_steering_from_prediction(prediction)
    print("time take:", time.time() - t)
    print("steering value:", steering)

    print("Second Prediction")
    image = cv2.cvtColor(cv2.imread("sample_dataset/image/image_210_0.4794_0.5000.png"), cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, (224, 224))
    t = time.time()
    prediction = tiramisu.predict_one(image)
    steering = get_steering_from_prediction(prediction)
    print("time take:", time.time() - t)
    print("steering value:", steering)

    print("all done")