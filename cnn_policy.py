#!/usr/bin/python3.8

import torch as th
import torch.nn as nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.policies import ActorCriticCnnPolicy



class BinarySegmentationCNN(BaseFeaturesExtractor):
    """Class handling binary segmented images and includes a custom CNN architecture"""

    def __init__(self, observation_space=9, features_dim=512):
        super(BinarySegmentationCNN, self).__init__(observation_space, features_dim)

        # CNN architecture
        self.cnn = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=8, stride=4, padding=0),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=0),
            nn.ReLU(),
            nn.Flatten()
        )

        # size of the output from the CNN
        with th.no_grad():
            n_flatten = self.cnn(th.as_tensor(observation_space.sample()[None]).float()).shape[1]

        # fully connected layers
        self.fc = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU()
        )


    def forward(self, observations):
        """
        forward pass function
        """
        return self.fc(self.cnn(observations))





class CnnPolicy(ActorCriticCnnPolicy):
    """Class handling custom CNN policy"""

    def __init__(self, *args, **kwargs):
        super(CnnPolicy, self).__init__(*args, **kwargs,
                                              features_extractor_class=BinarySegmentationCNN)
