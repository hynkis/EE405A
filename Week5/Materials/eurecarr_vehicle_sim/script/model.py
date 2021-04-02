import torch
import torch.nn as nn
import torchvision
import torchvision.transforms as transforms
import pandas as pd
from sklearn.preprocessing import StandardScaler

# Fully connected neural network with one hidden layer
class NeuralNet(nn.Module):
    def __init__(self, input_size, output_size, hidden_size1=256, hidden_size2=64):
        super(NeuralNet, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size1) 
        self.relu1 = nn.ReLU()
        # self.relu1 = nn.Tanh()
        self.fc2 = nn.Linear(hidden_size1, hidden_size2)
        self.relu2 = nn.ReLU()
        # self.relu2 = nn.Tanh()
        self.fc3 = nn.Linear(hidden_size2, output_size)

    
    def forward(self, x):
        out = self.fc1(x)
        out = self.relu1(out)
        out = self.fc2(out)
        out = self.relu2(out)
        out = self.fc3(out)
        return out
