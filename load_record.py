# -*- coding: utf-8 -*-
"""
Created on Thu May 30 16:29:50 2024

@author: yueminding
"""

import pickle

# Path to the pickle file
file_path = 'data_record/2024-05-30-17-37-50-4.pkl'
#file_path = 'data_record/2024-05-30-17-19-15-820.pkl'


# Load the pickle file
with open(file_path, 'rb') as file:
    data = pickle.load(file)