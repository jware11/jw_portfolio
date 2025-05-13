'''
John Ware
Final Project
21 August 2024

This script imports and runs all applicable code blocks for the SFT portion of this project.  

'''

from download_sft_data import download_dataset
from construct_sft_dataset import construct_dataset


#download dataset
download_dataset('THUDM/webglm-qa')

construct_dataset("./data.txt", 256)

# Run Training Loop until complete, ~8-12 hours - commented out for ease
# exec(open('train.py').read())

# Run SFT Generate
exec(open('generate.py').read())

# Run GPT2 Generate
exec(open('generate_gpt2.py').read())