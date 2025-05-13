John Ware
Final Project
21 August 2024

### Code Modules ###
- download_sft_dataset.py
    - Downloads the Hugging Face webglm dataset: https://huggingface.co/datasets/THUDM/webglm-qa

- construct_sft_dataset.py
    - This takes a data text file, tokenizes, formats, and outputs a 'dataset.npy' file

- train.py
    - Primary training loop. Based on the course-provided GPT-2 training loop, and modified to apply to SFT

- batch_mask.py
    - This is a custom function that takes in a batch from the model, and masks the input and output. 

- generate.py
    - This is the main file for probing the trained model.

- generate_gpt2.py
    - This imports the GPT2 model and toknizer, and allows sampling for the same input as the SFT model
    - To adjust input, users can change the 'initial_text' variable, or change the commented code around user inputs. 

- sampler.py
    - Course-provided code for sampling the model using top-p or top-k methods

- warmup_cosine.py
    -  helper to build a warmup+cosine scheudule

- main.py
    - used to run each function in order, as described in Code Flow section



### Code Flow of main.py ###
1. Ensure libraries below are installed
2. Run download_sft_dataset.py to get Hugging Face Data
3. Run construct_sft_dataset.py to generate dataset.npy file
4. Run train.py to conduct SFT and output a loss vs tokens plot
5. Run generate.py to generate SFT output for a given input question
6. Run generate_gpt2.py to generate GPT2 output for the same input string



### Additional Run Notes ###
- The train.py call is included in the main.py function, but can be commented out to speed things along. 
- Model weights / parameters are included in the submitted files
- To test the model, you can go straight to the 'generate.py' file and adjust the user input. 
- You can also use main.py and comment out the download + construct dataset functions



### Libraries / Import Commands ###
    import torch
    import numpy as np
    from transformers import AutoModelForCausalLM
    import matplotlib.pyplot as plt
    import time
    import os
    import numpy as np
    from warmup_cosine import cosine_with_warmup_lr_scheduler
    from datasets import load_dataset
    from batch_mask import batch_mask
    from tqdm import tqdm



### Outputs ###
- Loss vs tokens plot (output from train.py)
- my_custom_gpt2 folder (output from train.py)
- data.txt (output from download_sft_data.py)
- dataset.npy (output from construct_sft_data.py)

