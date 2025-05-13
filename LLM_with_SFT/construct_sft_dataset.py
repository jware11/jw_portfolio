'''
John Ware
Final Project
21 August 2024

This takes a data text file, tokenizes, formats, and outputs a 'dataset.npy' file

This is modified from the course-provided 'construct_dataset.py' function. 
'''

import torch
from tqdm import tqdm
import numpy as np
from transformers import AutoTokenizer


def construct_dataset(data_txt_file, sequence_length=256):

    print('Constructing dataset...')

    # construct tokenizer
    tokenizer = AutoTokenizer.from_pretrained("gpt2", sep_token='<|answer|>')
    pad_tok = -999
    sep_tok = 50257

    # tokenize the text and add <eos> at the env of each sample
    f = open(data_txt_file, "r")
    lines = f.readlines()
    tokenized_samples = []
    
    for line in tqdm(lines):
        # remove newlines
        line = line.replace("\n", "")
        line += tokenizer.eos_token #add eos, 50256

        #find answer index
        a_ind = line.find('answer') # starting index of the answer
        line = line[: a_ind] + tokenizer.sep_token + line[a_ind :]  #add in answer token (sep)

        tok = (tokenizer(line)["input_ids"]) #tokenize

        # padding after tokenized
        while len(tok) < sequence_length: 
            tok.append(pad_tok) #padding #50257

        if len(tok) > sequence_length: 
            continue #skip this entry - too long

        tokenized_samples.append(np.array(tok))

    data = tokenized_samples
    

    ### Optional Prints ###
    # print('data size:')
    # print('len data: ', len(data))
    # print('len[0] data: ', len(data[0]))
    # print('data example: ')
    # print(data[0])

    # shuffle
    np.random.shuffle(data)

    # save the tokenized and sequenced data
    with open('dataset.npy', 'wb') as f:
        np.save(f, data)

    print('')

if __name__ == "__main__":
    construct_dataset("./data.txt", 256)


