'''
John Ware
Final Project
21 August 2024

This is a custom function that takes in a batch from the model, and masks the input and output. 
The format of the training dataset is 'question': {question text} 'answer': {answer text}

This masks the question (x) with all ignore_tokens (-100) after the question text, and 
masks the answer with the same token before the answer text. 

ex: 
    original text = 'question': {question text} 'answer': {answer text}
    inp = 'question': {question text} <sep> 'answer': {answer text}
    targ = -100 -100 -100 -100 -100 'answer': {answer text}

This is a Q&A style mask - returns x (question) and y (answer) masked
token values are set from gpt2 and custom tokenizer inputs in construct_sft_dataset

'''

import torch
import numpy as np

def batch_mask(batch, eos_tok=50256, pad_tok=-999, sep_tok=50257, ignore_tok=-100):
    
    special_pad = 50258 # pad for input, to avoid passing negative value into model

    new_batch = [] #init
    inp, targ = [], []
    for item in batch: 
        x = item.clone() #input, question
        y = item.clone() #target, answer
        answer_reached = False

        for i in range(len(item)): #step through each value
            if answer_reached == True: #nothing changed
                # x[i] = ignore_tok
                pass
            else: #answer_reached == False - front of answer (targ) masked
                y[i] = ignore_tok

            if item[i] == eos_tok: 
                pass #keep eos tokens

            elif item[i] == pad_tok: 
                x[i] = special_pad
                y[i] = ignore_tok

            elif item[i] == sep_tok: 
                answer_reached = True
                # x[i] = ignore_tok #ignore separation in question

        new_batch.append([x, y])
        inp.append(x)
        targ.append(y)

    #convert lists of tensors to tensor of tensors
    inp = torch.stack(inp, dim=0)
    targ = torch.stack(targ, dim=0)

    #print to check
    # print('input: ', inp)
    # print('target: ', targ)

    return inp, targ

