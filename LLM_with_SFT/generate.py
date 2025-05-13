'''
John Ware
Final Project
21 August 2024

This is the main file for probing the trained model. Text can be input under 'initial_text' and this 
will output N tokens from the model. Additional inputs are included and commented out here. 

'''

import numpy as np
import torch
from tqdm import tqdm
from sampler import Sampler
from transformers import AutoModelForCausalLM
from transformers import AutoTokenizer


'''
Example of using Sampler with our own GPT model trained on wikipedia data.
'''

# make sampling algorithm
samp = Sampler(top_p=0.9, frequency_penalty=1.2)

# load our model & tokenizer, make sure these settings match whatever we trained
tokenizer = AutoTokenizer.from_pretrained("gpt2", pad_token='<|padtok|>', sep_token='<|answer|>')
model = AutoModelForCausalLM.from_pretrained("./my_custom_gpt2") #fine tuned model
model.eval() #switch to eval mode


# =================================================================

# User Input
# initial_text = "question: What is a MAC address? answer:"
# initial_text = "question: who was the first president?"
initial_text = 'What is the capital of Oregon? answer: '
# initial_text = "Why doesn't it rain salt water?"
# initial_text = "Thomas Jefferson was"

print('SFT Test Run with input: ', initial_text)

# =================================================================

token_ids = tokenizer.encode(initial_text, return_tensors='pt')[0]

# generate N more tokens. We are not using kv cache so this may be slow.
N = 50 
for i in tqdm(range(N)):

	# pass tokens through the model to get logits
	output = model(token_ids)['logits'][-1,:]

	# sample from the logits, take away batch dim
	token_ids_np = token_ids.data.cpu().numpy()#[0]
	tok = samp(output.data.cpu().numpy(), token_ids_np)

	# add the resulting token id to our list
	token_ids_np = np.append(token_ids_np, tok)
	token_ids = torch.from_numpy(token_ids_np)

	# if we generated a stop token, stop!
	if tok == tokenizer.eos_token_id:
		break


token_ids = token_ids.data.cpu().numpy()#[0]

# print out resulting ids
print(token_ids)

# print out the decoded text
print(tokenizer.decode(token_ids))
print('')