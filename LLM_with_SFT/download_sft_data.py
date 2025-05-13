'''
John Ware
Final Project
21 August 2024

This will download the webglm-qa dataset from huggingface
https://huggingface.co/datasets/THUDM/webglm-qa

The final file (data.txt) is around 118MB of text.

Requires the datasets library (pip install datasets).

This is modified from the course-provided 'download_data.py' function. 
'''

from datasets import load_dataset

def download_dataset(dataset_name): 

	print('Downloading Dataset...')

	ds = load_dataset(dataset_name, split='train')
	ds = ds.remove_columns('references') # remove references column

	lines = []
	for entry in ds: #{q,a} pair
		text = str(entry)
		text = text.replace("\n", " ") # remove newline formatting
		text = " ".join(text.split()) # remove sequences of whitespace
		lines.append(text+"\n")


	f = open("data.txt", "w")
	f.writelines(lines)
	f.close()

if __name__ == "__main__":
    download_dataset("THUDM/webglm-qa")