import glob
from tokenizers import CharBPETokenizer

training_file_search_expr = "data_*/*/*.txt"
vocab_size = 53
output_path = "./tokenizer.json"


# Search for all text / label files that match the glob expression
training_files = glob.glob(training_file_search_expr)

# Initialize a tokenizer
tokenizer = CharBPETokenizer(split_on_whitespace_only=True)

# Train it on the training files
tokenizer.train(training_files, vocab_size=vocab_size, min_frequency=10)

# Save it
tokenizer.save("./tokenizer.json")
